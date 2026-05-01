#!/usr/bin/env python3
"""
双层检测测试脚本：车辆检测 → ROI 提取 → 装甲板检测
使用 ONNX 模型 (OpenCV DNN 推理)

用法:
    python test_pipeline.py --image test.jpg
    python test_pipeline.py --image test.jpg --car weights/car.onnx --armor weights/armor.onnx
"""

import argparse
import cv2
import numpy as np
from pathlib import Path


# ========== 模型配置 ==========
CAR_CLASSES   = ["car", "armor", "ignore", "watcher", "base"]
ARMOR_CLASSES = ["B1", "B2", "B3", "B4", "B5", "B7", "R1", "R2", "R3", "R4", "R5", "R7"]

CAR_CONF    = 0.25   # 车辆检测置信度阈值
CAR_NMS_IOU = 0.45   # 车辆 NMS IoU 阈值
ARMOR_CONF    = 0.25   # 装甲板检测置信度阈值
ARMOR_NMS_IOU = 0.30   # 装甲板 NMS IoU 阈值 (装甲板更密集, 阈值设低些)


# ========== 后处理 (与 C++ inference.cpp 对齐) ==========

def sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def letterbox(img: np.ndarray, target_size: int = 640):
    """等比例缩放 + 填充到正方形, 返回 (image, pad_x, pad_y, scale)"""
    h, w = img.shape[:2]
    scale = min(target_size / max(w, 1), target_size / max(h, 1))
    new_w = max(int(w * scale), 1)
    new_h = max(int(h * scale), 1)
    resized = cv2.resize(img, (new_w, new_h))
    pad_x = (target_size - new_w) // 2
    pad_y = (target_size - new_h) // 2
    canvas = np.zeros((target_size, target_size, 3), dtype=np.uint8)
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized
    return canvas, pad_x, pad_y, scale


def nms(boxes: np.ndarray, scores: np.ndarray, iou_thresh: float) -> list:
    """返回保留的索引列表"""
    idxs = cv2.dnn.NMSBoxes(
        bboxes=boxes.tolist(),
        scores=scores.tolist(),
        score_threshold=0.0,
        nms_threshold=iou_thresh,
    )
    if len(idxs) == 0:
        return []
    return idxs.flatten().tolist()


def postprocess(output: np.ndarray, classes: list, conf_thresh: float,
                 nms_iou: float, input_size: int, pad_x: int, pad_y: int,
                 scale: float, orig_w: int, orig_h: int) -> list:
    """
    解析 YOLO 输出 [1, 5+N_cls, N_pred] → 检测框列表
    与 C++ Inference_trt::runInference_TensorRT 后处理逻辑一致

    返回: [{box: [x,y,w,h], class_name, confidence}, ...]
    """
    # squeeze batch dim → [elem_per_pred, num_preds]
    if output.ndim == 3:
        output = output[0]
    # 现在 output 是 [elem_per_pred, num_preds], field-major 布局
    elem_per_pred = output.shape[0]
    num_preds = output.shape[1]
    num_classes = elem_per_pred - 5

    detections = []
    for i in range(num_preds):
        raw_obj = output[4, i]
        conf_obj = sigmoid(raw_obj)
        if conf_obj < conf_thresh:
            continue

        cx = float(output[0, i])
        cy = float(output[1, i])
        w  = float(output[2, i])
        h  = float(output[3, i])

        # 归一化检测: 如果坐标在 [-1, 1] 范围，反归一化到 input_size
        if abs(cx) <= 1.01 and abs(cy) <= 1.01 and abs(w) <= 1.01 and abs(h) <= 1.01:
            cx *= input_size
            cy *= input_size
            w  *= input_size
            h  *= input_size

        x1 = cx - w * 0.5
        y1 = cy - h * 0.5
        x2 = cx + w * 0.5
        y2 = cy + h * 0.5

        # 分类 (如果有 class logits)
        best_class = 0
        if num_classes > 0:
            # 取 argmax 并合并 objectness × class_prob
            cls_scores = sigmoid(output[5:5 + num_classes, i])
            best_class = int(np.argmax(cls_scores))
            combined_conf = conf_obj * float(cls_scores[best_class])
            if combined_conf < conf_thresh:
                continue

        # de-letterbox
        x1 = (x1 - pad_x) / scale
        y1 = (y1 - pad_y) / scale
        x2 = (x2 - pad_x) / scale
        y2 = (y2 - pad_y) / scale

        # clamp 到原图范围
        x1 = max(0.0, min(x1, orig_w - 1))
        y1 = max(0.0, min(y1, orig_h - 1))
        x2 = max(0.0, min(x2, orig_w - 1))
        y2 = max(0.0, min(y2, orig_h - 1))

        bw, bh = x2 - x1, y2 - y1
        if bw <= 1 or bh <= 1:
            continue

        cls_name = classes[best_class] if best_class < len(classes) else f"cls_{best_class}"
        detections.append({
            "box": [int(round(x1)), int(round(y1)), int(round(bw)), int(round(bh))],
            "class_name": cls_name,
            "confidence": float(conf_obj),
        })

    # NMS
    if detections:
        boxes_np = np.array([d["box"] for d in detections], dtype=np.int32)
        scores_np = np.array([d["confidence"] for d in detections], dtype=np.float32)
        keep = nms(boxes_np, scores_np, nms_iou)
        detections = [detections[i] for i in keep]

    return detections


# ========== ONNX 推理 ==========

class YOLO_ONNX:
    """单层 YOLO ONNX 推理器"""

    def __init__(self, onnx_path: str, classes: list, conf_thresh: float,
                 nms_iou: float, input_size: int = 640):
        self.classes = classes
        self.conf_thresh = conf_thresh
        self.nms_iou = nms_iou
        self.input_size = input_size

        if not Path(onnx_path).exists():
            raise FileNotFoundError(f"ONNX model not found: {onnx_path}")

        self.net = cv2.dnn.readNetFromONNX(onnx_path)
        # 尝试 CUDA 后端
        try:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
            print(f"[INFO] {Path(onnx_path).name}: CUDA backend enabled")
        except Exception:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            print(f"[INFO] {Path(onnx_path).name}: CPU backend")

    def detect(self, img: np.ndarray) -> list:
        """
        对任意尺寸图像推理，返回检测列表

        参数:
            img: BGR 图像 (任意尺寸)
        返回:
            [{box:[x,y,w,h], class_name, confidence}, ...]
        """
        orig_h, orig_w = img.shape[:2]

        # 预处理
        lb_img, pad_x, pad_y, scale = letterbox(img, self.input_size)
        blob = cv2.dnn.blobFromImage(lb_img, 1.0 / 255.0, (self.input_size, self.input_size),
                                     mean=(0, 0, 0), swapRB=True, crop=False)
        # 推理
        self.net.setInput(blob)
        output = self.net.forward()

        # 后处理
        return postprocess(
            output=output,
            classes=self.classes,
            conf_thresh=self.conf_thresh,
            nms_iou=self.nms_iou,
            input_size=self.input_size,
            pad_x=pad_x,
            pad_y=pad_y,
            scale=scale,
            orig_w=orig_w,
            orig_h=orig_h,
        )


# ========== 可视化 ==========

def draw_results(img: np.ndarray, car_dets: list, armor_map: dict) -> np.ndarray:
    """
    绘制双层检测结果
    car_dets:  车辆检测列表
    armor_map: {car_idx: [armor_det, ...]}
    """
    vis = img.copy()

    for i, car in enumerate(car_dets):
        bx, by, bw, bh = car["box"]
        # 车辆框 — 绿色
        cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), (0, 255, 0), 2)
        cv2.putText(vis, f"car {car['confidence']:.2f}",
                    (bx, by - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 装甲板
        for armor in armor_map.get(i, []):
            ax, ay, aw, ah = armor["box"]
            # 装甲板坐标是 ROI 相对坐标，转换到原图
            abs_x = bx + ax
            abs_y = by + ay
            # 装甲板框 — 蓝色
            cv2.rectangle(vis, (abs_x, abs_y), (abs_x + aw, abs_y + ah), (255, 0, 0), 2)
            label = f"{armor['class_name']} {armor['confidence']:.2f}"
            cv2.putText(vis, label, (abs_x, abs_y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            # 中心点 — 红色
            cx = abs_x + aw // 2
            cy = abs_y + ah
            cv2.circle(vis, (cx, cy), 3, (0, 0, 255), -1)

    return vis


# ========== 主流程 ==========

def main():
    parser = argparse.ArgumentParser(description="双层 YOLO ONNX 检测测试")
    parser.add_argument("--image", type=str, required=True, help="测试图片路径")
    parser.add_argument("--car", type=str, default="car.onnx", help="车辆检测 ONNX 路径")
    parser.add_argument("--armor", type=str, default="armor.onnx", help="装甲板检测 ONNX 路径")
    parser.add_argument("--input-size", type=int, default=640, help="模型输入尺寸")
    parser.add_argument("--car-conf", type=float, default=CAR_CONF, help="车辆检测置信度阈值")
    parser.add_argument("--car-nms", type=float, default=CAR_NMS_IOU, help="车辆 NMS IoU 阈值")
    parser.add_argument("--armor-conf", type=float, default=ARMOR_CONF, help="装甲板检测置信度阈值")
    parser.add_argument("--armor-nms", type=float, default=ARMOR_NMS_IOU, help="装甲板 NMS IoU 阈值")
    parser.add_argument("--min-roi", type=int, default=10, help="ROI 最小边长 (px)")
    parser.add_argument("--output", type=str, default="result.jpg", help="输出图片路径")
    parser.add_argument("--verbose", "-v", action="store_true", help="打印每个 ROI 的检测详情")
    args = parser.parse_args()

    # 加载图片
    img = cv2.imread(args.image)
    if img is None:
        raise FileNotFoundError(f"Image not found: {args.image}")
    print(f"[INFO] Image: {img.shape[1]}×{img.shape[0]}")

    # 加载模型
    car_model   = YOLO_ONNX(args.car,   CAR_CLASSES,   args.car_conf,   args.car_nms,   args.input_size)
    armor_model = YOLO_ONNX(args.armor, ARMOR_CLASSES, args.armor_conf, args.armor_nms, args.input_size)

    # ===== 第一层: 车辆检测 =====
    print("[STAGE 1] Car detection...")
    car_dets = car_model.detect(img)
    print(f"  → Found {len(car_dets)} cars")

    # ===== 第二层: ROI 装甲板检测 =====
    print("[STAGE 2] Armor detection on each ROI...")
    armor_map = {}  # {car_index: [armor_det, ...]}

    for i, car in enumerate(car_dets):
        bx, by, bw, bh = car["box"]
        if bw < args.min_roi or bh < args.min_roi or bx < 0 or by < 0:
            continue
        if bx + bw > img.shape[1] or by + bh > img.shape[0]:
            continue

        roi = img[by:by + bh, bx:bx + bw].copy()
        armor_dets = armor_model.detect(roi)

        if armor_dets:
            armor_dets.sort(key=lambda d: (d["class_name"], -d["confidence"]))
            unique = []
            seen = set()
            for d in armor_dets:
                if d["class_name"] not in seen:
                    seen.add(d["class_name"])
                    unique.append(d)
            armor_dets = unique

        if armor_dets:
            armor_map[i] = armor_dets
            if args.verbose:
                names = [d["class_name"] for d in armor_dets]
                print(f"  Car[{i}] @ ({bx},{by},{bw}×{bh}): {names}")

        # 进度条 (每处理 50 个或最后一个输出)
        if args.verbose or (i + 1) % 50 == 0 or i == len(car_dets) - 1:
            armor_count = sum(len(v) for v in armor_map.values())
            print(f"\r  [{i + 1}/{len(car_dets)}] ROIs processed, {armor_count} armor plates found", end="")

    print()  # 换行

    # ===== 可视化 =====
    vis = draw_results(img, car_dets, armor_map)
    cv2.imwrite(args.output, vis)
    print(f"\n[DONE] Result saved to: {args.output}")

    # 打印统计
    total_armor = sum(len(v) for v in armor_map.values())
    print(f"[STATS] Cars: {len(car_dets)} | Armor plates: {total_armor}")

    # 显示结果 (自动缩放到适合屏幕)
    screen_w, screen_h = 1920, 1080
    h, w = vis.shape[:2]
    if w > screen_w or h > screen_h:
        ds_scale = min(screen_w / w, screen_h / h)
        vis_display = cv2.resize(vis, None, fx=ds_scale, fy=ds_scale)
    else:
        vis_display = vis

    cv2.namedWindow("Detection Result", cv2.WINDOW_NORMAL)
    cv2.imshow("Detection Result", vis_display)
    print("[INFO] Displaying result — press any key to close...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
