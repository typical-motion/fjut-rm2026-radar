#!/usr/bin/env python3
"""
YOLOv11 / YOLOv8 .pt → .onnx 导出脚本
(ultralytics 统一 API，v5/v8/v11 均适用)

用法:
    python export_onnx.py --weights model.pt --imgsz 640 --opset 11

参数:
    --weights : 原始 .pt 权重路径 (训练时 1920×1920，导出时自动适配 640×640)
    --imgsz   : 导出 ONNX 时的输入尺寸 (默认 640)
    --opset   : ONNX opset 版本 (默认 11, TensorRT 推荐 11~17)
    --half    : 导出 FP16 模型 (可选)
    --simplify: 使用 onnx-simplifier 简化模型图 (可选)
    --nms     : 导出端到端含 NMS 的模型 (可选，但 TRT 后处理不需要)
"""

import argparse
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser(description="YOLOv8 .pt → .onnx export")
    parser.add_argument("--weights", type=str, required=True, help="Path to .pt weights file")
    parser.add_argument("--imgsz", type=int, default=640, help="Export input size (default: 640)")
    parser.add_argument("--opset", type=int, default=11, help="ONNX opset version (default: 11)")
    parser.add_argument("--half", action="store_true", help="Export FP16 model")
    parser.add_argument("--simplify", action="store_true", help="Apply onnx-simplifier")
    parser.add_argument("--nms", action="store_true", help="Export with end-to-end NMS")
    parser.add_argument("--batch", type=int, default=1, help="Batch size (default: 1)")
    args = parser.parse_args()

    print(f"[INFO] Loading model: {args.weights}")
    model = YOLO(args.weights)

    # pt模型在1920上训练，导出时imgsz设为640，
    # YOLO会自动处理grid/anchor的重计算
    export_kwargs = dict(
        format="onnx",
        imgsz=args.imgsz,
        opset=args.opset,
        batch=args.batch,
        half=args.half,
        simplify=args.simplify,
        # 关键: 不做 end2end NMS，输出原始 [1, 5+Ncls, 8400] 格式
        # C++ 后处理代码(TRT)依赖这个格式
        nms=args.nms,
    )

    print(f"[INFO] Exporting with imgsz={args.imgsz}, opset={args.opset}")
    if args.half:
        print("[INFO] FP16 mode enabled")
    if args.nms:
        print("[WARN] NMS export: 输出格式为 [1, N, 6] (x1,y1,x2,y2,conf,cls)")
        print("[WARN] C++ 后处理代码不兼容该格式，需修改 inference.cpp")
    else:
        n_cls = model.model.model[-1].nc if hasattr(model.model, 'model') else model.model.model[-1].nc
        grids = (args.imgsz // 8) ** 2 + (args.imgsz // 16) ** 2 + (args.imgsz // 32) ** 2
        print(f"[INFO] 预期输出: [1, {5 + n_cls}, {grids}]  (4 bbox + 1 obj + {n_cls} class logits)")

    output_path = model.export(**export_kwargs)
    print(f"[DONE] ONNX model saved to: {output_path}")


if __name__ == "__main__":
    main()
