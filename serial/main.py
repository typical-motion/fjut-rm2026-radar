import cv2
import time
import numpy as np
from utils import *
from yolov5Detector import YOLOv5Detector
from hik_camera import HikCamera
from config import Config
from location import Location
from ui_design import MapVisualizer
import threading
from my_serial import SerialManager
from video_recorder import VideoRecorder

class VideoProcessor:
    def __init__(self):
        self.car_detector = YOLOv5Detector(Config.CAR_MODEL_PATH, Config.CAR_YAML_PATH, conf_threshold=0.1, iou_threshold=0.5, max_det=14)
        self.armor_detector = YOLOv5Detector(Config.ARMOR_MODEL_PATH, Config.ARMOR_YAML_PATH, conf_threshold=0.5, iou_threshold=0.2, max_det=1)
        self.frame_counter = 0
        self.fps = 0.0
        self.prev_time = time.time()
        self.locator = Location()
        self.map_visualizer = MapVisualizer("image//map.jpg")
        self.current_enemy_positions = {}
        self.friendly_positions = {}
        
        self.serial_manager = None
        self.thread_receive = None
        self.thread_send = None
        
        self.recorder = VideoRecorder()
        
    def initialize_serial(self, port, color='R'):
        try:
            self.serial_manager = SerialManager(port, color=color)
            # 将录制器实例传递给串口管理器
            self.serial_manager.recorder = self.recorder
            # 启动接收线程
            self.thread_receive = threading.Thread(target=self.serial_manager.receive_serial, daemon=True)
            self.thread_receive.start()
            # 启动发送线程
            print(f"串口通信初始化完成: 端口 {port}, 队伍 {color}")
            self.recorder.log_message(f"串口通信初始化完成: 端口 {port}, 队伍 {color}")
            self.thread_send = threading.Thread(target=self.serial_manager.send_serial, daemon=True)
            self.thread_send.start()
            return True
        except Exception as e:
            print(f"串口通信初始化失败: {e}")
            self.recorder.log_message(f"串口通信初始化失败: {e}", "error")
            return False
            
    def send_positions(self):
        if self.serial_manager:
            # 合并敌我双方位置
            all_positions = {}
            all_positions.update(self.current_enemy_positions)
            all_positions.update(self.friendly_positions)
            
            # 如果有位置数据则更新
            if all_positions:
                try:
                    self.serial_manager.send_serial(all_positions)
                    # 记录位置数据到日志
                    if hasattr(self.serial_manager, 'seq'):
                        # 不需要额外在这里转换，因为log_positions方法已经包含转换逻辑
                        self.recorder.log_positions(all_positions, self.serial_manager.seq)
                    # 记录裁判系统数据
                    global double_vulnerability_chance
                    global opponent_double_vulnerability
                    if 'double_vulnerability_chance' in globals() and double_vulnerability_chance != -1:
                        self.recorder.log_referee_data(
                            double_vulnerability_chance,
                            opponent_double_vulnerability,
                            self.serial_manager.color
                        )
                except Exception as e:
                    print(f"发送位置数据失败: {e}")
                    self.recorder.log_message(f"发送位置数据失败: {e}", "error")
        
    def calculate_fps(self):
        self.frame_counter += 1
        if self.frame_counter % 10 == 0:
            current_time = time.time()
            self.fps = 10 / (current_time - self.prev_time)
            self.prev_time = current_time
        return self.fps
    
    def process_video(self, video_path):
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"无法打开视频文件: {video_path}")
            self.recorder.log_message(f"无法打开视频文件: {video_path}", "error")
            return
    
        
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))  
        map_size = self.map_visualizer.get_map_size()
        self.recorder.start_recording((width, height), map_size)
        self.recorder.log_message(f"视频文件: {video_path}, 大小: {width}x{height}")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            raw_frame = frame.copy()
            
            final_detections = process_frame(frame, self.car_detector, self.armor_detector, Config.CAMERA_MATRIX, Config.MAP_SIZE, self.locator)
            frame = draw_preditions(frame, final_detections, self.calculate_fps())
            
            # 可视化当前检测到的机器人位置
            self._visualize_positions(final_detections)
            
            # 获取地图帧用于录制
            map_frame = self.map_visualizer.get_map_frame()
            self.recorder.record_frame(raw_frame, frame, map_frame)
            
            frame1 = cv2.resize(frame, (1500, 1000))
            cv2.imshow("Detection Result", frame1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.recorder.stop_recording()
        cap.release()
        
    def process_image_test(self, image_path):       # 用来测试定位的（用图片）
        frame = cv2.imread(image_path)
        if frame is None:  
            print(f"无法加载图片: {image_path}")
            self.recorder.log_message(f"无法加载图片: {image_path}", "error")
            return None
        
        final_detections = process_frame(frame, self.car_detector, self.armor_detector, Config.CAMERA_MATRIX, Config.MAP_SIZE, self.locator)
        frame = draw_preditions(frame, final_detections)
        
        frame = cv2.resize(frame, (1500, 1000))
        cv2.imshow("Image Test", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
                
    def _resize_frame(self, frame, target_size=(1500, 1000)):
        h, w = frame.shape[:2]
        scale = min(target_size[0]/w, target_size[1]/h)
        return cv2.resize(frame, (int(w*scale), int(h*scale)))
    
    def _visualize_positions(self, detections):
        # 更新当前位置信息
        for det in detections:
            _, _, _, _, _, cls_name, pos, _ = det  # 添加下划线来忽略最后一个装甲板ROI信息
            if pos is None:
                continue
                
            # 区分敌我双方
            if self.serial_manager and self.serial_manager.color == "R":
                if cls_name.startswith("B"):  # 敌方（蓝方）
                    self.current_enemy_positions[cls_name] = pos
                else:  # 友方（红方）
                    self.friendly_positions[cls_name] = pos
            else:  # 蓝方视角
                if cls_name.startswith("R"):  # 敌方（红方）
                    self.current_enemy_positions[cls_name] = pos
                else:  # 友方（蓝方）
                    self.friendly_positions[cls_name] = pos
        
        # 使用地图可视化器显示位置
        self.map_visualizer.clear()
        for robot_id, pos in self.current_enemy_positions.items():
            self.map_visualizer.add_enemy(robot_id, pos[0], 15-pos[1])
        for robot_id, pos in self.friendly_positions.items():
            self.map_visualizer.add_friendly(robot_id, pos[0], 15-pos[1])
            
        self.map_visualizer.update()
        
        # Send positions to referee system
        self.send_positions()
    
class Runner:
    def __init__(self, mode=Config.SELECT_MODE, debug=True):
        self.mode = mode
        self.debug = debug  # 默认启用调试模式
        self.port = Config.PORT
        self.color = Config.COLOR
        self.locator = Location()
        
        # 初始化视频处理器
        self.processor = VideoProcessor()
        
        # 初始化串口通信
        if self.port:
            print(f"正在初始化串口通信: {self.port}, 队伍颜色: {'红方' if self.color == 'R' else '蓝方'}")
            self.processor.initialize_serial(self.port, self.color)
        
    def run(self):
        try:
            if self.mode == 'test':
                self._run_video()
                # self._run_image_test()  # 用图片测试定位效果
            elif self.mode == 'hik':
                self._run_camera()
        finally:
            cv2.destroyAllWindows()
            
    def _run_video(self):
        self.processor.process_video(Config.VIDEO_PATH)
        # 处理完视频后暂停一下，让用户能看到最后的画面
        print("视频处理完成，按任意键关闭...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def _run_image_test(self):
        test_image_path = "image/test_image.jpg"
        self.processor.process_image_test(test_image_path)
        
    def _run_camera(self):
        from hik_camera import HikCamera
        
        camera = None
        try:
            # 初始化相机
            self.processor.recorder.log_message("正在初始化相机...", "info")
            camera = HikCamera(Config.HIK_CONFIG)
            
            # 等待相机初始化完成
            init_timeout = 5  # 5秒超时
            init_start_time = time.time()
            while time.time() - init_start_time < init_timeout:
                if camera.camera_active:
                    break
                time.sleep(0.1)
            
            if not camera.camera_active:
                error_msg = "相机初始化超时，请检查连接和配置"
                self.processor.recorder.log_message(error_msg, "error")
                print(error_msg)
                return
                
            # 获取第一帧来设置视频尺寸
            frame_timeout = 3  # 3秒超时
            frame_start_time = time.time()
            initial_frame = None
            
            while time.time() - frame_start_time < frame_timeout:
                initial_frame = camera.get_latest_frame()
                if initial_frame is not None:
                    break
                time.sleep(0.1)
                
            if initial_frame is None:
                error_msg = "无法获取相机画面，请检查相机配置"
                self.processor.recorder.log_message(error_msg, "error")
                print(error_msg)
                return
                
            h, w = initial_frame.shape[:2]
            map_size = self.processor.map_visualizer.get_map_size()
            self.processor.recorder.start_recording((w, h), map_size)
            self.processor.recorder.log_message(f"相机初始化成功，分辨率: {w}x{h}")
            
            # 设置帧获取超时和错误计数
            frame_error_count = 0
            max_frame_errors = 10  # 连续10次获取帧失败后重启相机
            
            while True:
                try:
                    frame = camera.get_latest_frame()
                    if frame is None:
                        frame_error_count += 1
                        if frame_error_count >= max_frame_errors:
                            self.processor.recorder.log_message("连续多次获取帧失败，尝试重启相机", "warning")
                            camera.reset()
                            frame_error_count = 0
                            time.sleep(1)  # 等待相机重置
                        continue
                    
                    # 成功获取帧，重置错误计数
                    frame_error_count = 0
                    
                    raw_frame = frame.copy()
                    final_detections = process_frame(frame, self.processor.car_detector, 
                                                   self.processor.armor_detector, 
                                                   Config.CAMERA_MATRIX, Config.MAP_SIZE, 
                                                   self.locator)
                                                   
                    frame = draw_preditions(frame, final_detections)
                    
                    # 可视化位置
                    self.processor._visualize_positions(final_detections)
                    
                    # 获取地图帧用于录制
                    map_frame = self.processor.map_visualizer.get_map_frame()
                    self.processor.recorder.record_frame(raw_frame, frame, map_frame)
    
                    cv2.imshow("Detection Result", self.processor._resize_frame(frame))
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                        
                except Exception as e:
                    error_msg = f"处理帧时出错: {str(e)}"
                    self.processor.recorder.log_message(error_msg, "error")
                    print(error_msg)
                    time.sleep(0.5)  # 出错后稍微暂停一下
                
        except Exception as e:
            error_msg = f"相机运行时出错: {str(e)}"
            if hasattr(self.processor, 'recorder'):
                self.processor.recorder.log_message(error_msg, "error")
            print(error_msg)
        finally:
            self._safe_shutdown(camera)
            # 处理完相机后暂停一下，让用户能看到最后的画面
            print("相机处理结束，按任意键关闭...")
            cv2.waitKey(0)

    def _safe_shutdown(self, camera):
        # 停止录制
        if hasattr(self.processor, 'recorder'):
            try:
                self.processor.recorder.log_message("正在停止录制...", "info")
                self.processor.recorder.stop_recording()
                self.processor.recorder.log_message("录制已停止", "info")
            except Exception as e:
                print(f"停止录制出错: {e}")
                
        # 关闭串口通信
        if hasattr(self, 'processor') and self.processor.serial_manager:
            try:
                self.processor.recorder.log_message("正在关闭串口通信...", "info")
                self.processor.serial_manager.stop()  # 停止串口线程
                self.processor.recorder.log_message("串口通信已关闭", "info")
            except Exception as e:
                error_msg = f"停止串口线程出错: {e}"
                print(error_msg)
                if hasattr(self.processor, 'recorder'):
                    self.processor.recorder.log_message(error_msg, "error")
        
        # 关闭相机
        if camera:
            try:
                self.processor.recorder.log_message("正在关闭相机...", "info")
                camera.stop()
                self.processor.recorder.log_message("相机已关闭", "info")
                
                # 等待线程完全终止
                thread_wait_timeout = 5  # 5秒超时
                thread_wait_start = time.time()
                while time.time() - thread_wait_start < thread_wait_timeout:
                    if not any([camera.capture_thread.is_alive(), camera.monitor_thread.is_alive()]):
                        break
                    time.sleep(0.1)
                
                if any([camera.capture_thread.is_alive(), camera.monitor_thread.is_alive()]):
                    self.processor.recorder.log_message("相机线程未能完全终止", "warning")
            except Exception as e:
                error_msg = f"关闭相机出错: {e}"
                print(error_msg)
                if hasattr(self.processor, 'recorder'):
                    self.processor.recorder.log_message(error_msg, "error")

if __name__ == "__main__":
    import argparse
    
    # 命令行参数解析
    parser = argparse.ArgumentParser(description="雷达系统")
    parser.add_argument('--mode', type=str, default=Config.SELECT_MODE, choices=['test', 'hik'], 
                        help='运行模式: test使用视频测试, hik使用海康相机')
    parser.add_argument('--debug', action='store_true', help='启用调试模式')
    args = parser.parse_args()
    
    print(f"开始运行雷达系统")
    print(f"模式: {args.mode}, 调试: {'开启' if args.debug else '关闭'}")
    
    runner = Runner(mode=args.mode, debug=args.debug)
    runner.run()
