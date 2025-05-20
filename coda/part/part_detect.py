import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rclpy.time import Time as RclTime
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger

from std_srvs.srv import Empty as srv_empty
from std_msgs.msg import Empty as msg_empty
from std_msgs.msg import Int32 as i32
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from coda_interfaces.srv import VehicleControl

# nav
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs

# object detection
from ultralytics import YOLO
import torch
from cv_bridge import CvBridge
import cv2
import numpy as np

# dispatch
from PyQt5.QtWidgets import QApplication
from display import VehicleControlDisplay, DisplayWindow
from audio import AudioPublisher

import threading, os, sys, time

TB0_NAMESPACE = "/robot0"  # photo
TB1_NAMESPACE = "/robot1"  # patrol


class DetectionNode(Node):

    HOME_PATH = os.path.expanduser("~")  # 홈 디렉토리 경로
    MODEL_PATH = os.path.join(HOME_PATH, "rokey_ws", "model", "best.pt")  # 모델 경로

    RGB_TOPIC = TB0_NAMESPACE + "/oakd/rgb/preview/image_raw"

    TARGET_CLASS_ID = [0, 3]
    INFERENCE_PERIOD_SEC = 1.0 / 30

    def __init__(self, detection_callback):
        super().__init__("Detection Node")
        self.get_logger().info("[1/3] Detection Node 노드 초기화 시작...")

        self.detection_callback = detection_callback  # 사고 감지 콜백 함수
        self.bridge = CvBridge()
        self.latest_rgb = self.latest_rgb_msg = None
        self.overlay_info = []
        self.display_rgb = None
        self.lock = threading.Lock()

        if not os.path.exists(self.MODEL_PATH):
            self.get_logger().error(f"Model file not found: {self.MODEL_PATH}")
            sys.exit(1)

        self.model = YOLO(self.MODEL_PATH)
        self.model.to("cuda" if torch.cuda.is_available() else "cpu")
        self.class_names = getattr(self.model, "names", [])
        self.get_logger().info(
            f"[2/3] YOLO 모델 로드 완료 (GPU 사용: {torch.cuda.is_available()})"
        )

        self.create_subscription(Image, self.RGB_TOPIC, self._rgb_callback, 10)
        self.get_logger().info(f"[3/3] 토픽 구독 완료:\n  RGB: {self.RGB_TOPIC}")

        self.create_timer(self.INFERENCE_PERIOD_SEC, self._inference_callback)

    def _rgb_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.latest_rgb = img.copy()
                self.display_rgb = img.copy()
                self.latest_rgb_msg = msg
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")

    def _inference_callback(self):
        with self.lock:
            rgb, rgb_msg = self.latest_rgb.copy(), self.latest_rgb_msg

        if rgb is None or rgb_msg is None:
            return

        results = self.model(rgb, stream=True)
        overlay_info = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                cls = int(box.cls[0])
                if cls not in self.TARGET_CLASS_ID:
                    continue

                if float(box.conf[0]) < 0.6:
                    continue

                label = (
                    self.class_names[cls]
                    if cls < len(self.class_names)
                    else f"class_{cls}"
                )
                conf = float(box.conf[0])

                u, v = map(int, box.xywh[0][:2].cpu().numpy())
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

                overlay_info.append(
                    {
                        "label": label,
                        "conf": conf,
                        "center": (u, v),
                        "bbox": (x1, y1, x2, y2),
                        "header": rgb_msg.header,
                    }
                )
                self.get_logger().info(f"overlay_info: {overlay_info}")

        with self.lock:
            self.overlay_info = overlay_info

    def rgb_capture_callback(self):
        with self.lock:
            rgb = self.latest_rgb.copy()
            overlay_info = self.overlay_info.copy()

    pass
