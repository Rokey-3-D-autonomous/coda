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

# TB0_NAMESPACE = "/robot0"  # photo
# TB1_NAMESPACE = "/robot1"  # patrol

TB0_NAMESPACE = "/robot1"  # photo
TB1_NAMESPACE = "/robot0"  # patrol


class TransformNode(Node):

    INIT_LOADING_TIME = 5.0

    DEPTH_TOPIC = TB0_NAMESPACE + "/oakd/stereo/image_raw"
    CAMERA_INFO_TOPIC = TB0_NAMESPACE + "/oakd/stereo/camera_info"
    MARKER_TOPIC = TB0_NAMESPACE + "/detected_objects_marker"

    def __init__(self):
        super().__init__("TransformNode")
        self.get_logger().info("Transform Node 초기환")

        self.bridge = CvBridge()
        self.K = None
        self.latest_depth = None
        self.latest_info = None
        self.lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        time.sleep(self.INIT_LOADING_TIME)
        self.get_logger().info(
            f"[1/2] TF2 Transform Listener 초기화 대기... {int(self.INIT_LOADING_TIME)}s"
        )
        self.get_logger().info(f"00000\n")

        self.create_subscription(Image, self.DEPTH_TOPIC, self._depth_callback, 1)
        self.create_subscription(
            CameraInfo, self.CAMERA_INFO_TOPIC, self._camera_info_callback, 1
        )
        self.get_logger().info(
            f"[2/3] 토픽 구독 완료:\n  Depth: {self.DEPTH_TOPIC}\n  CameraInfo: {self.CAMERA_INFO_TOPIC}"
        )

        self.marker_pub = self.create_publisher(Marker, self.MARKER_TOPIC, 10)
        self.marker_id = 0
        self.get_logger().info(
            f"[3/3] 퍼블리셔 설정 완료\n  MAKER: {self.MARKER_TOPIC}"
        )

    def _depth_callback(self, msg):
        self.get_logger().info(f"11111\n")
        try:
            self.get_logger().info(f"tranform_node _depth_callback done \n")
            depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            with self.lock:
                self.latest_depth = depth
                self.get_logger().info(f"22222\n")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")
            self.get_logger().info(f"@@@@22222@@@@@\n")

    def _camera_info_callback(self, msg):
        self.get_logger().info(f"33333\n")
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(
                f"CameraInfo 수신: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}"
            )
            self.get_logger().info(f"44444\n")
        else:
            self.get_logger().info(f"@@@@44444\n")

    def _transform_to_map(self, point: PointStamped, label: str):
        try:
            map = self.tf_buffer.transform(
                point, "map", timeout=rclpy.duration.Duration(seconds=1.0)
            )
            x, y, z = map.point.x, map.point.y, map.point.z
            self.get_logger().info(
                f"[TF] {label} → map: (x={x:.2f}, y={y:.2f}, z={z:.2f})"
            )
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"[TF] class={label} 변환 실패: {e}")
            return float("nan"), float("nan"), float("nan")

    def _publish_marker(self, x, y, z, label):
        self.get_logger().info(f"55555\n")
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_objects"
        marker.id = self.marker_id
        marker.text = label
        self.marker_id += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 3
        self.marker_pub.publish(marker)

    def process_overlay(self, overlay_info, rgb_image):
        with self.lock:
            depth = self.latest_depth
            K = self.K

        if depth is None or K is None:
            return

        for obj in overlay_info:
            u, v = obj["center"]
            label = obj["label"]
            header = obj["header"]

            if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
                continue

            z = float(depth[v, u]) / 1000.0
            if z <= 0.05 or np.isnan(z):
                continue

            fx, fy = K[0, 0], K[1, 1]
            cx, cy = K[0, 2], K[1, 2]
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy

            point = PointStamped()
            point.header = header
            point.point.x, point.point.y, point.point.z = x, y, z
            obj_x, obj_y, obj_z = self._transform_to_map(point, label)

            if not np.isnan(obj_x):
                self._publish_marker(obj_x, obj_y, obj_z, label)

    pass


def main(args=None):
    rclpy.init(args=args)

    # 멀티스레드 실행을 위한 Executor (TF listener 등 동시 처리 위해)
    executor = MultiThreadedExecutor()

    try:
        node = TransformNode()
        executor.add_node(node)

        # 실행 시작
        node.get_logger().info("TransformNode 실행 시작")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("TransformNode 종료 요청 (Ctrl+C)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
