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


class DispatchNode:

    def __init__(self):
        super().__init__("Dispatch Node")

        self.app = QApplication(sys.argv)
        self.window = DisplayWindow()

        self.audio_publisher = AudioPublisher()

    def display_ui(self):
        pass

    def turn_off_ui(self):
        pass

    def turn_on_beep(self):
        pass

    def turn_off_beep(self):
        pass

    def terminate(self):
        pass

    pass
