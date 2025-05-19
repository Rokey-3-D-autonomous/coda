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


class PatrolNode:

    INIT_POSE = [0.25, 0.71, 270.0]  # docking station
    GOAL_POSES = [
        # start to patrol
        [0.0350, 0.6523, 180.0],  # point 1
        [-0.0619, -1.6371, 180.0],  # point 2
        [0.4564, -0.8648, 270.0],  # point 3
        [0.8353, -1.6557, 0.0],  # point 4
        [1.0313, 0.6398, 0.0],  # front of docking station
    ]
    DISPATCH_POSE = [0.0, 0.0, 0.0]  # central of map

    INIT_LOADING_TIME = 5.0

    SUCCEEDED = "SUCCEEDED"
    DISPATCHED = "DISPATCHED"
    RECOVERED = "RECOVERED"
    DONE = "DONE"

    class AccidentDetected(Exception):
        """사고 감지로 인해 순찰을 중단하고 출동해야 하는 예외"""

        pass

    class PatrolFailure(Exception):
        """순찰지 이동 실패 시 복구가 필요한 예외"""

        pass

    def __init__(self):
        self.patrol_logger = get_logger("PatrolNode")
        self.patrol_logger.info("start initialization")

        # 두 navigator 인스턴스 생성
        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator(node_name="navigator_robot1")
        self.patrol_logger.info("create navigators")

        self._get_init_pose()
        self._undock()
        self._get_goal_poses()
        self.patrol_logger.info("configure PatrolNode")

    def _create_pose(self, x, y, yaw_deg) -> PoseStamped:
        """x, y, yaw(degree) → PoseStamped 생성"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.nav_navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y

        yaw_rad = yaw_deg * 3.141592 / 180.0
        q = quaternion_from_euler(0, 0, yaw_rad)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def _get_init_pose(self) -> None:
        initial_pose = self._create_pose(*self.INIT_POSE)
        self.nav_navigator.setInitialPose(initial_pose)
        self.nav_navigator.get_logger().info(
            f"초기 위치 설정 중... {int(self.INIT_LOADING_TIME)}s"
        )
        time.sleep(
            self.INIT_LOADING_TIME
        )  # AMCL이 초기 pose 처리 시 필요한 시간과 TF를 얻을 수 있게 됨

        self.nav_navigator.waitUntilNav2Active()
        self.nav_navigator.get_logger().info(f"네비게이션 시스템 작동 중...")

    def _undock(self) -> None:
        if self.dock_navigator.getDockedStatus():
            self.dock_navigator.get_logger().info("현재 도킹 상태 → 언도킹 시도")
            self.dock_navigator.undock()
        else:
            self.dock_navigator.get_logger().info("언도킹 상태에서 시작")

    def _get_goal_poses(self) -> None:
        self.goal_poses = [
            self._create_pose(*self.GOAL_POSES[i]) for i in range(len(self.GOAL_POSES))
        ]
        self.patrol_logger.info("순찰지 생성")

    def move_generator(self) -> iter:
        i = 0
        while i < len(self.goal_poses):
            goal = self.goal_poses[i]
            try:
                self._move_once(i, goal)
                i += 1
                yield i, self.SUCCEEDED
            except self.AccidentDetected as e:
                self.nav_navigator.get_logger().warn(str(e))
                yield i, self.DISPATCHED
            except self.PatrolFailure as e:
                self.nav_navigator.get_logger().error(str(e))
                yield i, self.RECOVERED
        yield i, self.DONE

    def _get_feedback(self, log_msg: str) -> None:
        while not self.nav_navigator.isTaskComplete():
            feedback = self.nav_navigator.getFeedback()
            if feedback:
                self.nav_navigator.get_logger().info(
                    f"{log_msg}, 남은 거리: {feedback.distance_remaining:.2f} m"
                )

    def _move_once(self, i: int, goal: PoseStamped) -> None:
        self.nav_navigator.get_logger().info(f"{i+1} 순찰지 순찰 시작")

        # 이동 명령
        self.nav_navigator.goToPose(goal)

        # 이동 중 사고 감지 시, 출동.
        self._get_feedback(f"{i+1}번째 순찰지 이동 중")

        # 목표 지점까지 이동 완료
        result = self.nav_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            if i < 4:
                self.nav_navigator.get_logger().info(f"{i+1}번째 순찰지 도달.")
            else:
                self.nav_navigator.get_logger().info(f"순찰 완료. 복귀 중...")
        elif result == TaskResult.CANCELED:
            raise self.AccidentDetected(f"{i+1}번째 순찰지 이동 취소됨")
        else:
            raise self.PatrolFailure(f"{i+1}번째 순찰지 이동 실패. 상태: {result}")

    def cancel(self) -> None:
        if not self.nav_navigator.isTaskComplete():
            self.nav_navigator.cancelTask()
            self.nav_navigator.get_logger().warn("현재 이동 목표 취소 요청됨")

    def dispatch(
        self,
    ) -> None:
        self.nav_navigator.get_logger().info(f"사고 발생: 차량 통제를 위해 출동")
        dispatch_pose = self._create_pose(*self.DISPATCH_POSE)

        # 출동 명령
        self.nav_navigator.goToPose(dispatch_pose)
        self.nav_navigator.get_logger().info("출동 시작")

        # 출동 중
        self._get_feedback("출동지 이동 중")

        # 목표 지점까지 출동 완료
        result = self.nav_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.nav_navigator.get_logger().info("출동지 도달. 임무 수행")
        else:
            raise self.PatrolFailure(f"출동지 이동 실패. 상태: {result}")

    def recovery(self, i: int) -> None:
        self.nav_navigator.get_logger().error(f"오류 발생: {i+1}번 위치로 재이동 시도")
        docking_station_pose = self._create_pose(*self.GOAL_POSES[i])

        # 복구 명령
        self.nav_navigator.goToPose(docking_station_pose)
        self.nav_navigator.get_logger().info("복구 시작")

        # 복구 중
        self._get_feedback("복구 중")

        # 복구 완료
        result = self.nav_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.nav_navigator.get_logger().info("복구 위치 도달 성공")
        else:
            self.nav_navigator.get_logger().error(
                f"복구 위치 도달 실패. 코드: {result}"
            )
            raise self.PatrolFailure(f"복구 실패. 상태: {result}")

    def dock(self) -> None:
        self.dock_navigator.get_logger().info("복귀 완료. 도킹 시도...")
        self.dock_navigator.dock()

    def terminate(self) -> None:
        self.dock_navigator.get_logger().info("도킹 완료. 종료...")
        self.dock_navigator.destroy_node()
        self.nav_navigator.get_logger().info("순찰 완료. 종료...")
        self.nav_navigator.destroy_node()

    pass


def main(args=None):
    rclpy.init(args=args)

    # 멀티스레드 실행을 위한 Executor (TF listener 등 동시 처리 위해)
    executor = MultiThreadedExecutor()

    try:
        node = PatrolNode()
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
