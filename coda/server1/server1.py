import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from coda_interfaces.srv import VehicleControl
from rclpy.logging import get_logger

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

import threading, os, sys, time

class PatrolNode:

    INIT_POSE = [0.2509, 0.7195, 270.0]
    GOAL_POSES = [
        # start to patrol
        [0.0350, 0.6523, 180.0],    # point 1
        [-0.0619, -1.6371, 180.0],  # point 2
        [0.4564, -0.8648, 270.0],   # point 3
        [0.8353, -1.6557, 0.0],     # point 4
        [1.0313, 0.6398, 0.0],      # docking station
    ]
    DISPATCH_POSE = [0.0, 0.0, 0.0]
    INIT_LOADING_TIME = 5.0

    def __init__(self):
        self.patrol_logger = get_logger('PatrolNode')
        self.patrol_logger.info('[PatrolNode] start initialization')

        # 두 navigator 인스턴스 생성
        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator(node_name='navigator_robot1')
        self.patrol_logger.info('[PatrolNode] create navigators')

    def create_pose(self, x, y, yaw_deg) -> PoseStamped:
        """x, y, yaw(degree) → PoseStamped 생성"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
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

    def get_init_pose(self) -> None:
        initial_pose = self.create_pose(*self.INIT_POSE)
        self.nav_navigator.setInitialPose(initial_pose)
        self.nav_navigator.get_logger().info(f'초기 위치 설정 중... {int(self.INIT_LOADING_TIME)}s')
        time.sleep(self.INIT_LOADING_TIME) #AMCL이 초기 pose 처리 시 필요한 시간과 TF를 얻을 수 있게 됨
        
        self.nav_navigator.waitUntilNav2Active()
        self.nav_navigator.get_logger().info(f'네비게이션 시스템 작동 중...')

    def undock(self) -> None:
        if self.dock_navigator.getDockedStatus():
            self.dock_navigator.get_logger().info('현재 도킹 상태 → 언도킹 시도')
            self.dock_navigator.undock()
        else:
            self.dock_navigator.get_logger().info('언도킹 상태에서 시작')

    def get_goal_poses(self) -> None:
        self.goal_poses = [self.create_pose(*self.GOAL_POSES[i]) for i in range(len(self.GOAL_POSES))]
        self.patrol_logger.info('[PatrolNode] get goal poses')
    
    def move_generator(self) -> iter:
        for i, goal in enumerate(self.goal_poses):
            try:
                self.move_once(i, goal)
                yield 'SUCCEEDED'
            except RuntimeError as e:
                self.recovery(i, e)
                yield 'RECOVERED'

    def get_feedback(self, log_msg: str) -> None:
        while not self.nav_navigator.isTaskComplete():
            feedback = self.nav_navigator.getFeedback()
            if feedback:
                self.nav_navigator.get_logger().info(f'{log_msg}, 남은 거리: {feedback.distance_remaining:.2f} m')
    
    def move_once(self, i: int, goal: PoseStamped) -> None:
        # 이동 명령
        self.nav_navigator.goToPose(goal)
        self.nav_navigator.get_logger().info(f'{i+1} 순찰지 순찰 시작')

        # 이동 중 사고 감지 시, 출동.
        self.get_feedback(f'{i+1}번째 순찰지 이동 중')

        # 목표 지점까지 이동 완료
        result = self.nav_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            if i < 4:
                self.nav_navigator.get_logger().info(f'{i+1}번째 순찰지 도달.')
            else:
                self.nav_navigator.get_logger().info(f'순찰 완료. 복귀 중...')
        else:
            raise RuntimeError(f'{i+1}번째 순찰지 이동 실패. 상태: {result}')

    def cancel(self) -> None:
        if not self.nav_navigator.isTaskComplete():
            self.nav_navigator.cancelTask()
            self.nav_navigator.get_logger().warn('현재 이동 목표 취소 요청됨')
        
    def dispatch(self) -> None:
        self.nav_navigator.get_logger().info(f'사고 발생: 차량 통제를 위해 출동')
        dispatch_pose = self.create_pose(*self.DISPATCH_POSE)

        # 출동 명령
        self.nav_navigator.goToPose(dispatch_pose)
        self.nav_navigator.get_logger().info('출동 시작')

        # 출동 중
        self.get_feedback('출동지 이동 중')

        # 목표 지점까지 출동 완료
        result = self.nav_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.nav_navigator.get_logger().info('출동지 도달. 임무 수행')
        else:
            raise RuntimeError(f'출동지 이동 실패. 상태: {result}')

    def recovery(self, i: int, e: RuntimeError) -> None:
        self.nav_navigator.get_logger().error(f'오류 발생: {e} → {i+1}번 위치로 재이동 시도')
        docking_station_pose = self.create_pose(*self.GOAL_POSES[i])
        
        # 복구 명령
        self.nav_navigator.goToPose(docking_station_pose)
        self.nav_navigator.get_logger().info('복구 시작')

        # 복구 중
        self.get_feedback('복구 중')

        # 복구 완료
        result = self.nav_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.nav_navigator.get_logger().info('복구 위치 도달 성공')
        else:
            self.nav_navigator.get_logger().error(f'복구 위치 도달 실패. 코드: {result}')

    def dock(self) -> None:
        self.dock_navigator.get_logger().info('복귀 완료. 도킹 시도...')
        self.dock_navigator.dock()

    def terminate(self) -> None:
        self.dock_navigator.get_logger().info('도킹 완료. 종료...')
        self.dock_navigator.destroy_node()
        self.nav_navigator.get_logger().info('순찰 완료. 종료...')
        self.nav_navigator.destroy_node()


    pass

class DetectionNode(Node):

    HOME_PATH = os.path.expanduser("~")                                     # 홈 디렉토리 경로
    MODEL_PATH = os.path.join(HOME_PATH, 'rokey_ws', 'model', 'best.pt')    # 모델 경로

    def __init__(self):
        super().__init__('[Detection Node]')
        self.get_logger().info("[1/5] Detection Node 노드 초기화 시작...")

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
        self.classNames = getattr(self.model, "names", [])
        self.get_logger().info(f"[2/5] YOLO 모델 로드 완료 (GPU 사용: {torch.cuda.is_available()})")
    pass

class TransformNode(Node):
    pass

class DispatchNode(Node):
    pass

class Server1:

    class ServerState:
        IDLE        = 'IDLE'
        PATROLING   = 'PATROLING'
        DISPATCHING = 'DISPATCHING'
        TERMINATED  = 'TERMINATED'

    def __init__(self):
        self.server_state = self.ServerState.IDLE
        # self.lock = threading.Lock()

        self.patrol_node    = PatrolNode()
        self.detect_node    = DetectionNode()
        self.transform_node = TransformNode()
        self.dispatch_node  = DispatchNode()

    def patrol(self):
        # with self.lock:
        if self.server_state in [self.ServerState.IDLE, self.ServerState.DISPATCHING]:
            self.server_state = self.ServerState.PATROLING
        else:
            print(f'[{self.patrol.__name__}] : {self.server_state} is wrong!!')

        # send patrol action from client

        pass

    def patrol_calllback(self):

        # detecting accident from camera image

        pass

    def dispatch(self):
        # with self.lock:
        if self.server_state in [self.ServerState.PATROLING]:
            self.server_state = self.ServerState.DISPATCHING
        else:
            print(f'[{self.dispatch.__name__}] : {self.server_state} is wrong!!')

        # send dispatch service from client

        pass

    def restart_patrol(self):
        # with self.lock:
        if self.server_state in [self.ServerState.DISPATCHING]:
            self.server_state = self.ServerState.PATROLING
        else:
            print(f'[{self.restart_patrol.__name__}] : {self.server_state} is wrong!!')

        # restart patrol with ignoring already detected accdient

        pass

    def exit_server(self):
        # with self.lock:
        self.server_state = self.ServerState.TERMINATED

        # exit server

        pass

def main():
    pass