import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rclpy.time import Time as RclTime
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger

from std_srvs.srv import Empty
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

import threading, os, sys, time

NAMESPACE = '/robot1'

class PatrolNode:

    INIT_POSE = [0.25, 0.71, 270.0]     # docking station
    GOAL_POSES = [
        # start to patrol
        [0.0350, 0.6523, 180.0],        # point 1
        [-0.0619, -1.6371, 180.0],      # point 2
        [0.4564, -0.8648, 270.0],       # point 3
        [0.8353, -1.6557, 0.0],         # point 4
        [1.0313, 0.6398, 0.0],          # front of docking station
    ]
    DISPATCH_POSE = [0.0, 0.0, 0.0]     # central of map

    INIT_LOADING_TIME = 5.0

    SUCCEEDED = 'SUCCEEDED'
    DISPATCHED = 'DISPATCHED'
    RECOVERED = 'RECOVERED'
    DONE = 'DONE'

    class AccidentDetected(Exception):
        """사고 감지로 인해 순찰을 중단하고 출동해야 하는 예외"""
        pass
    class PatrolFailure(Exception):
        """순찰지 이동 실패 시 복구가 필요한 예외"""
        pass

    def __init__(self):
        self.patrol_logger = get_logger('PatrolNode')
        self.patrol_logger.info('start initialization')

        # 두 navigator 인스턴스 생성
        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator(node_name='navigator_robot1')
        self.patrol_logger.info('create navigators')

        self._get_init_pose()
        self._undock()
        self._get_goal_poses()
        self.patrol_logger.info('configure PatrolNode')

    def _create_pose(self, x, y, yaw_deg) -> PoseStamped:
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

    def _get_init_pose(self) -> None:
        initial_pose = self._create_pose(*self.INIT_POSE)
        self.nav_navigator.setInitialPose(initial_pose)
        self.nav_navigator.get_logger().info(f'초기 위치 설정 중... {int(self.INIT_LOADING_TIME)}s')
        time.sleep(self.INIT_LOADING_TIME) #AMCL이 초기 pose 처리 시 필요한 시간과 TF를 얻을 수 있게 됨
        
        self.nav_navigator.waitUntilNav2Active()
        self.nav_navigator.get_logger().info(f'네비게이션 시스템 작동 중...')

    def _undock(self) -> None:
        if self.dock_navigator.getDockedStatus():
            self.dock_navigator.get_logger().info('현재 도킹 상태 → 언도킹 시도')
            self.dock_navigator.undock()
        else:
            self.dock_navigator.get_logger().info('언도킹 상태에서 시작')

    def _get_goal_poses(self) -> None:
        self.goal_poses = [self._create_pose(*self.GOAL_POSES[i]) for i in range(len(self.GOAL_POSES))]
        self.patrol_logger.info('순찰지 생성')
    
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
                self.nav_navigator.get_logger().info(f'{log_msg}, 남은 거리: {feedback.distance_remaining:.2f} m')
    
    def _move_once(self, i: int, goal: PoseStamped) -> None:
        self.nav_navigator.get_logger().info(f'{i+1} 순찰지 순찰 시작')
        
        # 이동 명령
        self.nav_navigator.goToPose(goal)

        # 이동 중 사고 감지 시, 출동.
        self._get_feedback(f'{i+1}번째 순찰지 이동 중')

        # 목표 지점까지 이동 완료
        result = self.nav_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            if i < 4:
                self.nav_navigator.get_logger().info(f'{i+1}번째 순찰지 도달.')
            else:
                self.nav_navigator.get_logger().info(f'순찰 완료. 복귀 중...')
        elif result == TaskResult.CANCELED:
            raise self.AccidentDetected(f'{i+1}번째 순찰지 이동 취소됨')
        else:
            raise self.PatrolFailure(f'{i+1}번째 순찰지 이동 실패. 상태: {result}')

    def cancel(self) -> None:
        if not self.nav_navigator.isTaskComplete():
            self.nav_navigator.cancelTask()
            self.nav_navigator.get_logger().warn('현재 이동 목표 취소 요청됨')
        
    def dispatch(self) -> None:
        self.nav_navigator.get_logger().info(f'사고 발생: 차량 통제를 위해 출동')
        dispatch_pose = self._create_pose(*self.DISPATCH_POSE)

        # 출동 명령
        self.nav_navigator.goToPose(dispatch_pose)
        self.nav_navigator.get_logger().info('출동 시작')

        # 출동 중
        self._get_feedback('출동지 이동 중')

        # 목표 지점까지 출동 완료
        result = self.nav_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.nav_navigator.get_logger().info('출동지 도달. 임무 수행')
        else:
            raise self.PatrolFailure(f'출동지 이동 실패. 상태: {result}')

    def recovery(self, i: int) -> None:
        self.nav_navigator.get_logger().error(f'오류 발생: {i+1}번 위치로 재이동 시도')
        docking_station_pose = self._create_pose(*self.GOAL_POSES[i])
        
        # 복구 명령
        self.nav_navigator.goToPose(docking_station_pose)
        self.nav_navigator.get_logger().info('복구 시작')

        # 복구 중
        self._get_feedback('복구 중')

        # 복구 완료
        result = self.nav_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.nav_navigator.get_logger().info('복구 위치 도달 성공')
        else:
            self.nav_navigator.get_logger().error(f'복구 위치 도달 실패. 코드: {result}')
            raise self.PatrolFailure(f'복구 실패. 상태: {result}')

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

    def __init__(self, detection_callback):
        super().__init__('Detection Node')
        self.get_logger().info("[1/5] Detection Node 노드 초기화 시작...")

        self.bridge = CvBridge()
        self.latest_rgb = self.latest_rgb_msg = None
        self.overlay_info = []
        self.display_rgb = None
        self.detection_callback = detection_callback
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

    VEHICLE_CONTROL = NAMESPACE + '/vehicle_control'

    def __init__(self):
        super().__init__('Dispatch Node')

        self._action_server = ActionClient(self, VehicleControl, self.VEHICLE_CONTROL)
    
    def send_dispatch(self, target_pose: PoseStamped):
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("TB2 action 서버가 준비되지 않았습니다.")
            return

        goal_msg = VehicleControl.Goal()
        goal_msg.target_pose = target_pose
        self.get_logger().info(f'🚨 사고 좌표 전송 → TB2: {target_pose.pose.position}')

        future = self._client.send_goal_async(goal_msg)

        def goal_response_callback(fut):
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Goal이 거부됨")
                return

            self.get_logger().info("Goal 수락됨. 결과 대기 중...")

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.result_callback)

        future.add_done_callback(goal_response_callback)

    def result_callback(self, future):
        result = future.result().result
        if result.resume_patrol:
            self.get_logger().info("✅ TB2 복귀 완료 → 순찰 재개")
            # 여기서 patrol() 다시 호출하거나 FSM에 신호 전달

    pass

class Server1:

    class ServerState:
        IDLE        = 'IDLE'
        PATROLING   = 'PATROLING'
        DISPATCHING = 'DISPATCHING'
        TERMINATED  = 'TERMINATED'

    def __init__(self):
        self.server_logger = get_logger('Server1')
        self.server_state = self.ServerState.IDLE
        # self.lock = threading.Lock()

        self.patrol_node    = PatrolNode()
        self.detect_node    = DetectionNode(self.patrol_node.cancel)    # detection 했을 떄, inference 잠시 중단
        self.transform_node = TransformNode()
        self.dispatch_node  = DispatchNode()

        # patrol
        self.patrol_iter = self.patrol_node.move_generator()

    def patrol(self):
        # with self.lock:
        if self.server_state in [self.ServerState.IDLE, self.ServerState.DISPATCHING]:
            self.server_state = self.ServerState.PATROLING
        else:
            self.server_logger.error(f'[{self.patrol.__name__}] : {self.server_state} is wrong!!')
            raise RuntimeError()

        for i, result in self.patrol_iter:
            self.server_logger.info(f'순찰 결과 : {result}')

            if result == 'SUCCEEDED':
                pass
            elif result == 'DISPATCHED':
                self.server_logger.info(f'{i+1}번째 순찰지로 향하던 중, 이동 목표 취소 요청 발생\n 상태 전이: {self.server_state} → {self.ServerState.DISPATCHING}')
                self.dispatch()     # 상태 전이
            elif result == 'RECOVERED':
                self.patrol_node.recovery(i)
            else:   # DONE
                self.server_logger.info(f'순찰지 {i+1}곳 순찰 완료\n 상태 전이: {self.server_state} → {self.ServerState.TERMINATED}')
                self.terminate()    # 상태 전이

        pass

    def dispatch(self):
        # with self.lock:
        if self.server_state in [self.ServerState.PATROLING]:
            self.server_state = self.ServerState.DISPATCHING
        else:
            self.server_logger.error(f'[{self.dispatch.__name__}] : {self.server_state} is wrong!!')
            raise RuntimeError()

        self.patrol_node.dispatch()

        self.patrol()   # 상태 전이

        pass

    def terminate(self):
        # with self.lock:
        self.server_state = self.ServerState.TERMINATED

        self.server_logger.info('순찰 종료')
        self.patrol_node.dock()
        self.patrol_node.terminate()

        self.server_logger.info('출동 종료')

        self.server_logger.info('탐지 종료')

        self.server_logger.info('변환 종료')

        pass

def main():
    rclpy.init()

    try:
        server1 = Server1()
    except RuntimeError as e:
        print(f'RuntimeError: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        server1.terminate()
        rclpy.shutdown()