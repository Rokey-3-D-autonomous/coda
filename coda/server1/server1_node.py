import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime
from rclpy.executors import MultiThreadedExecutor
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

class UtilForServer1():
    def create_pose(self, pose) -> PoseStamped:
        x, y, yaw_deg = pose
        """x, y, yaw(도 단위) → PoseStamped 생성"""
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

# ========================================= Patrol Node =========================================
# pose
INIT_POSE = [0.2509, 0.7195, 270.0]
GOAL_POSES_BOT = [
    # start to patrol
    [0.0350, 0.6523, 180.0],    # map_bot1
    [-0.0619, -1.6371, 180.0],  # map_bot2
    [0.4564, -0.8648, 270.0],   # map_bot3
    [0.8353, -1.6557, 0.0],     # map_bot4
    [1.0313, 0.6398, 0.0],      # map_bot5

    # return to docking station
    [0.4564, -0.8648, 180.0],   # map_bot3
    [0.0350, 0.6523, 0.0],      # map_bot1 
]

# constant
INIT_LOADING_TIME = 5.0
WAITING_FOR_DETECTION = 2.0

class PatrolNode():
    def __init__(self):
        self.util = UtilForServer1()

        # 두 navigator instance 생성
        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator(node_name='navigator_robot1')

    def get_init_pose(self) -> None:
        initial_pose = self.util.create_pose(INIT_POSE)
        self.nav_navigator.setInitialPose(initial_pose)
        self.nav_navigator.get_logger().info(f'초기 위치 설정 중... {int(INIT_LOADING_TIME)}s')
        time.sleep(INIT_LOADING_TIME) #AMCL이 초기 pose 처리 시 필요한 시간과 TF를 얻을 수 있게 됨
        self.nav_navigator.waitUntilNav2Active()

    def undock(self) -> None:
        if self.dock_navigator.getDockedStatus():
            self.dock_navigator.get_logger().info('현재 도킹 상태 → 언도킹 시도')
            self.dock_navigator.undock()
        else:
            self.dock_navigator.get_logger().info('언도킹 상태에서 시작')

    def get_goal_poses(self) -> list:
        return [self.util.create_pose(GOAL_POSES_BOT[i]) for i in range(len(GOAL_POSES_BOT))]

    def get_feedback(self, i) -> None:
        while not self.nav_navigator.isTaskComplete():
            feedback = self.nav_navigator.getFeedback()
            if feedback:
                if i > 0:
                    self.nav_navigator.get_logger().info(f'{i+1}번째 경유지 이동 중, 남은 거리: {feedback.distance_remaining:.2f} m')
                else:
                    self.nav_navigator.get_logger().info(f'[복귀] 남은 거리: {feedback.distance_remaining:.2f} m')

    def move(self, goal_poses) -> None:
        for i, goal in enumerate(goal_poses):
            self.nav_navigator.goToPose(goal)

            self.get_feedback(i)

            result = self.nav_navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                if i < 5:
                    self.nav_navigator.get_logger().info(f'{i+1}번째 경유지 도달. {WAITING_FOR_DETECTION}초 정지...')
                    time.sleep(WAITING_FOR_DETECTION)
                else:
                    self.nav_navigator.get_logger().info(f'{i+1}번째 경유지 도달. 복귀 중')
            else:
                raise RuntimeError(f'{i+1}번째 경유지 이동 실패. 상태: {result}')
            
    def move_once(self, i, goal) -> None:
        self.nav_navigator.goToPose(goal)

        self.get_feedback(i)

        result = self.nav_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            if i < 5:
                self.nav_navigator.get_logger().info(f'{i+1}번째 경유지 도달. {WAITING_FOR_DETECTION}초 정지...')
                time.sleep(WAITING_FOR_DETECTION)
            else:
                self.nav_navigator.get_logger().info(f'{i+1}번째 경유지 도달. 복귀 중')
        else:
            raise RuntimeError(f'{i+1}번째 경유지 이동 실패. 상태: {result}')

    def recovery(self, e: RuntimeError) -> None:
        self.nav_navigator.get_logger().error(f'오류 발생: {e} → 1번 위치로 재이동 시도')

        # 1번 위치 재이동 시도
        recovery_pose = self.util.create_pose(GOAL_POSES_BOT[0])
        self.nav_navigator.goToPose(recovery_pose)

        self.get_feedback(0)

        result = self.nav_navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.nav_navigator.get_logger().info('복구 위치 도달 성공')
        else:
            self.nav_navigator.get_logger().error(f'복구 위치 도달 실패. 코드: {result}')

    def dock(self) -> None:
        self.dock_navigator.get_logger().info('경로 완료. 도킹 시도...')
        self.dock_navigator.dock()

    def terminate(self) -> None:
        self.dock_navigator.destroy_node()
        self.nav_navigator.destroy_node()

# ====================================== Object Detection ======================================
# 상수 정의
HOME_PATH = os.path.expanduser("~")  # 현재 사용자의 홈 디렉토리 경로
MODEL_PATH = os.path.join(HOME_PATH, 'rokey_ws', 'model', 'best.pt')  # 모델 경로
TARGET_CLASS_ID = [0, 3]
INFERENCE_PERIOD_SEC = 1.0 / 20  # 30Hz 추론 주기
INIT_LOADING_TIME = 5.0

BASE_LINK = "base_link"
# XYZ = ["x", "y", "z"]
# MAKE_XYZ_DICT = lambda x: {k: v for k, v in zip(XYZ, x)}
ACCIDENT = 'accident'

# Topic
RGB_TOPIC = "/robot1/oakd/rgb/preview/image_raw"
DEPTH_TOPIC = "/robot1/oakd/stereo/image_raw"
CAMERA_INFO_TOPIC = "/robot1/oakd/stereo/camera_info"
MARKER_TOPIC = "/robot1/detected_objects_marker"

class ObjectDetectionNode(Node):
    def __init__(self, accident_callback=None):
        super().__init__('object_detection_node')
        self.get_logger().info("[1/5] 노드 초기화 시작...")

        self.util = UtilForServer1()
        self.bridge = CvBridge()
        self.K = None
        self.latest_rgb = self.latest_depth = self.latest_rgb_msg = None
        self.overlay_info = []
        self.display_rgb = None
        self.inference_timer = None
        self.accident_callback = accident_callback
        self.lock = threading.Lock()

        if not os.path.exists(MODEL_PATH):
            self.get_logger().error(f"Model file not found: {MODEL_PATH}")
            sys.exit(1)

        self.model = YOLO(MODEL_PATH)
        self.model.to("cuda" if torch.cuda.is_available() else "cpu")
        self.classNames = getattr(self.model, "names", [])
        self.get_logger().info(f"[2/5] YOLO 모델 로드 완료 (GPU 사용: {torch.cuda.is_available()})")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info(f"[3/5] TF2 Transform Listener 초기화 대기... {int(INIT_LOADING_TIME)}s")
        time.sleep(INIT_LOADING_TIME)

        self.create_subscription(Image, RGB_TOPIC, self._rgb_callback, 1)
        self.create_subscription(Image, DEPTH_TOPIC, self._depth_callback, 1)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._camera_info_callback, 1)
        self.get_logger().info(f"[4/5] 토픽 구독 완료:\n  RGB: {RGB_TOPIC}\n  Depth: {DEPTH_TOPIC}\n  CameraInfo: {CAMERA_INFO_TOPIC}")

        self.marker_pub = self.create_publisher(Marker, MARKER_TOPIC, 10)
        self.marker_id = 0
        self.get_logger().info(f"[5/5] 퍼블리셔 설정 완료\n  MAKER: {MARKER_TOPIC}")

    def _camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"CameraInfo 수신: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

    def _rgb_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.latest_rgb = img.copy()
                self.latest_rgb_msg = msg
                self.display_rgb = img.copy()
        except Exception as e:
            self.get_logger().error(f"RGB conversion error: {e}")

    def _depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            with self.lock:
                self.latest_depth = depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def _set_point(self, frame_id, x, y, z):
        point = PointStamped()
        point.header.frame_id = frame_id
        point.header.stamp = RclTime().to_msg()
        point.point.x, point.point.y, point.point.z = x, y, z
        return point

    def _transform_to_map(self, point: PointStamped, class_name: str):
        try:
            map = self.tf_buffer.transform(point, "map", timeout=rclpy.duration.Duration(seconds=0.5))
            x, y, z = map.point.x, map.point.y, map.point.z
            self.get_logger().info(f"[TF] {class_name} → map: (x={x:.2f}, y={y:.2f}, z={z:.2f})")
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"[TF] class={class_name} 변환 실패: {e}")
            return float("nan"), float("nan"), float("nan")

    def _publish_marker(self, x, y, z, label):
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

    def _inference_callback(self):
        with self.lock:
            rgb, depth, K, rgb_msg = (
                self.latest_rgb,
                self.latest_depth,
                self.K,
                self.latest_rgb_msg,
            )

        if any(v is None for v in (rgb, depth, K, rgb_msg)):
            return

        results = self.model(rgb)
        overlay_info = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                cls = int(box.cls[0])
                if cls not in TARGET_CLASS_ID:
                    continue

                u, v = map(int, box.xywh[0][:2].cpu().numpy())
                if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
                    continue

                z = float(depth[v, u]) / 1000.0
                if z <= 0.05 or np.isnan(z):
                    continue

                fx, fy = K[0, 0], K[1, 1]
                cx, cy = K[0, 2], K[1, 2]
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                conf = float(box.conf[0])
                label = self.classNames[cls] if cls < len(self.classNames) else f"class_{cls}"

                overlay_info.append({
                    "label": label,
                    "conf": conf,
                    "center": (u, v),
                    "bbox": (x1, y1, x2, y2),
                    "depth": z
                })
                self.get_logger().info(f'overlay_info: {overlay_info}')

                # object 기준 포인트 생성
                obj = self._set_point(rgb_msg.header.frame_id, x, y, z)
                obj_x, obj_y, obj_z = self._transform_to_map(obj, label)
                if not np.isnan(obj_x):
                    self._publish_marker(obj_x, obj_y, obj_z, label)

                # base_link 기준 포인트 생성
                # base = self._set_point(BASE_LINK, 0.0, 0.0, 0.0)
                # base_x, base_y, base_z = self._transform_to_map(base, BASE_LINK)
                
                # server1의 콜백함수 실행
                if label == ACCIDENT:
                    self.accident_callback()

        with self.lock:
            self.overlay_info = overlay_info

    def create_callback_timer(self):
        if self.inference_timer is None:
            self.get_logger().info("🟢 감지 타이머 시작")
            self.inference_timer = self.create_timer(INFERENCE_PERIOD_SEC, self._inference_callback)
        else:
            self.get_logger().warn("⚠️ 타이머가 이미 실행 중입니다.")

    def delete_callback_timer(self):
        if self.inference_timer:
            self.get_logger().info("🛑 감지 타이머 제거")
            self.destroy_timer(self.inference_timer)  # Node에서 완전히 제거
            self.inference_timer = None
        else:
            self.get_logger().warn("⚠️ 제거할 타이머가 없습니다.")

# ======================================= Vehicle Control =======================================
# Service
ACCIDENT_SERVICE = "/robot1/accident_detected"

class VehicleControlNode(Node):
    def __init__(self):
        super().__init__('vehicle_control_node')
        self._client = self.create_client(VehicleControl, ACCIDENT_SERVICE)

    def send_command(self, navigator: PatrolNode, target_pose: PoseStamped, response_callback=None):
        # 이동 명령
        self.get_logger().info(f'사고 발생: {target_pose} 위치로 turtlebot1 출동')
        navigator.move_once(target_pose)
        
        # 차량 통제 명령
        if not self._client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"Service {ACCIDENT_SERVICE} not available")
            if response_callback:
                response_callback(False)  # 실패로 간주
            return

        request = VehicleControl.Request()
        request.pose = target_pose

        future = self._client.call_async(request)

        # 응답을 처리한 뒤, 콜백으로 결과를 서버에 넘김
        if response_callback:
            future.add_done_callback(lambda fut: self._handle_response(fut, response_callback))
        else:
            future.add_done_callback(self._handle_response)

    def _handle_response(self, future, callback=None):
        try:
            response = future.result()
            accepted = response.accepted
            msg = "✅ 출동 명령 수락됨." if accepted else "❌ 출동 명령 거부됨."
            self.get_logger().info(msg)

            if callback:
                callback(accepted)

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            if callback:
                callback(False)

# ========================================= Server Node =========================================
# Constant
CONTROL_POSE = [0.01, 0.01, 0.0]

class Server1():
    def __init__(self):
        self.util = UtilForServer1()

        # 전역 상태 관리
        self.accident = False        # 사고 발생 여부
        self.detecting = False       # 감지 중 여부
        self.dispatching = False     # 출동 중 여부
        self.patrol_exit = False     # 순찰 종료 여부
        
        # rclpy.init()

        self.patrol_node = PatrolNode()
        self.object_detection_node = ObjectDetectionNode(self.on_accident_detected)
        self.vehicle_control_node = VehicleControlNode()

    # ---------- 순찰 제어 ----------
    def start_patrol(self):
        self.patrol_node.get_init_pose()
        self.patrol_node.undock()

    def do_patorl(self):
        # goal_poses = self.patrol_node.get_goal_poses()
        # try:
        #     self.patrol_node.move(goal_poses)
        # except RuntimeError as e:
        #     self.patrol_node.recovery(e)
        pass

    def exit_patrol(self):
        self.patrol_node.dock()
        self.patrol_node.terminate()
    
    # ---------- 탐지 제어 ----------
    def on_accident_detected(self):
        if not self.accident:
            self.accident = True
            self.send_control()         # 출동 명령

            # self.detecting = False
            # self.disable_detection()    # 감지 중단 등 필요하면

            
        # else:


    def detect_accident(self):
        pass

    def enable_detection(self):
        self.object_detection_node.create_callback_timer()

    def disable_detection(self):
        self.object_detection_node.delete_callback_timer()

    # ---------- 차량 통제 ----------
    def send_control(self):
        if self.dispatching:
            print("[Server1] 🚨 이미 출동된 상태입니다.")
            return

        control_pose = self.util.create_pose(CONTROL_POSE)

        def callback(accepted):
            if accepted:
                print("[Server1] 출동 명령 수락됨 → 상태 업데이트")
                self.dispatching = True
            else:
                print("[Server1] 출동 명령 거부됨 → 상태 유지")

        self.vehicle_control_node.send_command(control_pose, response_callback=callback)

    def reset_control_state(self):
        self.dispatching = False

def main():
    pass