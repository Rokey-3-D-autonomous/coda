import os, sys, time, threading
from typing import List, Optional, Dict, Any, Tuple

import numpy as np
if not hasattr(np, 'float'): np.float = float

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclTime
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler
import tf2_ros
import tf2_geometry_msgs

from ultralytics import YOLO
import torch
from cv_bridge import CvBridge
import cv2

from coda.server2.pcd_to_html import RGBDToPCDConverter

# --- 상수 선언 ---
TB0_NAMESPACE = '/robot0'
TB1_NAMESPACE = '/robot1'

MIN_CONFIDENCE = 0.6
DISPLAY_CONFIDENCE = 0.65

# ------------------ NODE CLASSES ------------------

class PatrolNode(Node):
    """로봇의 순찰/복귀/복구/출동 등 이동을 담당하는 ROS2 Node"""

    INIT_POSE = [0.25, 0.71, 270.0]  # 도킹 스테이션 위치
    GOAL_POSES = [
        [0.0350, 0.6523, 180.0],
        [-0.0619, -1.6371, 180.0],
        [0.4564, -0.8648, 270.0],
        [0.8353, -1.6557, 0.0],
        [1.0313, 0.6398, 0.0],  # 도킹 전방
    ]
    DISPATCH_POSE = [0.0, 0.0, 0.0]
    DOCK_POSE = [0.5, 0.5, 0.0]

    INIT_LOADING_TIME = 5.0

    class AccidentDetected(Exception): pass
    class PatrolFailure(Exception): pass

    def __init__(self):
        super().__init__('PatrolNode')
        self.get_logger().info('PatrolNode: 초기화')

        # tb0 navigator
        self.dock_navigator_photo = TurtleBot4Navigator(namespace=TB0_NAMESPACE)
        self.nav_navigator_photo = BasicNavigator(namespace=TB0_NAMESPACE)
        
        # tb1 navigator
        self.dock_navigator_patrol = TurtleBot4Navigator(namespace=TB1_NAMESPACE)
        self.nav_navigator_patrol = BasicNavigator(namespace=TB1_NAMESPACE)
        
        self.goal_poses: List[PoseStamped] = []
        self.PHOTO_POSE: Optional[PoseStamped] = None
        
        self._init_sequence()

    def _init_sequence(self) -> None:
        self._set_init_pose()
        self._undock()
        self._set_goal_poses()

    def _set_init_pose(self) -> None:
        pose = self._create_pose(*self.INIT_POSE)
        self.nav_navigator_patrol.setInitialPose(pose)
        self.get_logger().info(f'초기 위치 설정... {int(self.INIT_LOADING_TIME)}s')
        time.sleep(self.INIT_LOADING_TIME)
        self.nav_navigator_patrol.waitUntilNav2Active()
        self.get_logger().info('Nav2 시스템 활성화됨')

    def _undock(self) -> None:
        if self.dock_navigator_patrol.getDockedStatus():
            self.get_logger().info('도킹 상태 → 언도킹 시도')
            self.dock_navigator_patrol.undock()
        else:
            self.get_logger().info('이미 언도킹 상태')

    def _set_goal_poses(self) -> None:
        self.goal_poses = [self._create_pose(*pose) for pose in self.GOAL_POSES]
        self.get_logger().info('순찰 경로 설정 완료')

    def _create_pose(self, x: float, y: float, yaw_deg: float) -> PoseStamped:
        """x, y, yaw(degree) → PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, np.deg2rad(yaw_deg))
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        return pose

    def move_generator(self) -> iter:
        i = 0
        while i < len(self.goal_poses):
            try:
                self._move_once(i, self.goal_poses[i])
                i += 1
                yield i, "SUCCEEDED"
            except self.AccidentDetected as e:
                self.get_logger().warn(str(e))
                yield i, "DISPATCHED"
            except self.PatrolFailure as e:
                self.get_logger().error(str(e))
                yield i, "RECOVERED"
        yield i, "DONE"

    def _move_once(self, i: int, goal: PoseStamped) -> None:
        self.get_logger().info(f'{i+1}번째 순찰지로 이동')
        self.nav_navigator_patrol.goToPose(goal)
        while not self.nav_navigator_patrol.isTaskComplete():
            feedback = self.nav_navigator_patrol.getFeedback()
            if feedback:
                self.get_logger().info(f'남은 거리: {feedback.distance_remaining:.2f} m')
        result = self.nav_navigator_patrol.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f'{i+1}번째 순찰 완료')
        elif result == TaskResult.CANCELED:
            raise self.AccidentDetected(f'{i+1}번째 순찰지: 이동 취소')
        else:
            raise self.PatrolFailure(f'{i+1}번째 순찰 실패: {result}')

    def cancel(self) -> None:
        if not self.nav_navigator_patrol.isTaskComplete():
            self.nav_navigator_patrol.cancelTask()
            self.get_logger().warn('순찰 목표 취소 요청됨')

    def dispatch(self, navigator: BasicNavigator, dispatch_pose: PoseStamped) -> None:
        navigator.get_logger().info('출동 명령 수행')
        navigator.goToPose(dispatch_pose)
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                navigator.get_logger().info(f'출동: 남은 거리 {feedback.distance_remaining:.2f} m')
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info('출동 완료')
        else:
            raise self.PatrolFailure(f'출동 실패: {result}')

    def recovery(self, i: int) -> None:
        self.get_logger().error(f'{i+1}번 위치로 재이동 시도')
        pose = self._create_pose(*self.GOAL_POSES[i])
        self.nav_navigator_patrol.goToPose(pose)
        while not self.nav_navigator_patrol.isTaskComplete():
            feedback = self.nav_navigator_patrol.getFeedback()
            if feedback:
                self.get_logger().info(f'복구 중, 남은 거리: {feedback.distance_remaining:.2f} m')
        result = self.nav_navigator_patrol.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('복구 위치 도달')
        else:
            self.get_logger().error(f'복구 실패: {result}')
            raise self.PatrolFailure(f'복구 실패: {result}')

    def dock(self) -> None:
        self.get_logger().info('도킹 시도...')
        self.dock_navigator_patrol.dock()

    def terminate(self) -> None:
        self.get_logger().info('노드 종료')
        self.dock_navigator_patrol.destroy_node()
        self.nav_navigator_patrol.destroy_node()


class DetectionNode(Node):
    """YOLO 객체 탐지 및 사고 감지 ROS2 Node"""

    MODEL_REL_PATH = os.path.join('rokey_ws', 'model', 'best.pt')
    RGB_TOPIC = TB0_NAMESPACE + "/oakd/rgb/preview/image_raw"
    TARGET_CLASS_ID = [0, 3]
    INFERENCE_PERIOD_SEC = 1.0 / 30

    def __init__(self):
        super().__init__('DetectionNode')
        self.HOME_PATH = os.path.expanduser("~")
        self.MODEL_PATH = os.path.join(self.HOME_PATH, self.MODEL_REL_PATH)
        self.bridge = CvBridge()
        self.latest_rgb: Optional[np.ndarray] = None
        self.latest_rgb_msg: Optional[Image] = None
        self.rgb_info: List[Dict[str, Any]] = []
        self.display_rgb: Optional[np.ndarray] = None
        self.lock = threading.Lock()
        self.model = self._load_model()
        self.class_names = getattr(self.model, "names", [])
        self.create_subscription(Image, self.RGB_TOPIC, self._rgb_callback, 10)
        self.create_timer(self.INFERENCE_PERIOD_SEC, self._inference_callback)

    def _load_model(self):
        if not os.path.exists(self.MODEL_PATH):
            self.get_logger().error(f"Model file not found: {self.MODEL_PATH}")
            sys.exit(1)
        model = YOLO(self.MODEL_PATH)
        model.to("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"YOLO 모델 로드 (GPU 사용: {torch.cuda.is_available()})")
        return model

    def _rgb_callback(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            with self.lock:
                self.latest_rgb = img.copy()
                self.display_rgb = img.copy()
                self.latest_rgb_msg = msg
        except Exception as e:
            self.get_logger().error(f"RGB 변환 오류: {e}")

    def _inference_callback(self):
        with self.lock:
            rgb, rgb_msg = (self.latest_rgb.copy() if self.latest_rgb is not None else None,
                            self.latest_rgb_msg)
        if rgb is None or rgb_msg is None:
            return
        results = self.model(rgb, stream=True)
        rgb_info = []
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls = int(box.cls[0])
                if cls not in self.TARGET_CLASS_ID or float(box.conf[0]) < MIN_CONFIDENCE:
                    continue
                label = self.class_names[cls] if cls < len(self.class_names) else f'class_{cls}'
                conf = float(box.conf[0])
                u, v = map(int, box.xywh[0][:2].cpu().numpy())
                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                rgb_info.append({
                    "label": label,
                    "conf": conf,
                    "center": (u, v),
                    "bbox": (x1, y1, x2, y2),
                    "header": rgb_msg.header
                })
        with self.lock:
            self.rgb_info = rgb_info


class TransformNode(Node):
    """RGB+Depth를 3D map좌표로 변환, marker publish, TF변환"""

    DEPTH_TOPIC = TB0_NAMESPACE + "/oakd/stereo/image_raw"
    CAMERA_INFO_TOPIC = TB0_NAMESPACE + "/oakd/rgb/camera_info"
    MARKER_TOPIC = TB0_NAMESPACE + "/detected_objects_marker"
    INIT_LOADING_TIME = 5.0

    def __init__(self):
        super().__init__('TransformNode')
        self.bridge = CvBridge()
        self.K: Optional[np.ndarray] = None
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_info: Optional[CameraInfo] = None
        self.lock = threading.Lock()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        time.sleep(self.INIT_LOADING_TIME)
        self.create_subscription(Image, self.DEPTH_TOPIC, self._depth_callback, 1)
        self.create_subscription(CameraInfo, self.CAMERA_INFO_TOPIC, self._camera_info_callback, 1)
        self.marker_pub = self.create_publisher(Marker, self.MARKER_TOPIC, 10)
        self.marker_id = 0
        self.depth_info: List[Dict[str, Any]] = []

    def _depth_callback(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            with self.lock:
                self.latest_depth = depth
        except Exception as e:
            self.get_logger().error(f"Depth 변환 오류: {e}")

    def _camera_info_callback(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"CameraInfo: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

    def _transform_to_map(self, point: PointStamped, label: str) -> Tuple[float, float, float]:
        try:
            map_point = self.tf_buffer.transform(point, "map", timeout=rclpy.duration.Duration(seconds=1.0))
            x, y, z = map_point.point.x, map_point.point.y, map_point.point.z
            self.get_logger().info(f"[TF] {label}→map: (x={x:.2f}, y={y:.2f}, z={z:.2f})")
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"[TF] {label} 변환 실패: {e}")
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
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 3
        self.marker_pub.publish(marker)

    def process_rgb(self, rgb_info: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        with self.lock:
            depth = self.latest_depth.copy() if self.latest_depth is not None else None
            K = self.K.copy() if self.K is not None else None
        if depth is None or K is None:
            return []
        depth_info = []
        for obj in rgb_info:
            u, v = obj['center']
            label = obj['label']
            header = obj['header']
            if not (0 <= v < depth.shape[0] and 0 <= u < depth.shape[1]):
                continue
            z = float(depth[v, u]) / 1000.0
            if z <= 0.05 or np.isnan(z):
                continue
            fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
            x = (u - cx) * z / fx
            y = (v - cy) * z / fy
            point = PointStamped()
            point.header = header
            point.point.x, point.point.y, point.point.z = x, y, z
            obj_x, obj_y, obj_z = self._transform_to_map(point, label)
            if not np.isnan(obj_x):
                self._publish_marker(obj_x, obj_y, obj_z, label)
            depth_info.append({
                "depth": z,
                "obj_x": obj_x,
                "obj_y": obj_y,
                "obj_z": obj_z
            })
        with self.lock:
            self.depth_info = depth_info
        return depth_info


class DispatchNode(Node):
    """출동, 명령 송신, RGB→PCD 변환 및 종료 Node (ROS2 Node로 변경)"""

    DISPATCH_TOPIC = TB0_NAMESPACE + '/dispatch_command'

    def __init__(self):
        super().__init__('DispatchNode')
        self.dispatch_pub = self.create_publisher(Int32, self.DISPATCH_TOPIC, 10)
        self.msg = Int32()

    def turn_on(self):
        self.msg.data = 0  # active
        self.dispatch_pub.publish(self.msg)
        self.get_logger().info('Dispatch: turn_on (active)')

    def turn_off(self):
        self.msg.data = 1  # deactive
        self.dispatch_pub.publish(self.msg)
        self.get_logger().info('Dispatch: turn_off (deactive)')

    def convert(self):
        """RGB→PCD→HTML 변환"""
        converter = RGBDToPCDConverter()
        try:
            rclpy.spin(converter)
        except KeyboardInterrupt:
            converter.get_logger().info('Converter stopped by user.')
        finally:
            converter.destroy_node()


# ------------------ SERVER FSM CLASS ------------------

class Server1:
    """전체 FSM, 상태관리 및 모든 노드 통합"""

    class State:
        IDLE, PATROLING, DISPATCHING, RECOVERY, TERMINATED = 'IDLE', 'PATROLING', 'DISPATCHING', 'RECOVERY', 'TERMINATED'

    def __init__(self, executor: MultiThreadedExecutor):
        self.state = self.State.IDLE
        self.lock = threading.Lock()
        # 노드 인스턴스화
        self.patrol_node    = PatrolNode()
        self.detect_node    = DetectionNode()
        self.transform_node = TransformNode()
        self.dispatch_node  = DispatchNode()
        # ROS2 Executor에 노드 추가
        for node in [self.patrol_node, self.detect_node, self.transform_node, self.dispatch_node]:
            executor.add_node(node)
        self.executor_thread = threading.Thread(target=executor.spin, daemon=True)
        self.executor_thread.start()
        self.patrol_iter = self.patrol_node.move_generator()

    def set_state(self, new_state: str):
        with self.lock:
            self.state = new_state

    def get_state(self) -> str:
        with self.lock:
            return self.state

    def patrol(self):
        with self.lock:
            if self.state in [self.State.IDLE, self.State.DISPATCHING, self.State.RECOVERY]:
                self.state = self.State.PATROLING
            else:
                raise RuntimeError(f"patrol 호출시 상태 오류: {self.state}")
        for i, result in self.patrol_iter:
            if result == 'SUCCEEDED':
                pass
            elif result == 'DISPATCHED':
                self.set_state(self.State.DISPATCHING)
                return
            elif result == 'RECOVERED':
                self.patrol_node.recovery(i)
                self.set_state(self.State.RECOVERY)
                return
            else:
                self.set_state(self.State.TERMINATED)
                return

    def dispatch(self, photo_pose: PoseStamped):
        with self.lock:
            if self.state not in [self.State.PATROLING, self.State.RECOVERY]:
                raise RuntimeError(f"dispatch 호출시 상태 오류: {self.state}")
            self.state = self.State.DISPATCHING
        try:
            dispatch_pose = self.patrol_node._create_pose(*self.patrol_node.DISPATCH_POSE)
            self.patrol_node.dispatch(self.patrol_node.nav_navigator_patrol, dispatch_pose)
            self.dispatch_node.turn_on()
            self.patrol_node.dock_navigator_photo.undock()
            self.patrol_node.dispatch(self.patrol_node.nav_navigator_photo, photo_pose)
            self.dispatch_node.convert()
            dock_pose = self.patrol_node._create_pose(*self.patrol_node.DOCK_POSE)
            self.patrol_node.dispatch(self.patrol_node.nav_navigator_photo, dock_pose)
            self.patrol_node.dock_navigator_photo.dock()
            self.dispatch_node.turn_off()
            self.set_state(self.State.PATROLING)
        except PatrolNode.PatrolFailure as e:
            self.patrol_node.recovery(0)
            self.set_state(self.State.RECOVERY)

    def terminate(self):
        with self.lock:
            self.state = self.State.TERMINATED
        self.patrol_node.dock()
        self.patrol_node.terminate()
        self.detect_node.destroy_node()
        self.transform_node.destroy_node()
        self.dispatch_node.destroy_node()


# ------------------ MAIN ENTRY ------------------

def main():
    """엔트리포인트, UI/로직 분리, 안전종료"""
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=8)
    server1 = Server1(executor)
    try:
        while rclpy.ok():
            if server1.get_state() == Server1.State.PATROLING:
                threading.Thread(target=server1.patrol, daemon=True).start()
            # Detection + Transform: 데이터 복사로 안전하게 전달
            with server1.detect_node.lock:
                frame = server1.detect_node.display_rgb.copy() if server1.detect_node.display_rgb is not None else None
                rgb_info = server1.detect_node.rgb_info.copy()
            depth_info = server1.transform_node.process_rgb(rgb_info)
            # UI 표시
            if frame is not None and len(rgb_info) == len(depth_info):
                for obj, dep in zip(rgb_info, depth_info):
                    if obj["conf"] < DISPLAY_CONFIDENCE:
                        continue
                    u, v = obj["center"]
                    x1, y1, x2, y2 = obj["bbox"]
                    label = obj["label"]
                    conf = obj["conf"]
                    depth = dep.get("depth", 0.0)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)
                    cv2.putText(
                        frame,
                        f"{label}: {conf:.2f}, {depth:.2f}m",
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        2,
                    )
                display_img = cv2.resize(frame, (frame.shape[1]*2, frame.shape[0]*2))
                cv2.imshow("YOLO + Depth + TF", display_img)
                # 사고 감지시 출동
                for obj, dep in zip(rgb_info, depth_info):
                    if obj["conf"] < DISPLAY_CONFIDENCE: continue
                    obj_x, obj_y, obj_z = dep["obj_x"], dep["obj_y"], dep["obj_z"]
                    if np.isnan(obj_x): continue
                    photo_pose = server1.patrol_node._create_pose(obj_x, obj_y, 0.0)
                    if server1.get_state() == Server1.State.PATROLING:
                        threading.Thread(target=server1.dispatch, args=(photo_pose,), daemon=True).start()
                        break
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(0.01)
    except Exception as e:
        print(f'Exception: {e}')
    finally:
        server1.terminate()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        sys.exit(0)

if __name__ == '__main__':
    main()
