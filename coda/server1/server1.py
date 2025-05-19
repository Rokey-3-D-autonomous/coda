import rclpy
from rclpy.node import Node

from rclpy.time import Time as RclTime
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger

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
from coda.server2.pcd_to_html import RGBDToPCDConverter

import threading, os, sys, time

TB0_NAMESPACE = '/robot0'   # photo
TB1_NAMESPACE = '/robot1'   # patrol

class PatrolNode:

    INIT_POSE = [0.25, 0.71, 270.0]     # docking station
    GOAL_POSES = [
        # start to patrol
        [0.0350, 0.6523, 180.0],        # point 1
        [-0.0619, -1.6371, 180.0],      # point 2
        [0.4564, -0.8648, 270.0],       # point 3
        [0.8353, -1.6557, 0.0],         # point 4
        [1.0313, 0.6398, 0.0],          # front of docking station - tb1
    ]
    DISPATCH_POSE = [0.0, 0.0, 0.0]     # central of map
    PHOTO_POSE = None
    DOCK_POSE = [0.5, 0.5, 0.0]         # front of docking station - tb0

    INIT_LOADING_TIME = 5.0

    SUCCEEDED   = 'SUCCEEDED'
    DISPATCHED  = 'DISPATCHED'
    RECOVERED   = 'RECOVERED'
    DONE        = 'DONE'

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
        self.dock_navigator_photo = TurtleBot4Navigator(namespace=TB0_NAMESPACE)
        self.nav_navigator_photo = BasicNavigator(namespace=TB0_NAMESPACE)
        self.dock_navigator_patrol = TurtleBot4Navigator(namespace=TB1_NAMESPACE)
        self.nav_navigator_patrol = BasicNavigator(namespace=TB1_NAMESPACE)
        self.patrol_logger.info('create navigators')

        self._get_init_pose()
        self._undock()
        self._get_goal_poses()
        self.patrol_logger.info('configure PatrolNode')

    def _create_pose(self, x, y, yaw_deg) -> PoseStamped:
        """x, y, yaw(degree) → PoseStamped 생성"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.nav_navigator_patrol.get_clock().now().to_msg()
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
        self.nav_navigator_patrol.setInitialPose(initial_pose)
        self.nav_navigator_patrol.get_logger().info(f'초기 위치 설정 중... {int(self.INIT_LOADING_TIME)}s')
        time.sleep(self.INIT_LOADING_TIME) #AMCL이 초기 pose 처리 시 필요한 시간과 TF를 얻을 수 있게 됨
        
        self.nav_navigator_patrol.waitUntilNav2Active()
        self.nav_navigator_patrol.get_logger().info(f'네비게이션 시스템 작동 중...')

    def _undock(self) -> None:
        if self.dock_navigator_patrol.getDockedStatus():
            self.dock_navigator_patrol.get_logger().info('현재 도킹 상태 → 언도킹 시도')
            self.dock_navigator_patrol.undock()
        else:
            self.dock_navigator_patrol.get_logger().info('언도킹 상태에서 시작')

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
                self.nav_navigator_patrol.get_logger().warn(str(e))
                yield i, self.DISPATCHED
            except self.PatrolFailure as e:
                self.nav_navigator_patrol.get_logger().error(str(e))
                yield i, self.RECOVERED
        yield i, self.DONE

    def _get_feedback(self, navigaotr: BasicNavigator, log_msg: str) -> None:
        while not navigaotr.isTaskComplete():
            feedback = navigaotr.getFeedback()
            if feedback:
                navigaotr.get_logger().info(f'{log_msg}, 남은 거리: {feedback.distance_remaining:.2f} m')
    
    def _move_once(self, i: int, goal: PoseStamped) -> None:
        self.nav_navigator_patrol.get_logger().info(f'{i+1} 순찰지 순찰 시작')
        
        # 이동 명령
        self.nav_navigator_patrol.goToPose(goal)

        # 이동 중 사고 감지 시, 출동.
        self._get_feedback(self.nav_navigator_patrol, f'{i+1}번째 순찰지 이동 중')

        # 목표 지점까지 이동 완료
        result = self.nav_navigator_patrol.getResult()
        if result == TaskResult.SUCCEEDED:
            if i < 4:
                self.nav_navigator_patrol.get_logger().info(f'{i+1}번째 순찰지 도달.')
            else:
                self.nav_navigator_patrol.get_logger().info(f'순찰 완료. 복귀 중...')
        elif result == TaskResult.CANCELED:
            raise self.AccidentDetected(f'{i+1}번째 순찰지 이동 취소됨')
        else:
            raise self.PatrolFailure(f'{i+1}번째 순찰지 이동 실패. 상태: {result}')

    def cancel(self) -> None:
        if not self.nav_navigator_patrol.isTaskComplete():
            self.nav_navigator_patrol.cancelTask()
            self.nav_navigator_patrol.get_logger().warn('현재 이동 목표 취소 요청됨')
        
    def dispatch(self, navigator: BasicNavigator, dispatch_pose) -> None:
        navigator.get_logger().info(f'사고 발생: 차량 통제를 위해 출동 및 현장 촬영')
        # dispatch_pose = self._create_pose(*self.DISPATCH_POSE)

        # 출동 명령
        navigator.goToPose(dispatch_pose)
        navigator.get_logger().info('출동 시작')

        # 출동 중
        self._get_feedback(navigator, '출동지 이동 중')

        # 목표 지점까지 출동 완료
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info('출동지 도달. 임무 수행')
        else:
            raise self.PatrolFailure(f'출동지 이동 실패. 상태: {result}')

    def recovery(self, i: int) -> None:
        self.nav_navigator_patrol.get_logger().error(f'오류 발생: {i+1}번 위치로 재이동 시도')
        docking_station_pose = self._create_pose(*self.GOAL_POSES[i])
        
        # 복구 명령
        self.nav_navigator_patrol.goToPose(docking_station_pose)
        self.nav_navigator_patrol.get_logger().info('복구 시작')

        # 복구 중
        self._get_feedback(self.nav_navigator_patrol, '복구 중')

        # 복구 완료
        result = self.nav_navigator_patrol.getResult()
        if result == TaskResult.SUCCEEDED:
            self.nav_navigator_patrol.get_logger().info('복구 위치 도달 성공')
        else:
            self.nav_navigator_patrol.get_logger().error(f'복구 위치 도달 실패. 코드: {result}')
            raise self.PatrolFailure(f'복구 실패. 상태: {result}')

    def dock(self) -> None:
        self.dock_navigator_patrol.get_logger().info('복귀 완료. 도킹 시도...')
        self.dock_navigator_patrol.dock()

    def terminate(self) -> None:
        self.dock_navigator_patrol.get_logger().info('도킹 완료. 종료...')
        self.dock_navigator_patrol.destroy_node()
        self.nav_navigator_patrol.get_logger().info('순찰 완료. 종료...')
        self.nav_navigator_patrol.destroy_node()

    pass

class DetectionNode(Node):

    HOME_PATH = os.path.expanduser("~")                                     # 홈 디렉토리 경로
    MODEL_PATH = os.path.join(HOME_PATH, 'rokey_ws', 'model', 'best.pt')    # 모델 경로

    RGB_TOPIC = TB0_NAMESPACE + "/oakd/rgb/preview/image_raw"

    TARGET_CLASS_ID = [0, 3]
    INFERENCE_PERIOD_SEC = 1.0 / 30
    
    def __init__(self, detection_callback):
        super().__init__('Detection_Node')
        self.get_logger().info("[1/3] Detection Node 노드 초기화 시작...")

        self.detection_callback = detection_callback    # 사고 감지 콜백 함수
        self.bridge = CvBridge()
        self.latest_rgb = self.latest_rgb_msg = None
        self.rgb_info = []
        self.display_rgb = None
        self.lock = threading.Lock()

        if not os.path.exists(self.MODEL_PATH):
            self.get_logger().error(f"Model file not found: {self.MODEL_PATH}")
            sys.exit(1)

        self.model = YOLO(self.MODEL_PATH)
        self.model.to("cuda" if torch.cuda.is_available() else "cpu")
        self.class_names = getattr(self.model, "names", [])
        self.get_logger().info(f"[2/3] YOLO 모델 로드 완료 (GPU 사용: {torch.cuda.is_available()})")

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
        rgb_info = []

        for result in results:
            if result.boxes is None:
                continue

            for box in result.boxes:
                cls = int(box.cls[0])
                if cls not in self.TARGET_CLASS_ID:
                    continue

                if float(box.conf[0]) < 0.6:
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
                self.get_logger().info(f'overlay_info: {rgb_info}')

        with self.lock:
            self.rgb_info = rgb_info
    pass

class TransformNode(Node):

    INIT_LOADING_TIME = 5.0

    DEPTH_TOPIC = TB0_NAMESPACE + "/oakd/stereo/image_raw"
    CAMERA_INFO_TOPIC = TB0_NAMESPACE + "/oakd/rgb/camera_info"
    MARKER_TOPIC = TB0_NAMESPACE + "/detected_objects_marker"

    def __init__(self):
        super().__init__('Transform_Node')
        self.get_logger().info('Transform Node 초기환')

        self.bridge = CvBridge()
        self.K = None
        self.latest_depth = None
        self.latest_info = None
        self.lock = threading.Lock()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        time.sleep(self.INIT_LOADING_TIME)
        self.get_logger().info(f'[1/2] TF2 Transform Listener 초기화 대기... {int(self.INIT_LOADING_TIME)}s')

        self.create_subscription(Image, self.DEPTH_TOPIC, self._depth_callback, 1)
        self.create_subscription(CameraInfo, self.CAMERA_INFO_TOPIC, self._camera_info_callback, 1)
        self.get_logger().info(f"[2/3] 토픽 구독 완료:\n  Depth: {self.DEPTH_TOPIC}\n  CameraInfo: {self.CAMERA_INFO_TOPIC}")

        self.marker_pub = self.create_publisher(Marker, self.MARKER_TOPIC, 10)
        self.marker_id = 0
        self.get_logger().info(f"[3/3] 퍼블리셔 설정 완료\n  MAKER: {self.MARKER_TOPIC}")

    def _depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            with self.lock:
                self.latest_depth = depth
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")

    def _camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"CameraInfo 수신: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

    def _transform_to_map(self, point: PointStamped, label: str):
        try:
            map = self.tf_buffer.transform(point, "map", timeout=rclpy.duration.Duration(seconds=1.0))
            x, y, z = map.point.x, map.point.y, map.point.z
            self.get_logger().info(f"[TF] {label} → map: (x={x:.2f}, y={y:.2f}, z={z:.2f})")
            return x, y, z
        except Exception as e:
            self.get_logger().warn(f"[TF] class={label} 변환 실패: {e}")
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
        
    def process_rgb(self, rgb_info):
        with self.lock:
            depth = self.latest_depth
            K = self.K

        if depth is None or K is None:
            return

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

            depth_info.append({
                "depth": z,
                "obj_x": obj_x,
                "obj_y": obj_y,
                "obj_z": obj_z
            })

        with self.lock:
            self.depth_info = depth_info
    pass

class DispatchNode():

    DISPATCH_TOPIC = TB0_NAMESPACE + '/dispatch_command'

    def __init__(self):
        super().__init__('Dispatch Node')
        self.get_logger().info('Dispatch Node 초기환')

        self.tmp_node = Node('tmp_node')
        self.tmp_pub = self.tmp_node.create_publisher(i32, self.DISPATCH_TOPIC, 10)
        self.msg = i32()

    def turn_on(self):
        # to tb1: ui + beep turn on topic pub : i32.data = 0
        self.msg.data = 0    # active
        self.tmp_pub.publish(self.msg)

    def turn_off(self):
        # to tb1: ui + beep turn on topic pub : i32.data = 1
        self.msg.data = 1    # deactive
        self.tmp_pub.publish(self.msg)

    def convert(self):
        # rgb → pcd → html
        converter = RGBDToPCDConverter()
        try:
            rclpy.spin(converter)
        except KeyboardInterrupt:
            converter.get_logger().info('Converter stopped by user.')
        finally:
            converter.destroy_node()
        pass

    def terminate(self):
        self.tmp_node.destroy_node()

    pass

class Server1:

    class ServerState:
        IDLE        = 'IDLE'
        PATROLING   = 'PATROLING'
        DISPATCHING = 'DISPATCHING'
        RECOVERY    = 'RECOVERY'
        TERMINATED  = 'TERMINATED'

    def __init__(self):
        self.server_logger = get_logger('Server1')
        self.server_state = self.ServerState.IDLE
        self.lock = threading.Lock()

        self.patrol_node    = PatrolNode()
        self.detect_node    = DetectionNode(self.patrol_node.cancel)    # detection 했을 떄, inference 잠시 중단
        self.transform_node = TransformNode()
        self.dispatch_node  = DispatchNode()

        # threading
        executor = MultiThreadedExecutor(num_threads=8)
        executor.add_node(self.detect_node)
        executor.add_node(self.transform_node)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # patrol
        self.patrol_iter = self.patrol_node.move_generator()

    # def set_state(self, new_state):
    #     with self.lock:
    #         self.server_state = new_state

    # def get_state(self):
    #     with self.lock:
    #         return self.server_state

    def patrol(self):
        with self.lock:
            if self.server_state in [self.ServerState.IDLE, self.ServerState.DISPATCHING, self.ServerState.RECOVERY]:
                self.server_state = self.ServerState.PATROLING
                self.server_logger.info(f'current server state {self.server_state}')
            else:
                self.server_logger.error(f'[{self.patrol.__name__}] : {self.server_state} is wrong!!')
                raise RuntimeError()

        for i, result in self.patrol_iter:
            self.server_logger.info(f'순찰 결과 : {result}')

            if result == 'SUCCEEDED':
                pass
            elif result == 'DISPATCHED':
                self.server_logger.info(f'{i+1}번째 순찰지로 향하던 중, 이동 목표 취소 요청 발생\n 상태 전이: {self.server_state} → {self.ServerState.DISPATCHING}')
                # self.dispatch()     # 상태 전이
                self.server_state = self.ServerState.DISPATCHING
                return
            elif result == 'RECOVERED':
                self.patrol_node.recovery(i)
                self.server_state = self.ServerState.RECOVERY
                return
            else:   # DONE
                self.server_logger.info(f'순찰지 {i+1}곳 순찰 완료\n 상태 전이: {self.server_state} → {self.ServerState.TERMINATED}')
                # self.terminate()    # 상태 전이
                self.server_state = self.ServerState.TERMINATED
                return

        pass

    def dispatch(self):
        with self.lock:
            if self.server_state in [self.ServerState.PATROLING, self.ServerState.RECOVERY]:
                self.server_state = self.ServerState.DISPATCHING
                self.server_logger.info(f'current server state {self.server_state}')
            else:
                self.server_logger.error(f'[{self.dispatch.__name__}] : {self.server_state} is wrong!!')
                raise RuntimeError()

        try:
            # t0:photo, 
            # t1:patrol 
            # 모두 출동 시켜야 함!!
            dispatch_pose = self.patrol_node._create_pose(*self.patrol_node.DISPATCH_POSE)
            self.patrol_node.dispatch(self.patrol_node.nav_navigator_patrol, dispatch_pose)     # t1 start
            self.dispatch_node.turn_on()

            self.patrol_node.dock_navigator_photo.undock()
            self.patrol_node.dispatch(self.patrol_node.nav_navigator_photo, self.patrol_node.PHOTO_POSE)     # t0 start
            self.dispatch_node.convert()
            dock_pose = self.patrol_node._create_pose(*self.patrol_node.DOCK_POSE)
            self.patrol_node.dispatch(self.patrol_node.nav_navigator_photo, dock_pose)     # t0 return
            self.patrol_node.dock_navigator_photo.dock()

            self.dispatch_node.turn_off()

            # restart patrol
            self.server_state = self.ServerState.PATROLING
        except PatrolNode.PatrolFailure as e:
            self.server_logger.error(f'dispatch error: {e}')
            self.patrol_node.recovery(0)

            self.server_state = self.ServerState.RECOVERY
        pass

    def recovery(self):
        with self.lock:
            if self.server_state in [self.ServerState.PATROLING, self.ServerState.DISPATCHING]:
                self.server_state = self.ServerState.RECOVERY
                self.server_logger.info(f'current server state {self.server_state}')
            else:
                self.server_logger.error(f'[{self.dispatch.__name__}] : {self.server_state} is wrong!!')
                raise RuntimeError()
        
        # recovery

        pass

    def terminate(self):
        with self.lock:
            self.server_state = self.ServerState.TERMINATED
            self.server_logger.info(f'current server state {self.server_state}')

        self.server_logger.info('순찰 종료')
        self.patrol_node.dock()
        self.patrol_node.terminate()

        self.server_logger.info('출동 종료')
        self.patrol_node.terminate()

        self.server_logger.info('탐지 종료')
        # update

        self.server_logger.info('변환 종료')
        # update

        # detroy ros2 node
        self.detect_node.destroy_node()
        self.transform_node.destroy_node()

        pass

def main():
    rclpy.init()

    try:
        server1 = Server1()

        yolo_node = server1.detect_node()
        tf_node = server1.transform_node()

        while rclpy.ok():
            with server1.lock:
                if server1.server_state == server1.ServerState.PATROLING:
                    threading.Thread(target=server1.patrol, daemon=True).start()
            
            with yolo_node.lock:
                frame = yolo_node.display_rgb.copy() if yolo_node.display_rgb is not None else None
                rgb_info = yolo_node.rgb_info.copy()
                tf_node.process_rgb(rgb_info)

            with tf_node.lock:
                depth_info = tf_node.depth_info.copy()
                depth, obj_x, obj_y, obj_z = depth_info['depth'], depth_info['obj_x'], depth_info['obj_y'], depth_info['obj_z']

            if frame is not None:
                for obj in rgb_info:
                    if obj["conf"] < 0.65:
                        continue
                    
                    try:
                        with server1.lock:
                            if server1.server_state == server1.ServerState.PATROLING:
                                threading.Thread(target=yolo_node.detection_callback, daemon=True).start()
                                
                                # set pose with [obj_x, obj_y, obj_z]
                                server1.patrol_node.PHOTO_POSE = server1.patrol_node._create_pose(obj_x, obj_y, 0.0)
                                
                                threading.Thread(target=server1.dispatch, daemon=True).start()
                    except RuntimeError as e:
                        server1.server_logger.error(f'dispatch error: {e}')
                        pass
                        
                    u, v = obj["center"]
                    x1, y1, x2, y2 = obj["bbox"]
                    label = obj["label"]
                    conf = obj["conf"]

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)
                    cv2.putText(
                        frame,
                        f"{label}: {conf:.2f}, {depth}m",
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        2,
                    )

                display_img = cv2.resize(frame, (frame.shape[1]*2, frame.shape[0]*2))
                cv2.imshow("YOLO + Depth + TF", display_img)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(0.01)
    except RuntimeError as e:
        print(f'RuntimeError: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        server1.terminate()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        sys.exit(0)

if __name__ == '__main__':
    main()