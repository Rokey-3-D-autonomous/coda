##객체탐지, 사고 감지시 퍼블리시
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Int32  ###객체 탐지시 메시지타입 추가
import tf2_ros
import tf2_geometry_msgs  # 꼭 필요
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2, torch
import threading
import os
import sys
import time

# qos 추가
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos_profile_1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
qos_profile_5 = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
qos_profile_10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
qos_profile_10_default = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


# ========================
# 상수 정의
# ========================
YOLO_MODEL_PATH = '/home/rokey/rokey_ws/model/best_250520_v10n.pt'  # YOLO 모델 경로

# 실행 전 ns 확인
ROBOT_NAMESPACE = 'robot1' 

RGB_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/rgb/image_raw'
DEPTH_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/stereo/image_raw'
CAMERA_INFO_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/stereo/camera_info'
TARGET_CLASS_ID = [0, 1]
# ========================

class YoloDepthDistance(Node):
    def __init__(self):
        super().__init__('yolo_depth_distance')
        self.get_logger().info("YOLO + Depth 거리 출력 노드 시작")

        # YOLO 모델 로드
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if not os.path.exists(YOLO_MODEL_PATH):
            self.get_logger().error(f"YOLO 모델이 존재하지 않습니다: {YOLO_MODEL_PATH}")
            sys.exit(1)
        self.model = YOLO(YOLO_MODEL_PATH).to(device)
        self.class_names = getattr(self.model, 'names', [])

        self.bridge = CvBridge()
        self.K = None
        self.rgb_image = None
        self.depth_image = None
        self.lock = threading.Lock()
        self.inference_results = None   #추가

        # ROS 구독자 설정
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self.camera_info_callback, qos_profile_1)
        self.create_subscription(Image, RGB_TOPIC, self.rgb_callback, qos_profile_1)
        self.create_subscription(Image, DEPTH_TOPIC, self.depth_callback, qos_profile_1)

        # 퍼블리셔 설정
        self.accident_pub = self.create_publisher(Int32, f'/{ROBOT_NAMESPACE}/accident_detected', qos_profile_10)
        self.last_publish_time = 0.0
        self.accident_pos_pub = self.create_publisher(Point, f'/{ROBOT_NAMESPACE}/accident_position', qos_profile_10)

        # YOLO + 거리 출력 루프 실행
        threading.Thread(target=self.processing_loop, daemon=True).start()

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info("CameraInfo 수신 완료")

    def rgb_callback(self, msg):
        with self.lock:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def depth_callback(self, msg):
        with self.lock:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def processing_loop(self):
        cv2.namedWindow("YOLO Distance View", cv2.WINDOW_NORMAL)

        while rclpy.ok():
            with self.lock:
                if self.rgb_image is None or self.depth_image is None or self.K is None:
                    continue
                rgb = self.rgb_image.copy()
                depth = self.depth_image.copy()
   
            # YOLO 추론 수행
            results = list(self.model(rgb, stream=True))
            accident_detected = False  #####추가_사고감지여부 초기화
            
            for r in results:
                for box in r.boxes:
                    cls = int(box.cls[0])
                    if cls not in TARGET_CLASS_ID:
                        continue

                    if cls == 1:  ####추가_사고 클래스 ID
                        accident_detected = True

                    # 중심 좌표
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2

                    if not (0 <= u < depth.shape[1] and 0 <= v < depth.shape[0]):
                        continue

                    # 거리 계산 (mm → m)
                    val = depth[v, u]
                    if depth.dtype == np.uint16:
                        distance_m = val / 1000.0
                    else:
                        distance_m = float(val)

                    cam_x, cam_y = self.calc_xy(u,v,distance_m)

                    self.get_logger().info(f"x:{cam_x} y:{cam_y}")


                    # map_x, map_y, map_z= self.tf_cam2map(cam_x,cam_y, distance_m)
                    label = self.class_names[cls] if cls < len(self.class_names) else f'class_{cls}'

#=====
                    # RGB 이미지 위 시각화
                    cv2.rectangle(rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(rgb, (u, v), 4, (0, 0, 255), -1)
                    cv2.putText(rgb, f"{distance_m:.2f}m", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                    cv2.putText(rgb, f"{label}", (x1, y1 - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            #### 사고 감지 퍼블리시 타이밍(주기)
            if accident_detected:
                now = time.time()
                if now - self.last_publish_time > 3.0:
                    msg = Int32()
                    msg.data = 1
                    self.accident_pub.publish(msg)
                    self.get_logger().info("사고 감지!! /accident_detected → 1")
                    self.last_publish_time = now
                    
                    # 사고 감지 지점 topic 전송
                    point_msg = Point()
                    point_msg.x = float(cam_x)
                    point_msg.y = float(cam_y)
                    point_msg.z = distance_m  # z 축 (카메라 기준 거리)
                    self.accident_pos_pub.publish(point_msg)
                    self.get_logger().info(f"사고 좌표 전송: x={cam_x:.2f}, y={cam_y:.2f}, z={distance_m:.2f}")

            cv2.imshow("YOLO Distance View", rgb)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def calc_xy(self, u, v, d_m):
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        # 좌표 변환
        cam_x = (u - cx) * d_m / fx
        cam_y = (v - cy) * d_m / fy
            
        return cam_x,cam_y

    # ## 사용X
    # def tf_cam2map(self, x, y, z):
    #     try:
    #         # base_link 기준 포인트 생성
    #         point_cam = PointStamped()
    #         point_cam.header.stamp = rclpy.time.Time().to_msg()
    #         point_cam.header.frame_id = 'oakd_rgb_camera_optical_frame'
    #         point_cam.point.x = x #1m 앞
    #         point_cam.point.y = y
    #         point_cam.point.z = z


    #         # base_link → map 변환
    #         try:
    #             point_map = self.tf_buffer.transform(
    #                 point_cam,
    #                 'map',
    #                 timeout=rclpy.duration.Duration(seconds=0.5)
    #             )
    #             return point_map.point.x, point_map.point.y, point_map.point.z
            
    #         except Exception as e:
    #             self.get_logger().warn(f"TF transform to map failed: {e}")
    #             return None, None, None

                
    #     except Exception as e:
    #         self.get_logger().warn(f"Unexpected error: {e}")
    #     return None, None, None

# ========================
# 메인 함수
# ========================
def main():
    rclpy.init()
    node = YoloDepthDistance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()