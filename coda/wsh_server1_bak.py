from enum import Enum

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32 as i32
from std_msgs.msg import Float64 as f64
from geometry_msgs.msg import PointStamped, PoseStamped, Point

import tf2_ros
import tf2_geometry_msgs  # 꼭 필요


# system status
class STATUS(Enum):
    """Docstring for MyEnum."""

    READY_FLAG = 0  # 준비
    PATROL_FLAG = 0  # 순찰
    DETECTED_FLAG = 0  # 감지
    DISPATCH_FLAG = 0  # 출동
    EXIT_FLAG = 0  # 종료


# robot namespace
TB0_NAMESPACE = "/robot0"  # photo
TB1_NAMESPACE = "/robot1"  # patrol

# pub/sub topic names
NAV0_PUB = TB0_NAMESPACE + "/goal_position"
NAV0_SUB = TB0_NAMESPACE + "/goal_result"
NAV1_PUB = TB1_NAMESPACE + "/goal_position"
NAV1_SUB = TB1_NAMESPACE + "/goal_result"
CV_SUB_DETECTED = "/accident_detected"
CV_SUB_POSITION = "/accident_position"
UI_ALARM = TB1_NAMESPACE + "/dispatch_command"
PHOTO_PUB = "/photo"
SERVER_SUB = "/control_scenario"


class Server(Node):
    def __init__(self):
        super().__init__("yolo_depth_to_map")

        self.nav0_current_position = 0
        self.nav1_current_position = 0
        self.nav1_accident_position = (
            1  # ([-1.76, 3.77], TurtleBot4Directions.NORTH),    # 6
        )

        # TF 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("tf 안정화(5초)")
        time.sleep(5)
        self.get_logger().info("tf 안정화 완료")

        # self.get_logger().info("[1/5] 노드 초기화 시작...")
        self.get_logger().info("[1/5] Initialize server node...")

        self.status: STATUS = STATUS.READY_FLAG  # initialize status

        # ======= topics ======= #
        # nav0
        self.nav0_pub = self.create_publisher(i32, NAV0_PUB, 10)
        self.nav0_pub2 = self.create_publisher(
            Point, NAV0_PUB + "2", 10
        )  # yolo에서 탐지한 좌표로 이동
        self.nav0_sub = self.create_subscription(
            i32, NAV0_SUB, self._nav0_sub_callback, 10
        )
        # nav1
        self.nav1_pub = self.create_publisher(i32, NAV1_PUB, 10)
        self.nav1_sub = self.create_subscription(
            i32, NAV1_SUB, self._nav1_sub_callback, 10
        )
        # cv
        self.cv_sub = self.create_subscription(
            i32, CV_SUB_DETECTED, self._cv_suv_detected_callback, 10
        )
        self.cv_sub = self.create_subscription(
            Point, CV_SUB_POSITION, self._cv_suv_position_callback, 10
        )

        # ui/alarm
        self.ui_alarm_pub = self.create_publisher(i32, UI_ALARM, 10)

        # pcd
        self.pcd_pub = self.create_publisher(i32, PHOTO_PUB, 10)

        # control scenario
        self.control_scenario_sub = self.create_subscription(
            i32, SERVER_SUB, self.control_scenario, 10
        )

    def get_status(self) -> STATUS:
        return self.status

    def set_status(self, new_status: STATUS):
        self.status = new_status

    def make_msg(self, data):
        msg = i32()
        msg.data = data
        return msg

    def control_scenario(self, msg):
        data = msg.data
        if data == 0:
            self.get_logger().info("ready")
            self.ready()
        elif data == 1:
            self.get_logger().info("patrol")
            self.patrol()
        elif data == 2:
            self.get_logger().info("detected")
            self.detected()
        elif data == 3:
            self.get_logger().info("dispatch")
            self.dispatch()
        elif data == 4:
            self.get_logger().info("exit_scenario")
            self.exit_scenario()
        elif data == 5:
            self.get_logger().info("dispatch_test")
            self.dispatch_test()
        else:
            self.get_logger().info("all_stop")
            self.all_stop()
        pass

    def ready(self):
        self.set_status(STATUS.READY_FLAG)
        self.get_logger().info("[1/5] Ready to patrol...")

    def patrol(self):
        self.set_status(STATUS.PATROL_FLAG)
        self.get_logger().info("[2/5] Patrol...")

        # 완료
        self.nav1_pub.publish(self.make_msg(self.nav1_current_position))
        self.nav1_current_position += 1

    def detected(self):
        self.set_status(STATUS.DETECTED_FLAG)
        self.get_logger().info("[3/5] Detected...")

        # 완료
        self.ui_alarm_pub.publish(self.make_msg(0))  # alarm on

        # 완료
        self.nav1_pub.publish(
            self.make_msg(self.nav1_accident_position)
        )  # 차량 통제 위치로 가라
        self.nav1_current_position -= 1  # 순찰 미완료로 원래 위치 저장

    def dispatch(self):
        self.set_status(STATUS.DISPATCH_FLAG)
        self.get_logger().info("[4/5] Dispatch...")

        # 촬영 위치 pose
        point = Point()
        # point.x, point.y, point.z = self.point
        point.x, point.y, point.z = [-1.67, 1.54, 0.0]  # 7
        self.nav0_pub2.publish(point)  # 촬영 위치로 가라

        # pcd
        self.pcd_pub.publish(self.make_msg(0))  # 사진 촬영

        point.x, point.y, point.z = [-1.61, -0.38, 0.0]  # 8
        self.nav0_pub2.publish(point)  # 복귀해라

        # 완료
        self.ui_alarm_pub.publish(self.make_msg(1))  # alarm off

    def dispatch_test(self):
        self.set_status(STATUS.DISPATCH_FLAG)
        self.get_logger().info("[4/5] Dispatch...")

        # self.pcd_pub.publish(self.make_msg(0))
        self.nav0_pub.publish(self.make_msg(2))  # 8
        self.nav0_pub.publish(self.make_msg(3))  # 7
        self.nav0_pub.publish(self.make_msg(2))  # 8

        # 완료
        self.ui_alarm_pub.publish(self.make_msg(1))  # alarm off

    def exit_scenario(self):
        self.set_status(STATUS.EXIT_FLAG)
        self.get_logger().info("[5/5] Exit...")

        self.nav0_pub.publish(self.make_msg(0))

        # 완료
        self.nav1_pub.publish(self.make_msg(0))
        self.ui_alarm_pub.publish(self.make_msg(1))  # alarm off

    def all_stop(self):
        pass

    def _nav0_sub_callback(self, msg):
        self.get_logger().info(f"_nav0_sub_callback: {msg}")
        pass

    def _nav1_sub_callback(self, msg):
        self.get_logger().info(f"_nav1_sub_callback: {msg}")
        pass

    def _cv_suv_detected_callback(self, msg):
        self.get_logger().info(f"_cv_suv_detected_callback: {msg}")
        self.get_logger().info(f"_cv_suv_detected_callback: Accident Detected!!!")
        pass

    def _cv_suv_position_callback(self, msg):
        self.get_logger().info(f"_cv_suv_position_callback: {msg}")
        x, y, z = self.tf_cam2map(msg.x, msg.y, msg.z)
        self.get_logger().info(f"x, y, z : {x}, {y}, {z}")
        self.point = [x, y, z]

    def tf_cam2map(self, x, y, z):
        try:
            # base_link 기준 포인트 생성
            point_cam = PointStamped()
            point_cam.header.stamp = rclpy.time.Time().to_msg()
            # point_cam.header.frame_id = 'oakd_imu_frame'
            point_cam.header.frame_id = "oakd_rgb_camera_optical_frame"
            point_cam.point.x = x  # 1m 앞
            point_cam.point.y = y
            point_cam.point.z = z

            # base_link → map 변환
            try:
                point_map = self.tf_buffer.transform(
                    point_cam, "map", timeout=rclpy.duration.Duration(seconds=0.5)
                )
                return point_map.point.x, point_map.point.y, point_map.point.z

            except Exception as e:
                self.get_logger().warn(f"TF transform to map failed: {e}")
                return None, None, None

        except Exception as e:
            self.get_logger().warn(f"Unexpected error: {e}")

        return None, None, None


def main():
    rclpy.init()

    server = Server()

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()
        print("server down")


if __name__ == "__main__":
    main()
