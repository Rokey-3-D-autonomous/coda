## qos 버전
from enum import Enum

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32 as i32
from std_msgs.msg import Float64 as f64
from geometry_msgs.msg import PointStamped, PoseStamped, Point

import tf2_ros
import tf2_geometry_msgs  # 꼭 필요

# qos 추가
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos_profile_1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
qos_profile_5 = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
qos_profile_10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
qos_profile_10_default = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)


# system status
class STATUS(Enum):
    """Docstring for MyEnum."""

    READY_FLAG = 0  # 준비
    PATROL_FLAG = 1  # 순찰
    DETECTED_FLAG = 2  # 감지
    DISPATCH_FLAG = 3  # 출동 종료
    EXIT_FLAG = 4  # 종료

    NAV_DONE = 0
    NAV_WORKING = 1


# robot namespace
TB0_NAMESPACE = "/robot0"  # photo
TB1_NAMESPACE = "/robot1"  # patrol

# pub/sub topic names
NAV0_PUB = TB0_NAMESPACE + "/goal_position"
NAV0_SUB = TB0_NAMESPACE + "/goal_result"
NAV1_PUB = TB1_NAMESPACE + "/goal_position"
NAV1_SUB = TB1_NAMESPACE + "/goal_result"
CV_SUB_DETECTED = TB1_NAMESPACE + "/accident_detected"
CV_SUB_POSITION = TB1_NAMESPACE + "/accident_position"
UI_ALARM = TB1_NAMESPACE + "/dispatch_command"
PHOTO_PUB = "/photo"
SERVER_SUB = "/control_scenario"


class Server(Node):
    def __init__(self):
        super().__init__("yolo_depth_to_map")

        # DOING FLAG
        self.set_status(STATUS.READY_FLAG)

        # MODULE STATE CHECK
        self.nav0_state = STATUS.NAV_DONE
        self.nav1_state = STATUS.NAV_DONE
        self.cv_state = 0  # 카메라 촬영 대기 -> 촬영 실행 중 -> 촬영 완료
        # 촬영 위치 수신 대기 -> 촬영 위치 수신 완료 -> 촬영 완료
        self.cv_position_state = 0
        self.dispatch_state = 0  # 복귀 위치 수신 -> 복귀 완료

        self.nav0_current_position = 0
        self.nav1_current_position = 0
        # ([-1.76, 3.77], TurtleBot4Directions.NORTH),    # 6
        self.nav1_accident_position = 1

        # TF 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.get_logger().info("tf 안정화(5초)")
        time.sleep(5)
        self.get_logger().info("tf 안정화 완료")

        # self.get_logger().info("[1/5] 노드 초기화 시작...")
        self.get_logger().info("[1/5] Initialize server node...")

        self.status: STATUS = STATUS.READY_FLAG  # initialize status

        # ======= Main loop ======= #
        self.create_timer(1.0, self.update_loop)  # 1초 주기로 update_loop 실행

        # ======= topics ======= #
        # nav0
        self.nav0_pub = self.create_publisher(i32, NAV0_PUB, qos_profile_10)
        self.nav0_pub2 = self.create_publisher(
            Point, NAV0_PUB + "2", qos_profile_10
        )  # yolo에서 탐지한 좌표로 이동
        self.nav0_sub = self.create_subscription(
            i32, NAV0_SUB, self._nav0_sub_callback, qos_profile_10
        )
        # nav1
        self.nav1_pub = self.create_publisher(i32, NAV1_PUB, qos_profile_10)
        self.nav1_sub = self.create_subscription(
            i32, NAV1_SUB, self._nav1_sub_callback, qos_profile_10
        )
        # cv
        self.cv_sub = self.create_subscription(
            i32, CV_SUB_DETECTED, self._cv_suv_detected_callback, qos_profile_10
        )
        self.cv_sub_point = self.create_subscription(
            Point, CV_SUB_POSITION, self._cv_suv_position_callback, qos_profile_10
        )

        # ui/alarm
        self.ui_alarm_pub = self.create_publisher(i32, UI_ALARM, qos_profile_10)

        # pcd
        self.pcd_pub = self.create_publisher(i32, PHOTO_PUB, qos_profile_10)

        # control scenario
        self.control_scenario_sub = self.create_subscription(
            i32, SERVER_SUB, self.control_scenario, qos_profile_10
        )

    def update_loop(self):
        # 대기 시 순찰 시작
        if self.status == STATUS.READY_FLAG:
<<<<<<< HEAD
            if self.nav1_current_position >= 4:  # 이동 종료 시
=======
            self.get_logger().info('ready')

            if self.nav1_current_position >= 5:  # 이동 종료 시
>>>>>>> wsh
                self.set_status(STATUS.EXIT_FLAG)
            else:
                self.set_status(STATUS.PATROL_FLAG)
                self.patrol()
        # 순찰 중
        elif self.status == STATUS.PATROL_FLAG:
            if self.nav1_state == STATUS.NAV_DONE:  # 순찰 포지션 이동 완료
                self.set_status(STATUS.READY_FLAG)
        # 사고 감지 시
        elif self.status == STATUS.DETECTED_FLAG:
            # 위치 이동 종료 시
            if (
                self.cv_position_state == 2  # 위치 감지
                and self.nav1_state == STATUS.NAV_DONE  # 1 도착
                and self.nav0_state == STATUS.NAV_DONE  # 0 도착
            ):
                self.get_logger().info("1")
                self.set_status(STATUS.DISPATCH_FLAG)

            # 순찰 정보 수신 완료 시
            elif self.cv_position_state == 1:
                self.get_logger().info("2")
                self.detected()  # 각 로봇 이동 명령
                self.cv_position_state = 2

<<<<<<< HEAD
=======
            # if self.cv_position_state == 2:
            #     self.get_logger().info("aaa")
            # else:
            #     self.get_logger().info("bbb")
            # if self.nav1_state == STATUS.NAV_DONE:
            #     self.get_logger().info("ccc")
            # else:
            #     self.get_logger().info("ddd")
            # if self.nav0_state == STATUS.NAV_DONE:
            #     self.get_logger().info("eee")
            # else:
            #     self.get_logger().info("fff")

>>>>>>> wsh
        # 출동 종료 명령 시
        elif self.status == STATUS.DISPATCH_FLAG:

            if (
                self.dispatch_state == 1 and self.nav0_state == STATUS.NAV_DONE
            ):  # TB0 복귀 포지션 이동 완료
                self._dock(0)
                self.set_status(STATUS.PATROL_FLAG)
            if self.dispatch_state == 0:
                self.dispatch_state = 1
                self.dispatch()
<<<<<<< HEAD
=======

            if self.nav1_state == STATUS.NAV_DONE:
                self.get_logger().info("aaa")
            else:
                self.get_logger().info("bbb")
            if self.nav0_state == STATUS.NAV_DONE:
                self.get_logger().info("ccc")
            else:
                self.get_logger().info("ddd")
>>>>>>> wsh
        # 시나리오 종료 시
        elif self.status == STATUS.EXIT_FLAG:
            self.exit_scenario()

<<<<<<< HEAD
        self.get_logger().info("ready")
=======
>>>>>>> wsh

    def get_status(self) -> STATUS:
        return self.status

    def set_status(self, new_status: STATUS):
        self.status = new_status

    def _make_msg(self, data):
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
        else:
            self.get_logger().info("all_stop")
            self.all_stop()

    def ready(self):
        self.set_status(STATUS.READY_FLAG)
        self.get_logger().info("[1/5] Ready to patrol...")

    def patrol(self):
        self.set_status(STATUS.PATROL_FLAG)
        self.get_logger().info("[2/5] Patrol...")

        # 이동
        self.nav1_state = STATUS.NAV_WORKING
        self.nav1_pub.publish(self._make_msg(self.nav1_current_position))
        # 완료
        self.nav1_pub.publish(self._make_msg(self.nav1_current_position))
<<<<<<< HEAD
        self.get_logger().info(f"patrol pub: {self.nav1_current_position}")
=======
        self.get_logger().info(f'patrol pub: {self.nav1_current_position}')
>>>>>>> wsh
        self.nav1_current_position += 1

    def detected(self):
        """
        ### 사고 감지

        #### TB1 교통 안내 위치로 이동
        #### TB1 교통 통제 시작
        #### TB0 촬영 위치로 이동
        """
        self.get_logger().info("[3/5] Detected...")

        # TB1 교통 안내 위치로 이동
        self.nav1_state = STATUS.NAV_WORKING
        self.nav1_pub.publish(self._make_msg(self.nav1_accident_position))
        self.nav1_current_position -= 1  # 순찰 미완료로 원래 위치 저장
        # TB1 교통 통제 시작
        self.ui_alarm_pub.publish(self._make_msg(0))  # alarm on

        # TB0 촬영 위치로 이동
        self.nav0_state = STATUS.NAV_WORKING
        point = Point()
        # point.x, point.y, point.z = [
        #     -1.67,
        #     1.54,
        #     0.0,
        # ]  # 7번 위치, 촬영 위치, 트랜스폼 값 넣어주기
        point.x, point.y, point.z = self.point
        self.nav0_pub2.publish(point)

    def dispatch(self):
        """
        ### 출동 종료 명령

        #### PCD 촬영
        #### TB0 복귀
        #### TB1 교통 통제 종료
        """

        self.get_logger().info("[4/5] Dispatch...")

        # PCD 촬영
        self.pcd_pub.publish(self._make_msg(0))  # 사진 촬영
        # self.cv_state = 0
        # self.cv_position_state = 0

        # TB0 복귀
        point = Point()
<<<<<<< HEAD
        point.x, point.y, point.z = [-1.61, -0.38, 0.0]  # 8번 위치, dock 하기 전 위치
=======
        point.x, point.y, point.z = [-0.55, -0.37, 0.0]  # 8번 위치, dock 하기 전 위치
>>>>>>> wsh
        self.nav0_state = STATUS.NAV_WORKING
        self.nav0_pub2.publish(point)

        # TB1 교통 통제 종료
        self.ui_alarm_pub.publish(self._make_msg(1))  # alarm off

    def _dock(self, TB_num):
        """
        ### 도킹 시작
        """
        if TB_num == 0:
            self.nav0_pub.publish(self._make_msg(-1))  # TB0 도킹
            self.get_logger().info("TB0 DOCKING...")
        elif TB_num == 1:
            self.nav1_pub.publish(self._make_msg(-1))  # TB0 도킹
            self.get_logger().info("TB1 DOCKING...")

    def exit_scenario(self):
        self._dock(1)
        self.set_status(STATUS.EXIT_FLAG)
        self.get_logger().info("[5/5] Exit Scenario...")

    def all_stop(self):
        pass

    def _nav0_sub_callback(self, msg):
        self.get_logger().info(f"_nav0_sub_callback: {msg.data}")
        if msg.data != -2:
            self.get_logger().info("???? WHY NOT WOKR?")
            self.nav0_state = STATUS.NAV_DONE  # 이동 완료

    def _nav1_sub_callback(self, msg):
        self.get_logger().info(f"_nav1_sub_callback: {msg.data}")
        if msg.data != -2:
            self.nav1_state = STATUS.NAV_DONE  # 이동 완료

    def _cv_suv_detected_callback(self, msg):
        if self.cv_state == 0:  # 최초 감지 1회만 동작
            self.set_status(STATUS.DETECTED_FLAG)  # 사고 감지 상태
            self.cv_state = 1
            self.get_logger().info(f"_cv_suv_detected_callback: Accident Detected!!!")

    def _cv_suv_position_callback(self, msg):
        self.cv_position_state = 1
        self.get_logger().info(f"_cv_suv_position_callback: {msg}")
        x, y, z = self.tf_cam2map(msg.x, msg.y, msg.z)
        self.get_logger().info(f"x, y, z : {x}, {y}, {z}")
        self.point = [x, y, z]  # ???

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
