from enum import Enum

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32 as i32
from geometry_msgs.msg import PoseStamped


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
CV_SUB = "/accident_detected"
UI_ALARM = TB1_NAMESPACE + "/dispatch_command"
PHOTO_PUB = "/photo"


class Server(Node):
    def __init__(self):
        super().__init__("yolo_depth_to_map")

        self.nav0_current_position = 0
        self.nav1_current_position = 0
        self.nav1_accident_position = 7  # 수정 필요

        # self.get_logger().info("[1/5] 노드 초기화 시작...")
        self.get_logger().info("[1/5] Initialize server node...")

        self.status: STATUS = STATUS.READY_FLAG  # initialize status

        # ======= topics ======= #
        # nav0
        self.nav0_pub = self.create_publisher(i32, NAV0_PUB, 10)
        self.nav0_sub = self.create_subscription(
            i32, NAV0_SUB, self._nav0_sub_callback, 10
        )
        # nav1
        self.nav1_pub = self.create_publisher(i32, NAV1_PUB, 10)
        self.nav1_sub = self.create_subscription(
            i32, NAV1_SUB, self._nav1_sub_callback, 10
        )
        # cv
        self.cv_sub = self.create_subscription(i32, CV_SUB, self._cv_suv_callback, 10)
        # self.cv_sub = self.create_subscription(?, CV_SUB, 10)
        # ui/alarm
        self.ui_alarm_pub = self.create_publisher(i32, UI_ALARM, 10)
        # pcd
        self.pcd_pub = self.create_publisher(i32, PHOTO_PUB, 10)

        # control scenario
        self.control_scenario_sub = self.create_subscription(
            i32, "/control_scenario", self.control_scenario, 10
        )

    def get_status(self) -> STATUS:
        return self.status

    def set_status(self, new_status: STATUS):
        self.status = new_status

    def control_scenario(self, msg):

        if msg.data == 0:
            self.ready()
        elif msg.data == 1:
            self.patrol()
        elif msg.data == 2:
            self.detected()
        elif msg.data == 3:
            self.dispatch()
        elif msg.data == 4:
            self.exit_scenario()
        else:
            self.all_stop()
        pass

    def ready(self):
        self.set_status(STATUS.READY_FLAG)
        self.get_logger().info("[1/5] Ready to patrol...")

    def patrol(self):
        self.set_status(STATUS.PATROL_FLAG)
        self.get_logger().info("[2/5] Patrol...")
        # send goal position to nav0
        self.nav1_pub.publish(i32(self.nav1_current_position))
        self.nav1_current_position += 1

    def detected(self):
        self.set_status(STATUS.DETECTED_FLAG)
        self.get_logger().info("[3/5] Detected...")
        # send goal position to nav1
        self.ui_alarm_pub.publish(i32(1))  # alarm on

        self.nav0_pub.publish(i32(1))  # 촬영 위치로 가라
        self.nav1_pub.publish(i32(self.nav1_accident_position))  # 안내 위치로 가라
        self.nav1_current_position -= 1  # 순찰 미완료로 원래 위치 저장

    def dispatch(self):
        self.set_status(STATUS.DISPATCH_FLAG)
        self.get_logger().info("[4/5] Dispatch...")

        self.pcd_pub.publish(i32(0))
        self.nav0_pub.publish(i32(0))  # 복귀해라

    def exit_scenario(self):
        self.set_status(STATUS.EXIT_FLAG)
        self.get_logger().info("[5/5] Exit...")
        # send goal position to nav0
        self.nav0_pub.publish(i32(0))
        self.nav1_pub.publish(i32(0))

        # alarm off
        self.ui_alarm_pub.publish(i32(0))

    def all_stop(self):
        pass

    def _nav0_sub_callback(self, msg):
        pass

    def _nav1_sub_callback(self, msg):

        pass

    def _cv_suv_callback(self, msg):
        # self.set_status(STATUS.DETECTED_FLAG)
        pass


def main():
    