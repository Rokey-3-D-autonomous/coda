import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from coda_interfaces.srv import VehicleControl

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
import datetime
import os
import message_filters

# nav
from nav2_simple_commander.robot_navigator import BasicNavigator
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import tf2_geometry_msgs  
import time

CAPTURE_DISTANCE = 1.0 * 1.414
WAITING_FOR_IMAGE = 1.0


# ==========출동 정보 기반 촬영 위치 계산=====================
class CamPoseNode(Node):
    def __init__(self):
        super().__init__('campose_node')
        self.srv = self.create_service(
            VehicleControl,'이름',self.calc_pose_callback)
        self.cam_poses = []
        
    def calc_pose_callback(self,request,response):
        accident_pose = request.location

        acc_x = accident_pose.pose.position.x
        acc_y = accident_pose.pose.position.y
        
        cap_1 = (acc_x - CAPTURE_DISTANCE, acc_y + CAPTURE_DISTANCE, -45)
        cap_2 = (acc_x + CAPTURE_DISTANCE, acc_y + CAPTURE_DISTANCE, -135)
        cap_3 = (acc_x + CAPTURE_DISTANCE, acc_y - CAPTURE_DISTANCE, 135)
        cap_4 = (acc_x - CAPTURE_DISTANCE, acc_y - CAPTURE_DISTANCE, 45)

        self.cam_poses.extend([cap_1, cap_2, cap_3, cap_4])

        return response

# ==========촬영 명령=====================
class CheeseNode(Node):
    def __init__(self):
        super().__init__('cheese_ndoe')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.dock_navigator = TurtleBot4Navigator()
        self.nav_navigator = BasicNavigator(node_name='navigator_robot2')

        self.amclposesub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot0/amcl_pose',
            self.amcl_callback, 10)
        

        
    def amcl_callback(self,msg):
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        self.current_yaw = euler_from_quaternion(self.current_pose.pose.orientation, 'z')[2] * (180 / 3.141592)
        
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
    
    def get_init_pose(self) -> None:
        self.initial_pose = self.create_pose(-0.01, -0.01, 0.0, self.nav_navigator)
        self.nav_navigator.setInitialPose(self.initial_pose)
        self.nav_navigator.get_logger().info(f'초기 위치 설정 중...')
        time.sleep(1.0) #AMCL이 초기 pose 처리 시 필요한 시간과 TF를 얻을 수 있게 됨
    
        self.nav_navigator.waitUntilNav2Active()

    def create_waypoints(self, cam_poses):
        self.cam_poses = cam_poses
        self.waypoints = []
        for pose in self.cam_poses:
            self.waypoints.append(self.create_pose(pose))

    def undock(self) -> None:
        if self.dock_navigator.getDockedStatus():
            self.dock_navigator.get_logger().info('현재 도킹 상태 → 언도킹 시도')
            self.dock_navigator.undock()
            
            self.dock_pose = self.get_current_pose()
        else:
            self.dock_navigator.get_logger().info('언도킹 상태에서 시작')

    def move(self):
        self.nav_start = self.nav_navigator.get_clock().now()
        self.nav_navigator.followWaypoints(self.waypoints)

    def get_current_pose(self):
        return (self.current_pose.pose.position.x, self.current_pose.pose.position.y , self.current_yaw)

    def get_feedback(self) -> None:
        while not self.nav_navigator.isTaskComplete():
            feedback = self.nav_navigator.getFeedback()
            if feedback                                                                                                                                                                                      :
                elapsed = self.nav_navigator.get_clock().now() - self.nav_start
                self.nav_navigator.get_logger().info(
                    f'현재 waypoint: {feedback.current_waypoint + 1}/{len(self.waypoints)},'

                    f'경과 시간: {elapsed.nanoseconds / 1e9:.1f}초'
                )

                self.nav_navigator.get_logger().log('확인필요')
                
                for cam_pose in self.cam_poses:
                    if cam_pose == self.get_current_pose():
                        time.sleep(WAITING_FOR_IMAGE)

                        
    def gohome(self):
        self.nav_navigator.get_logger().info('도킹 위치로 복귀합니다.')
        self.nav_navigator.goToPose(self.dock_pose)
        self.nav_navigator.get_logger().info('도킹 위치로 복귀 완료')

        
    def dock(self) -> None:
        self.dock_navigator.get_logger().info('경로 완료. 도킹 시도...')
        self.dock_navigator.dock()
        self.dock_navigator.get_logger().info('도킹 요청 완료')

    def terminate(self) -> None:
        self.dock_navigator.destroy_node()
        self.nav_navigator.destroy_node()


# ==========복귀 명령=====================
class CompleteNode(Node):
    def __init__(self):
        super().__init__('complete_node')
        self.cli = self.create_client(VehicleControl, '이름')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = VehicleControl.Request()

    def send_return(self, location):
        self.req.location = location
        future = self.cli.call_async(self.req)
        return future


# ==========main=====================
def main():
    rclpy.init()

    campose_node = CamPoseNode()
    cam_poses = campose_node.cam_poses


    cheese_node = CheeseNode()
    cheese_node.get_init_pose()
    cheese_node.undock()
    dock_pose = cheese_node.dock_pose
    cheese_node.create_waypoints(cam_poses)
    cheese_node.move()
    cheese_node.get_feedback()

    complete_node = CompleteNode()
    future = complete_node.send_return(dock_pose) # location => 처음 undock 했을 때 좌표
    rclpy.spin_until_future_complete(complete_node,future)
    cheese_node.gohome()


