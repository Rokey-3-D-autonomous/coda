# 바라보는 위치에서 3D image 저장
# 저장경로 : ~/pcd_files
# 파일명   : rgbd_pointcloud_YYYYMMDD_HHMMSS.pcd
# pcd 파일 간단하게 보기 위해서 pcl_viewer 필요
# pcl_viewer 실행 : pcl_viewer [file_path + file_name]

# pcl_viewer 설치 : sudo apt update
#                  sudo apt install pcl-tools

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import open3d as o3d
import datetime
import os
import message_filters
import sys  # <- 터미널 반환을 위한 추가

ROBOT_NAMESPACE = 'robot0'
RGB_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/rgb/preview/image_raw'
DEPTH_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/stereo/image_raw'
CAMERA_INFO_TOPIC = f'/{ROBOT_NAMESPACE}/oakd/stereo/camera_info'

class RGBDToPCDConverter(Node):
    def __init__(self):
        super().__init__('rgbd_to_pcd_converter')
        self.bridge = CvBridge()
        self.saved = False

        rgb_sub = message_filters.Subscriber(self, Image, RGB_TOPIC)
        depth_sub = message_filters.Subscriber(self, Image, DEPTH_TOPIC)
        info_sub = message_filters.Subscriber(self, CameraInfo, CAMERA_INFO_TOPIC)

        ts = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.callback)

        self.save_dir = os.path.expanduser('~/pcd_files')
        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info('RGBD to PCD Converter is running.')

    def callback(self, rgb_msg, depth_msg, cam_info_msg):
        if self.saved:
            return

        self.get_logger().info('Received synced RGB-D and camera info.')

        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        if depth_image.dtype == np.uint16:
            depth_image = depth_image.astype(np.float32) / 1000.0  # mm → m

        fx = cam_info_msg.k[0]
        fy = cam_info_msg.k[4]
        cx = cam_info_msg.k[2]
        cy = cam_info_msg.k[5]

        height, width = depth_image.shape
        points = []
        colors = []

        for v in range(height):
            for u in range(width):
                z = depth_image[v, u]
                if z == 0 or np.isnan(z) or z > 10.0:
                    continue
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append([x, y, z])
                b, g, r = rgb_image[v, u]
                colors.append([r / 255.0, g / 255.0, b / 255.0])

        if not points:
            self.get_logger().warn('No valid points found.')
            return

        points_np = np.array(points, dtype=np.float32)
        colors_np = np.array(colors, dtype=np.float32)

        # 회전
        rotation_matrix = np.array([
            [ 1,  0,  0],
            [ 0,  -1,  0],
            [ 0,  0,  -1]
        ], dtype=np.float32)
        rotated_points = (rotation_matrix @ points_np.T).T

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(rotated_points)
        pcd.colors = o3d.utility.Vector3dVector(colors_np)

        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'rgbd_pointcloud_{timestamp}.pcd'
        filepath = os.path.join(self.save_dir, filename)

        o3d.io.write_point_cloud(filepath, pcd)
        self.get_logger().info(f'Saved rotated RGB-D point cloud to {filepath}')

        self.saved = True
        self.get_logger().info('Shutting down...')
        # shutdown + 종료 예약
        self.executor = rclpy.get_global_executor()
        self.executor.call_soon_threadsafe(self._shutdown)

    def _shutdown(self):
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    converter = RGBDToPCDConverter()

    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        converter.get_logger().info('Converter stopped by user.')
    finally:
        if rclpy.ok():
            converter.destroy_node()
            rclpy.shutdown()
        sys.exit(0)  # <- 최종 fallback 종료

if __name__ == '__main__':
    main()