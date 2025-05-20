from setuptools import find_packages, setup

package_name = "coda"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lhj",
    maintainer_email="hojun7889@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pub_test_detection_node = coda.pub_test_detection_node:main",
            "pub_test_transform_node = coda.pub_test_transform_node:main",
            "server_test = coda.server_test:main",
            "server1_test = coda.server1_test:main",
            "server1 = coda.server1:main",
            "server2_node = coda.server2_node:main",
            "turtlebot1_node = coda.turtlebot1_node:main",
            "turtlebot2_node = coda.turtlebot2_node:main",
            "wsh_nav2_controller = coda.wsh_nav2_controller:main",
            "wsh_server1 = coda.wsh_server1:main",
            "wsh_yolo_depth_rviz = coda.wsh_yolo_depth_rviz:main",
            "part_transform = coda.server1.part_transform:main",   #여기부터 파일없음
            "part_detect = coda.server1.part_detect:main",
            "part_patrol = coda.part_patrol:main",
            "part_transform = coda.server1.part_transform:main",
            "server1_test = coda.server1.server1_test:main",
            "server1 = coda.server1.server1:main",
            
            # for test
            "server_test = coda.server_test:main",
            "wsh_server = coda.wsh_server1:main",
            "cv = coda.3_4_tb4_yolo_bbox_depth_checker_obj_11am:main",
            "pcd = coda.server1.pcd_to_html:main",
            "wsh_nav2_controller1 = coda.wsh_nav2_controller:main",
            "wsh_nav2_controller0 = coda.wsh_nav2_controller0_camera:main",
        ],
    },
)
