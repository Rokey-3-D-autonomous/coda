#!/usr/bin/env python3

import rclpy

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info("Docking before intialising pose")
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped(
        [-0.182, -0.314], TurtleBot4Directions.NORTH
    )
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = []

    goal_pose.append(
        navigator.getPoseStamped([-1.98, -0.00], TurtleBot4Directions.NORTH)
    )
    goal_pose.append(
        navigator.getPoseStamped([-2.01, -2.02], TurtleBot4Directions.EAST)
    )
    goal_pose.append(
        navigator.getPoseStamped([-0.79, -2.39], TurtleBot4Directions.SOUTH)
    )
    # goal_pose.append(navigator.getPoseStamped([-1.1, -0.3], TurtleBot4Directions.WEST)) # 실패 좌표 loop closure됨ㅇ
    # goal_pose.append(navigator.getPoseStamped([9.0, 1.0], TurtleBot4Directions.SOUTH))
    # goal_pose.append(navigator.getPoseStamped([-1.0, 1.0], TurtleBot4Directions.EAST))

    # Undock
    navigator.undock()

    # Navigate through poses
    navigator.startThroughPoses(goal_pose)

    # Finished navigating, dock
    navigator.dock()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

# https://rainy-cream-352.notion.site/Robot_TB4_nav_through_pose-1f4450ef7c59819e95c0f4d9ce8e2a12
# 코드 제목은 없음
