#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_nav')

        # Read waypoints from YAML file
        with open('/home/amrutha/ros2_ws/src/Intelligent-Agricultural-Robot/simulation/navigation/waypoints.yaml', 'r') as f:
            self.waypoints = yaml.safe_load(f)['waypoints']

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waypoint Navigator Started")

    def send_goal(self, pose_dict):
        goal_msg = NavigateToPose.Goal()
        goal = PoseStamped()

        goal.header.frame_id = 'map'
        goal.pose.position.x = pose_dict['x']
        goal.pose.position.y = pose_dict['y']
        goal.pose.orientation.z = pose_dict['z']
        goal.pose.orientation.w = pose_dict['w']

        goal_msg.pose = goal

        self.client.wait_for_server()
        self.get_logger().info(f"Navigating to waypoint: {pose_dict}")

        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Reached waypoint.")

    def run(self):
        for wp in self.waypoints:
            self.send_goal(wp)
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

