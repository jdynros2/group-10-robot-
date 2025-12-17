#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Image
import time

class AutoNavigateToFace(Node):
    def __init__(self):
        super().__init__('auto_navigate_to_face')
        
        # Publishers
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
        
        # Wait 12 seconds for Nav2 to start
        time.sleep(12)
        
        # Set initial pose at robot spawn location
        self.set_initial_pose()
        
        # Wait for localization
        time.sleep(8)
        
        # Send goal to face location
        self.send_goal_to_face()
        
    def set_initial_pose(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = 4.0  # Robot spawn x
        pose.pose.pose.position.y = 0.0  # Robot spawn y
        pose.pose.pose.orientation.z = 1.0  # 180 degree rotation
        pose.pose.pose.orientation.w = 0.0  # 180 degree rotation
        
        self.initial_pose_pub.publish(pose)
        self.get_logger().info('Initial pose set at (4.0, 0.0) facing 180°')
        
    def send_goal_to_face(self):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = -4.0  # Actor x position
        goal.pose.position.y = 5.0   # Actor y position
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        self.get_logger().info('Goal sent to face at (-4.0, 5.0)')

def main():
    rclpy.init()
    node = AutoNavigateToFace()
    rclpy.spin(node)

if __name__ == '__main__':
    main()