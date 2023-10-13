#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

import random

class MoveToRandomPointsNode(Node):
    def __init__(self):
        super().__init__('move_to_random_points')

        self.target_x = random.randint(-5,5)
        self.target_y = random.randint(-5,5)
        self.get_logger().info(f"Punto objetivo: ({self.target_x}, {self.target_y})")

        self.pose = None
        self.q = None

        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.callback_odometry, 10)
        self.control_loop_timer = self.create_timer(0.01, self.control_loop)
    
    def callback_odometry(self, msg):
        self.pose = msg.pose.pose

    def control_loop(self):
        if self.pose == None:
            return
        
        dist_x = self.target_x - self.pose.position.x
        dist_y = self.target_y - self.pose.position.y
        distance = math.sqrt(dist_x**2 + dist_y**2)

        msg = Twist()

        #atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy^2 + qz^2))

        if distance > 0.3:
            # Position
            msg.linear.x = min(max(distance, 0.3), 0.8)

            # Orientation
            q = self.pose.orientation

            current_theta = math.atan2(
                2*(q.w * q.z + q.x * q.y),
                1 - 2 * (q.y**2 + q.z**2)
            )

            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - current_theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi

            msg.angular.z = min(max(diff * 2, -3.0), 3.0)
        else:
            self.target_x = random.randint(-5,5)
            self.target_y = random.randint(-5,5)
            self.get_logger().info(f"Punto objetivo: ({self.target_x}, {self.target_y})")


        self.cmd_vel_publisher.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    node = MoveToRandomPointsNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()