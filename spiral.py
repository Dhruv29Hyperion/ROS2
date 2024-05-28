#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import sys, termios, tty, os, time

class SpiralMotion(Node):
    def __init__(self):
        super().__init__('spiral_motion')
        self.velocity_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.timer = self.create_timer(1.0, self.publish_velocity)
        self.radius = 1.0
        self.pose_x = 0.0
        self.pose_y = 0.0

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def pose_callback(self, pose_msg):
        pass

    def publish_velocity(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.radius
        
        vel_msg.angular.z += 2.0

        self.velocity_publisher_.publish(vel_msg)

def main(args=None):
    print("Press 's' for spiral and 'q' to stop")
    rclpy.init(args=args)
    spiral_motion = SpiralMotion()
    rclpy.spin(spiral_motion)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
