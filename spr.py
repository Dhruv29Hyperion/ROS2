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
        self.radius = 1.0
        self.spiraling = False

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
        if self.spiraling:
            vel_msg = Twist()
            vel_msg.linear.x = self.radius
            vel_msg.angular.z = 2.0
            self.radius += 0.5
            self.velocity_publisher_.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    spiral_motion = SpiralMotion()
    key_pressed = False

    while rclpy.ok():
        key = spiral_motion.get_key()
        if key == 'o':
           spiral_motion.spiraling = True
           key_pressed = True
        elif key == 'x':
            spiral_motion.spiraling = False
            key_pressed = False
        else:
            rclpy.spin_once(spiral_motion)
            spiral_motion.publish_velocity()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

