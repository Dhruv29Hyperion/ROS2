#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty, os, time

class TurtleControl(Node):
    def __init__(self):
        super().__init__('turtlesim_control')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.twist = Twist()
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

    def control_turtle(self):
        print("Use 'a' to move anticlockwise and 'c' to move clockwise.")
        while True:
            key = self.get_key()
            if key == 'a':
                self.twist.angular.z = 1.0
                self.twist.linear.x = self.radius * self.twist.angular.z
                for _ in range(100):
                    self.publisher.publish(self.twist)
            elif key == 'c':
                self.twist.angular.z = -1.0
                self.twist.linear.x = self.radius * self.twist.angular.z
                for _ in range(100):
                    self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    control_node = TurtleControl()
    control_node.control_turtle()
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
