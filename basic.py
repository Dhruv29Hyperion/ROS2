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
        print("Use 'i' to move forward, 'k' to move backward, 'j' to turn left, 'l' to turn right, and 'q' to quit.")
        while True:
            key = self.get_key()
            if key == 'i':
                self.twist.linear.x = 1.0
                self.twist.angular.z = 0.0
            elif key == 'k':
                self.twist.linear.x = -1.0
                self.twist.angular.z = 0.0
            elif key == 'j':
                self.twist.linear.x = 0.0
                self.twist.angular.z = 1.0
            elif key == 'l':
                self.twist.linear.x = 0.0
                self.twist.angular.z = -1.0
            elif key == 'q':
                break
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
            self.publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    control_node = TurtleControl()
    control_node.control_turtle()
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
