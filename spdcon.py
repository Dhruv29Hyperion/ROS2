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
        self.linear_speed = 1.0
        self.angular_speed = 1.0

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
        print("Use 'w' to increase speed, 's' to decrease speed, 'a' to turn left, 'd' to turn right,")
        print("'i' to move forward, 'k' to move backward, 'j' to turn left, 'l' to turn right, and 'q' to quit.")
        while True:
            key = self.get_key()
            print(f"Key Pressed: {key}")
            if key == 'w':
                self.linear_speed += 0.1
                print(f"Linear Speed Increased: {self.linear_speed:.2f}")
            elif key == 's':
                self.linear_speed -= 0.1
                if self.linear_speed < 0:
                    self.linear_speed = 0
                print(f"Linear Speed Decreased: {self.linear_speed:.2f}")
            elif key == 'a':
                self.angular_speed += 0.1
                print(f"Angular Speed Increased: {self.angular_speed:.2f}")
            elif key == 'd':
                self.angular_speed -= 0.1
                if self.angular_speed < 0:
                    self.angular_speed = 0
                print(f"Angular Speed Decreased: {self.angular_speed:.2f}")
            elif key == 'i':
                self.twist.linear.x = self.linear_speed
                self.twist.angular.z = 0.0
                self.publisher.publish(self.twist)
            elif key == 'k':
                self.twist.linear.x = -self.linear_speed
                self.twist.angular.z = 0.0
                self.publisher.publish(self.twist)
            elif key == 'j':
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.angular_speed
                self.publisher.publish(self.twist)
            elif key == 'l':
                self.twist.linear.x = 0.0
                self.twist.angular.z = -self.angular_speed
                self.publisher.publish(self.twist)
            elif key == 'q':
                break
            else:
                self.twist.linear.x = 0.0
                self.twist.angular.z = 0.0
                self.publisher.publish(self.twist)
            print(f"Current Speed: Linear - {self.linear_speed:.2f}, Angular - {self.angular_speed:.2f}")

def main(args=None):
    rclpy.init(args=args)
    control_node = TurtleControl()
    control_node.control_turtle()
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

