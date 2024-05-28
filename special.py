import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class StarMotion(Node):
    def __init__(self):
        super().__init__('star_motion')
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_velocity)
        self.angle = 144  # Angle to create a star shape
        self.side_length = 4.0

    def publish_velocity(self):
        vel_msg = Twist()

        for _ in range(5):
            # Move forward (draw a side of the star)
            vel_msg.angular.z = 0.0
            vel_msg.linear.x = self.side_length
            self.velocity_publisher.publish(vel_msg)
            self.get_logger().info('Moving forward')
            self.spin_once(0.5)  # Wait for half a second

            # Turn right to create the star shape
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 2.6  # Set an appropriate angular velocity to create the angle
            self.velocity_publisher.publish(vel_msg)
            self.get_logger().info('Turning')
            self.spin_once(2)  # Wait for 2 seconds


    def spin_once(self, duration):
        rclpy.spin_once(self, timeout_sec=duration)

def main(args=None):
    rclpy.init(args=args)
    star_motion = StarMotion()
    rclpy.spin(star_motion)
    star_motion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

