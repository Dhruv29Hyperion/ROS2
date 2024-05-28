import rclpy
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('user_input_publisher')
    # Create a publisher on the /ug26matters topic
    publisher = node.create_publisher(String, '/ug26matters', 10)
    while rclpy.ok():
        user_input = input("Enter a message: ")
        msg = String()
        msg.data = user_input
        # Publish the user input
        publisher.publish(msg)
    
    node.get_logger().info(f"Published: {msg.data}")
    node.destroy_node()

if __name__ == '__main__':
    main()
