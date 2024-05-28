import rclpy
from std_msgs.msg import String
import sys, select, termios, tty

# IM LAB TEST P2

# Function to get a single key press from the user
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    
    # Create a ROS node
    node = rclpy.create_node('name')
    
    # Create a ROS publisher for the String message type
    pub = node.create_publisher(String, '/keyboard', 10)

    while True:
        # Get a single key press from the user
        key = get_key()

        # Check if the key pressed is 's', print the name
        while key == 's':
           print("Dhruv Srivastava")
           key = get_key()
           if key == 'h':
           	print("halted")
           else:
           	key == 's'

    # Cleanup: destroy the ROS node and shutdown the ROS client library
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

