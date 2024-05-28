import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

class LidarPlotterNode(Node):
    def __init__(self):
        super().__init__('lidar_plotter_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10  # Adjust the queue size as needed
        )

        # Plotting setup
        plt.ion()  # Turn on interactive mode for real-time plotting
        self.fig, self.ax = plt.subplots()
        self.ax.set_title('Lidar Data')
        self.ax.set_xlabel('Angle (radians)')
        self.ax.set_ylabel('Range (meters)')

    def lidar_callback(self, msg):
        # Lidar data processing and plotting
        angles = [msg.angle_min + i * msg.angle_increment for i in range(len(msg.ranges))]
        ranges = msg.ranges

        # Plot the Lidar data
        self.ax.clear()
        self.ax.plot(angles, ranges, label='Lidar Data')
        self.ax.set_title('Lidar Data')
        self.ax.set_xlabel('Angle (radians)')
        self.ax.set_ylabel('Range (meters)')
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    lidar_plotter_node = LidarPlotterNode()
    rclpy.spin(lidar_plotter_node)
    lidar_plotter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
