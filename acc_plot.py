import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import numpy as np

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
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')

    def lidar_callback(self, msg):
        # Lidar data processing and plotting
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        # Convert polar coordinates to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # Plot the Lidar data as a scatter plot
        self.ax.clear()
        self.ax.scatter(x, y, label='Lidar Data')
        self.ax.set_title('Lidar Data')
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
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


