import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class PathPlotterNode(Node):
    def __init__(self):
        super().__init__('path_plotter')
        self.get_logger().info("Path plotter node started")

        # Subscribe to PoseStamped topic
        self.subscription = self.create_subscription(
            PoseStamped,
            'robot_pose',  # Topic name
            self.pose_callback,
            10
        )

        # Initialize data for plotting
        self.x_data = []
        self.y_data = []

        # Initialize the Matplotlib plot
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-', label='Path')  # Path line
        self.ax.set_title('Robot Path')
        self.ax.set_xlabel('X Position')
        self.ax.set_ylabel('Y Position')
        self.ax.legend()

        # Start the animation
        self.anim = FuncAnimation(self.fig, self.update_plot, interval=500)

    def pose_callback(self, msg):
        """Callback to handle incoming PoseStamped messages."""
        # Extract x and y from the Pose
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Append to data lists
        self.x_data.append(x)
        self.y_data.append(y)

        # Log the received data
        self.get_logger().info(f"Pose received: x={x}, y={y}")

    def update_plot(self, frame):
        """Update the plot with new data."""
        if len(self.x_data) > 0 and len(self.y_data) > 0:
            self.line.set_data(self.x_data, self.y_data)
            self.ax.relim()  # Recalculate limits
            self.ax.autoscale_view()  # Adjust the view to fit new data
        return self.line,

def main(args=None):
    rclpy.init(args=args)
    node = PathPlotterNode()

    try:
        # Use a timer to integrate ROS spinning with Matplotlib
        def ros_spin_once():
            rclpy.spin_once(node, timeout_sec=0.1)
            plt.pause(0.01)
        while rclpy.ok():
            ros_spin_once()

    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Path Plotter Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
