import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import transforms3d
import math

class SensorMeasurement(Node):
    def __init__(self):
        super().__init__('sensor_measurement')

        # Common QoS profiles
        self.qos_best_effort = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )
        self.qos_reliable = rclpy.qos.QoSProfile(
            depth=100,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )

        self.qos_profile = rclpy.qos.QoSProfile(
                depth=10,
                reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL
            )

        # Subscription to sensor_measurement
        self.subscription = self.create_subscription(
            String,
            'sensor_measurement',
            self.decode_and_store,
            self.qos_best_effort
        )

        # Publishers for pose and path
        self.pose_publisher = self.create_publisher(PoseStamped, 'robot_pose', self.qos_reliable)
        self.path_publisher = self.create_publisher(Path, 'robot_path', self.qos_profile)

        # Timers for periodic publishing
        self.pose_timer = self.create_timer(0.01, self.publish_pose)  # 100 Hz
        self.path_timer = self.create_timer(0.1, self.publish_path)  # 10 Hz

        # Initialize pose and path
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "map"
        self.path = Path()
        self.path.header.frame_id = "map"
        self.max_path_length = 100

    def decode_and_store(self, msg):
        """Decode incoming message and store the current pose."""
        try:
            robot_data = msg.data.split()
            if len(robot_data) < 4:
                self.get_logger().error("Invalid data format")
                return

            # Parse incoming data
            data_type = robot_data[0]
            if data_type == "1":  # Pose data
                x = float(robot_data[1])
                y = float(robot_data[2])
                theta_deg = float(robot_data[3])
                theta_rad = math.radians(theta_deg)
                
                # Update current pose
                quaternion = transforms3d.euler.euler2quat(0, 0, theta_rad)

                self.current_pose.pose.position.x = x
                self.current_pose.pose.position.y = y
                self.current_pose.pose.position.z = 0.0
                self.current_pose.pose.orientation.w = quaternion[0]
                self.current_pose.pose.orientation.x = quaternion[1]
                self.current_pose.pose.orientation.y = quaternion[2]
                self.current_pose.pose.orientation.z = quaternion[3]

                self.current_pose.header.stamp = self.get_clock().now().to_msg()
                self.path.header.stamp = self.current_pose.header.stamp
                self.path.poses.append(self.current_pose)

                # Limit the path size to avoid excessive memory usage
                if len(self.path.poses) > self.max_path_length:
                    self.path.poses.pop(0)

            else:
                self.get_logger().warning(f"Unknown data type: {data_type}")

        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Failed to decode data: {msg.data}, error: {e}")

    def publish_pose(self):
        """Publish the current pose."""
        if self.current_pose.header.stamp.sec > 0:  # Ensure pose has been initialized
            self.pose_publisher.publish(self.current_pose)

    def publish_path(self):
        """Publish the current path."""
        if self.path.poses:
            self.path_publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = SensorMeasurement()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



