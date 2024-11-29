import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import transforms3d
import math
from rclpy.executors import MultiThreadedExecutor

class SensorMeasurement(Node):
    def __init__(self):
        super().__init__('sensor_measurement')

        self.subscription = self.create_subscription(
            String,
            'sensor_measurement',
            self.decode_and_publish,
            10
        )
        self.qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )
        self.subscription = self.create_subscription(
            String,
            'sensor_measurement',
            self.decode_and_publish,
            self.qos
        )


        self.pose_publisher = self.create_publisher(PoseStamped, 'robot_pose', 10)
        self.path_publisher = self.create_publisher(Path, 'robot_path', 10)
        self.timer = self.create_timer(0.1, self.publish_path)


        self.path = Path()
        self.path.header.frame_id = "map"

    def decode_and_publish(self, msg):
        try:
            robotData = msg.data.split()
            if not robotData or len(robotData) < 8:
                self.get_logger().error("Invalid data format")
                return

            data_type = robotData[0]

            if data_type == "1":
                x = float(robotData[1])
                y = float(robotData[2])
                theta_deg = float(robotData[3])
                theta_rad = math.radians(theta_deg)

                # Create quaternion from yaw
                quaternion = transforms3d.euler.euler2quat(0, 0, theta_rad)

                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = "map"
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = 0.0
                pose_stamped.pose.orientation.w = quaternion[0]
                pose_stamped.pose.orientation.x = quaternion[1]
                pose_stamped.pose.orientation.y = quaternion[2]
                pose_stamped.pose.orientation.z = quaternion[3]

                                # Create quaternion from yaw
                quaternion = transforms3d.euler.euler2quat(0, 0, theta_rad)


                self.pose_publisher.publish(pose_stamped)

                # Update and limit path length
                self.path.header.stamp = pose_stamped.header.stamp
                self.path.poses.append(pose_stamped)
                if len(self.path.poses) > 100:  # Limit to 100 poses
                    self.path.poses.pop(0)
                #self.path_publisher.publish(self.path)

               # self.get_logger().info(f"Pose published: {x}, {y}, {theta_deg}. Path length: {len(self.path.poses)}")
            else:
                self.get_logger().warning(f"Unknown data type: {data_type}")

        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Failed to decode data: {msg.data}, error: {e}")
    
    def publish_path(self):
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


