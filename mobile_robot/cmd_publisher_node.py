import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommandPublisherNode(Node):
    def __init__(self):
        super().__init__('command_publisher_node')
        self.publisher_ = self.create_publisher(String, '/cmd', 10)
        self.get_logger().info("Command Publisher Node started. Waiting for input...")

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisherNode()

    try:
        while rclpy.ok():
            command = input("Enter command: ")
            node.publish_command(command)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Command Publisher Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


