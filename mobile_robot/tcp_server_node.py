import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import struct
import time
from geometry_msgs.msg import Pose,PoseStamped,Pose2D
from nav_msgs.msg import Path
import transforms3d
import math

class TcpServerNode(Node):
    def __init__(self):
        super().__init__('tcp_server_node')

        self.publisher_ = self.create_publisher(String, 'sensor_measurement', 10)
        self.subscriber_ = self.create_subscription(String, "/cmd", self.send_data_to_client, 10)
        
        self.server_socket = None
        self.client_socket = None
        self.client_addr = None
        self.server_host = '0.0.0.0'
        self.server_port = 8081
        self.lock = threading.Lock()
        self.path = Path()
        self.path.header.frame_id = "map"

        # Start the server and the thread for receiving data
        self.start_tcp_server()
        self.receive_data_thread = threading.Thread(target=self.receive_data_from_client, daemon=True)
        self.receive_data_thread.start()

    def start_tcp_server(self):
        """Initialize the server socket and wait for a client connection."""
        try:
            if self.server_socket:
                self.server_socket.close()
                
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.server_host, self.server_port))
            self.server_socket.listen()
            self.get_logger().info(f"Listening for TCP connections on port {self.server_port}...")
        except Exception as e:
            self.get_logger().error(f"Failed to start server: {e}")
            self.server_socket = None  # Reset to ensure it tries to reinitialize

    def accept_client_connection(self):
        """Accept a new client connection."""
        while rclpy.ok():
            if not self.server_socket:
                # Attempt to reinitialize the server socket if it's not set
                self.get_logger().info("Reinitializing server socket...")
                self.start_tcp_server()
                time.sleep(1)
                continue

            try:
                self.client_socket, self.client_addr = self.server_socket.accept()
                self.client_socket.setblocking(False)
                self.get_logger().info(f"Accepted connection from {self.client_addr}")
                break  # Exit loop once connected
            except BlockingIOError:
                time.sleep(1)  # Wait before trying to accept again
            except Exception as e:
                self.get_logger().error(f"Error accepting connection: {e}")
                time.sleep(1)

    def receive_data_from_client(self):
        """Continuously receive data from the client and handle reconnections."""
        while rclpy.ok():
            if not self.client_socket:
                self.get_logger().info("Waiting for a client to connect...")
                self.accept_client_connection()

            try:
                client_data = self.client_socket.recv(1024)
                if client_data:
                    msg = String()
                    msg.data = client_data.decode('utf-8').strip()
                    self.publisher_.publish(msg)
                    self.get_logger().info("data published")
            except BlockingIOError:
                continue
            except Exception as e:
                self.get_logger().error(f"Error receiving data: {e}")
                self.close_client_socket()
        time.sleep(0.1)
    
    def close_client_socket(self):
        """Close the client socket and reset for reconnection."""
        if self.client_socket:
            try:
                self.client_socket.close()
            except Exception as e:
                self.get_logger().error(f"Error closing client socket: {e}")
            finally:
                self.client_socket = None

    def send_data_to_client(self, msg):
        """Send data to the client over TCP."""
        with self.lock:
            if self.client_socket:
                try:
                    # Parse the command string from the message
                    command_parts = msg.data.split()
                    data_id = int(command_parts[0])
                    data_values = [float(val) if '.' in val else int(val) for val in command_parts[1:]]
                    
                    # Define the struct format and pack data
                    byte_format = f'!i{"f" * len(data_values)}'
                    byte_data = struct.pack(byte_format, data_id, *data_values)

                    # Send the packed byte data
                    self.client_socket.sendall(byte_data)
                    self.get_logger().info(f"Sent to client: data_id={data_id}, values={data_values}")
                except Exception as e:
                    self.get_logger().error(f"Failed to send data to client: {e}")
                    self.close_client_socket()

def main(args=None):
    rclpy.init(args=args)
    node = TcpServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down TCP Server Node...')
    finally:
        if node.client_socket:
            node.client_socket.close()
        if node.server_socket:
            node.server_socket.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


