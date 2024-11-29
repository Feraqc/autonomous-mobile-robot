import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class DifferentialDriveRobot:
    def __init__(self, x=0.0, y=0.0, theta=0.0, wheel_distance=0.5):
        self.x = x  # Position in x
        self.y = y  # Position in y
        self.theta = theta  # Orientation in radians
        self.wheel_distance = wheel_distance  # Distance between wheels
        self.path_x = [x]  # List to store x positions
        self.path_y = [y]  # List to store y positions


        # Initialize PID control variables for linear distance
        self.linear_error_sum = 0.0
        self.linear_error_prev = 0.0

        # Initialize PID control variables for angular orientation
        self.angular_error_sum = 0.0
        self.angular_error_prev = 0.0

        # Linear PID gains
        self.kp_dist = 0.8
        self.ki_dist = 0.02
        self.kd_dist = 0.15

        # Angular PID gains
        self.kp_angle = 1.9
        self.ki_angle = 0.1
        self.kd_angle = 0.3

    def update_position(self, v, omega, dt):
        """Update robot position based on linear and angular velocities."""
        # Update the position and orientation using the motion model
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        self.path_x.append(self.x)
        self.path_y.append(self.y)

    def control_to_waypoint(self, waypoint, v_max=0.3, omega_max=0.7, dt=0.1):
        """Control robot to move towards a waypoint with PID control."""
        # Calculate distance and angle to the waypoint
        dx = waypoint[0] - self.x
        dy = waypoint[1] - self.y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self._normalize_angle(angle_to_goal - self.theta)

        # PID Control for Linear Velocity
        linear_error = distance
        self.linear_error_sum += linear_error * dt
        linear_derivative = (linear_error - self.linear_error_prev) / dt
        self.linear_error_prev = linear_error

        # Calculate PID control for linear velocity
        v = (self.kp_dist * linear_error + 
            self.ki_dist * self.linear_error_sum +
            self.kd_dist * linear_derivative)
        
        v = min(v_max, max(-v_max, v))  # Clamp to max linear velocity

        # PID Control for Angular Velocity
        angular_error = angle_diff
        self.angular_error_sum += angular_error * dt
        angular_derivative = (angular_error - self.angular_error_prev) / dt
        self.angular_error_prev = angular_error

        # Calculate PID control for angular velocity
        omega = (self.kp_angle * angular_error + 
                self.ki_angle * self.angular_error_sum +
                self.kd_angle * angular_derivative)
        
        omega = min(omega_max, max(-omega_max, omega))  # Clamp to max angular velocity


            # Structure to hold PID tuning data
        pid_data = {
            "linear": {
                "error": linear_error,
                "P": self.kp_dist * linear_error,
                "I": self.ki_dist * self.linear_error_sum,
                "D": self.kd_dist * linear_derivative,
                "velocity": v
            },
            "angular": {
                "error": angular_error,
                "P": self.kp_angle * angular_error,
                "I": self.ki_angle * self.angular_error_sum,
                "D": self.kd_angle * angular_derivative,
                "velocity": omega
            }
        }

        # Print the structured PID data for tuning purposes
        print("PID Data:", pid_data)
            
        return v, omega


    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to be within the range [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def follow_path(self, waypoints, dt=0.1):
        """Make the robot follow a series of waypoints."""
        for waypoint in waypoints:
            while True:
                # Calculate control inputs to reach the current waypoint
                v, omega = self.control_to_waypoint(waypoint)
                # Update the robot's position
                self.update_position(v, omega, dt)
                # Check if the robot is close enough to the waypoint
                if math.hypot(waypoint[0] - self.x, waypoint[1] - self.y) < 0.1:
                    print(f"Reached waypoint {waypoint}")
                    break
                yield self.x, self.y, self.theta  # Yield position and orientation for real-time plotting


# Create a DifferentialDriveRobot instance
robot = DifferentialDriveRobot()

# Define waypoints for a more complex path
waypoints = [(1, 1), (1, 2), (2, 2), (3, 2), (3, 3), (4, 4), (5, 4), (6, 4)]

# Set up the plot
fig, ax = plt.subplots()
ax.set_xlim(0, 7)
ax.set_ylim(0, 6)
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.set_title("Real-Time Robot Path Following")
ax.grid()

# Plot the desired path connecting waypoints
desired_path_x = [w[0] for w in waypoints]
desired_path_y = [w[1] for w in waypoints]
ax.plot(desired_path_x, desired_path_y, 'r--', label="Desired Path")  # Dashed red line for the path
waypoints_plot, = ax.plot(desired_path_x, desired_path_y, 'rx', label="Waypoints")  # Waypoints as red 'x'

path_line, = ax.plot([], [], '-o', label="Actual Path")
robot_pose = ax.quiver(robot.x, robot.y, np.cos(robot.theta), np.sin(robot.theta), 
                       angles='xy', scale_units='xy', scale=1, color='blue')
ax.legend()

def init():
    path_line.set_data([], [])
    robot_pose.set_offsets([robot.x, robot.y])
    return path_line, robot_pose

def update(frame):
    x, y, theta = frame
    robot.path_x.append(x)
    robot.path_y.append(y)
    path_line.set_data(robot.path_x, robot.path_y)
    
    # Update the robot's pose (position and heading)
    robot_pose.set_offsets([x, y])
    robot_pose.set_UVC(np.cos(theta), np.sin(theta))  # Update direction of the arrow
    return path_line, robot_pose

# Animate the robot following the path
ani = animation.FuncAnimation(
    fig, update, frames=robot.follow_path(waypoints, dt=0.5), init_func=init,
    blit=True, interval=1000, repeat=False
)

plt.show()

