#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <cmath>
#include <vector>
#include <sstream>

class SensorMeasurement : public rclcpp::Node{
    public:
        SensorMeasurement()
            : Node("sensor_measurement"),
            qos_best_effort_(rclcpp::QoS(rclcpp::KeepLast(10)).best_effort()),
            qos_reliable_(rclcpp::QoS(rclcpp::KeepLast(100)).reliable()),
            qos_profile_(rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local())
        {
            // Subscription to "sensor_measurement"
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "sensor_measurement", qos_best_effort_,
                std::bind(&SensorMeasurement::decodeAndStore, this, std::placeholders::_1));

            // Publishers for "robot_pose" and "robot_path"
            pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose", qos_reliable_);
            path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("robot_path", qos_profile_);
            waypoints_publisher_ = this->create_publisher<nav_msgs::msg::Path>("waypoint_path", qos_profile_);
            obstacles_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", qos_profile_);
            // Timers for periodic publishing
            pose_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&SensorMeasurement::publishPose, this));
            path_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&SensorMeasurement::publishPath, this));

        
            //path_();
            current_pose_.header.frame_id = "map";
            path_.header.frame_id = "map";
            waypoints_.header.frame_id = "map";
            obstacles_.header.frame_id = "map";
        }

        private:
            void decodeAndStore(const std_msgs::msg::String::SharedPtr msg){
                try{
                    std::istringstream iss(msg->data);
                    std::vector<std::string> tokens;
                    std::string token;
                    while (iss >> token)
                        tokens.push_back(token);

                    if (tokens.size() < 4)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Invalid data format");
                        return;
                    }

                    // Parse incoming data
                    const std::string data_type = tokens[0];
                    if (data_type == "1"){ // Pose data{
                        double x = std::stod(tokens[1]);
                        double y = std::stod(tokens[2]);
                        double theta_deg = std::stod(tokens[3]);
                        double theta_rad = theta_deg * M_PI / 180.0;
                        double waypoint_x = std::stod(tokens[4]);
                        double waypoint_y = std::stod(tokens[5]);

                        // // Combine remaining tokens into a single string
                        // std::string points_data;
                        // for (size_t i = 4; i < tokens.size(); ++i)
                        // {
                        //     points_data += tokens[i];
                        // }

                        // // Parse the comma-separated point data
                        // std::vector<float> point_values;
                        // std::istringstream point_stream(points_data);
                        // while (std::getline(point_stream, token, ','))
                        // {
                        //     try
                        //     {
                        //         point_values.push_back(100*std::stof(token));
                        //     }
                        //     catch (const std::invalid_argument &e)
                        //     {
                        //         RCLCPP_ERROR(this->get_logger(), "Invalid point value: %s", token.c_str());
                        //         return;
                        //     }
                        // }

                        // // Validate point data size
                        // if (point_values.size() % 2 != 0)
                        // {
                        //     RCLCPP_ERROR(this->get_logger(), "Invalid point data size. Must contain pairs of (x, y).");
                        //     return;
                        // }

                        // // Publish the point cloud
                        // publish_pointcloud(point_values);

                        // Create quaternion from yaw
                        tf2::Quaternion quaternion;
                        quaternion.setRPY(0, 0, theta_rad);

                        // Update current pose
                        current_pose_.pose.position.x = x;
                        current_pose_.pose.position.y = y;
                        current_pose_.pose.position.z = 0.0;
                        current_pose_.pose.orientation.x = quaternion.x();
                        current_pose_.pose.orientation.y = quaternion.y();
                        current_pose_.pose.orientation.z = quaternion.z();
                        current_pose_.pose.orientation.w = quaternion.w();

                        waypoint_pose_.pose.position.x = waypoint_x;
                        waypoint_pose_.pose.position.y = waypoint_y;
                        waypoint_pose_.pose.position.z = 0.0;

                        current_pose_.header.stamp = this->get_clock()->now();
                        waypoint_pose_.header.stamp = this->get_clock()->now();
                        path_.header.stamp = current_pose_.header.stamp;
                        path_.poses.push_back(current_pose_);
                        waypoints_.header.stamp = waypoint_pose_.header.stamp;
                        waypoints_.poses.push_back(waypoint_pose_);
                        RCLCPP_INFO(this->get_logger(), "Publishing Path: %zu poses", path_.poses.size());


                        // Limit the path size to avoid excessive memory usage
                        if (path_.poses.size() > max_path_length_)
                        {
                            path_.poses.erase(path_.poses.begin());
                        }
                        if (waypoints_.poses.size() > max_path_length_)
                        {
                            waypoints_.poses.erase(path_.poses.begin());
                        }
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "Unknown data type: %s", data_type.c_str());
                    }
                }
                catch (const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to decode data: %s, error: %s", msg->data.c_str(), e.what());
                }
            }

            void publishPose()
            {
                if (current_pose_.header.stamp.sec > 0) // Ensure pose has been initialized
                {
                    pose_publisher_->publish(current_pose_);
                }
            }

            void publishPath()
            {
                if (!path_.poses.empty())
                {
                    path_publisher_->publish(path_);
                }
                if (!waypoints_.poses.empty())
                {
                    waypoints_publisher_->publish(waypoints_);
                }
            }

            void publish_pointcloud(const std::vector<float> &obstacles){
                // Check if the vector size is valid
                if (obstacles.size() % 2 != 0)
                {
                    RCLCPP_ERROR(this->get_logger(), "Invalid obstacles vector size. Must contain pairs of (x, y).");
                    return;
                }

                size_t num_points = obstacles.size() / 2;

                sensor_msgs::msg::PointCloud2 cloud_msg;

                // Set PointCloud2 metadata
                cloud_msg.header.stamp = this->get_clock()->now();
                cloud_msg.header.frame_id = "map";  // Absolute positions are in the "map" frame

                cloud_msg.height = 1;                // Unordered point cloud
                cloud_msg.width = num_points;        // Number of points
                cloud_msg.is_dense = true;
                cloud_msg.is_bigendian = false;

                // Define fields: x, y, z
                sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
                modifier.setPointCloud2Fields(
                    3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
                modifier.resize(num_points);  // Resize based on number of points

                // Fill point cloud data
                sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

                for (size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z)
                {
                    *iter_x = obstacles[i * 2];       // X coordinate (even index)
                    *iter_y = obstacles[i * 2 + 1];   // Y coordinate (odd index)
                    *iter_z = 0.0f;                   // Z coordinate (default to 0)
                }

                // Publish the point cloud
                RCLCPP_INFO(this->get_logger(), "Publishing PointCloud2 with %zu points", num_points);
                obstacles_publisher_->publish(cloud_msg);
            }

                    
            
            // QoS profiles
            
            rclcpp::QoS qos_best_effort_;
            rclcpp::QoS qos_reliable_;
            rclcpp::QoS qos_profile_;

            // Publishers and subscriber
            rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
            rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_publisher_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_publisher_;

            // Timers
            rclcpp::TimerBase::SharedPtr pose_timer_;
            rclcpp::TimerBase::SharedPtr path_timer_;

            // Current pose and path
            geometry_msgs::msg::PoseStamped current_pose_;
            geometry_msgs::msg::PoseStamped waypoint_pose_;
            nav_msgs::msg::Path path_;
            nav_msgs::msg::Path waypoints_;
            size_t max_path_length_ = 100; 
            geometry_msgs::msg::Point obstacle_;
            sensor_msgs::msg::PointCloud2 obstacles_;


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorMeasurement>());
    rclcpp::shutdown();
    return 0;
}
