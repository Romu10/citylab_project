// ROS
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <rclcpp/rclcpp.hpp>

// Messages
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {

    public:
        // Constructor
        Patrol() : Node("Node_name"){

        // Laser Sub
        Laser_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options1;
        options1.callback_group = Laser_callback_group_;

        subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        odom_topic_name1, 10,
        std::bind(&Patrol::Laser_callback, this, std::placeholders::_1),
        options1);

        }

    private:

        // Laser Sub
        void Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            // Define constants for the desired angle range (between -π/2 and π/2).
            const float min_angle = -M_PI / 2.0;
            const float max_angle = M_PI / 2.0;

            // Initialize variables to keep track of the maximum range and its corresponding angle.
            float max_range = 0.0; // Initialize to 0.
            float max_range_angle = 0.0; // Initialize to 0.

            // Iterate through the laser scan data within the desired angle range.
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                float angle = msg->angle_min + i * msg->angle_increment;
                
                // Check if the angle falls within the desired range.
                if (angle >= min_angle && angle <= max_angle) {
                    float range = msg->ranges[i];
                    
                    // Check if the range is finite (not inf) and greater than the current maximum range.
                    if (std::isfinite(range) && range > max_range) {
                        max_range = range;
                        max_range_angle = angle;
                    }
                }
            }

            // Store the angle in the class variable direction_.
            direction_ = max_range_angle;
        }
        
        // Define the subscription to the Laser Scan 
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;

        // Member Variables
        std::list<float> laser_ranges_;
        float direction_;
};

int main (int argc, char **argv){


};