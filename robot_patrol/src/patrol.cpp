// ROS
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <rclcpp/rclcpp.hpp>

// Messages
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {

    public:
        // Constructor
        Patrol() : Node("Node_name"){

        // Laser Sub
        laser_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options1;
        options1.callback_group = laser_callback_group_;

        subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        odom_topic_name1, 10,
        std::bind(&Patrol::Laser_callback, this, std::placeholders::_1),
        options1);

        // Cmd_Vel Pub
        publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 1);

        // Timer Loop 10Hz
        timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        this->wait_time = 1.0;
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Patrol::timer_callback, this),
            timer_callback_group_);

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
        
        void timer_callback() {
            RCLCPP_INFO(this->get_logger(), "ROBOT CONTROLLER");

            // Define Twist message to publish in Cmd_Vel
            geometry_msgs::msg::Twist message;
            
            // Define proper linear velocity in X
            message.linear.x = 0.1;
            
            // Define proper angular velocity in Z
            message.angular.z = (direction_/2);

            // Print the data 
            RCLCPP_INFO(this->get_logger(), "Linear Velocity in X: %f", message.linear.x);
            RCLCPP_INFO(this->get_logger(), "Angular Velocity in Y: %f", message.angular.z);

            return; 
        }

        // Define the subscription to the Laser Scan 
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
        rclcpp::CallbackGroup::SharedPtr laser_callback_group_;

        // Define the publisher to the cmd_vel
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;

        // Define the timer 
        rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
        rclcpp::TimerBase::SharedPtr timer_;
        float wait_time;

        // Member Variables
        std::list<float> laser_ranges_;
        float direction_;
};

int main (int argc, char **argv){
    rclcpp::init(argc, argv);

    std::shared_ptr<Patrol> patrol_node = std::make_shared<Patrol>();
    rclcpp::spin(node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(patrol_node);
    executor.spin(node);

    rclcpp::shutdown();
    return 0;

};