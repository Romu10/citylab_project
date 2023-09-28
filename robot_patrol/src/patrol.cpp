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
        Patrol() : Node("Robot_Patrol"){

        // Laser Sub
        laser_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options1;
        options1.callback_group = laser_callback_group_;

        subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
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
            // Message received
            int range_total = msg->ranges.size();
            RCLCPP_INFO(this->get_logger(), "Total Laser Received: %i", range_total);

            // Define constants for the desired angle range (between -π/2 and π/2).
            const float min_angle = -M_PI / 2.0;
            const float max_angle = M_PI / 2.0;

            // Initialize variables to keep track of the minimum range and its corresponding angle.
            float min_range = std::numeric_limits<float>::max(); // Initialize to a large value.
            float min_range_angle = 0.0;

            // Iterate through the laser scan data within the desired angle range.
            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                float angle = msg->angle_min + i * msg->angle_increment;

                // Check if the angle falls within the desired range.
                if (angle >= min_angle && angle <= max_angle) {
                    float range = msg->ranges[i];

                    // Check if the range is finite (not inf), less than the current minimum range, and within the specified threshold.
                    if (std::isfinite(range) && range < min_range && range <= 0.5) {
                        min_range = range;
                        min_range_angle = angle;
                    }
                }
            }

            // Check if a close obstacle was detected (within 0.5 meters).
            if (min_range <= 0.42) {
                
                // Calculate the opposite direction to move away from obstacles.
                //opposite_direction_ = min_range_angle;
                //direction_ = opposite_direction_;
                direction_ = min_range_angle;

                // Determine whether to rotate left or right based on the angle.
                if (min_range_angle >= 0.0) {
                    // Rotate to the left.
                    direction_ -= M_PI / 1.3;
                } else {
                    // Rotate to the right.
                    direction_ += M_PI / 1.3;
                }
            }
            else{
                opposite_direction_ = 0.00;
                direction_ = opposite_direction_;
            }
                

            RCLCPP_INFO(this->get_logger(), "Direction: %f", direction_);
        }

        
        void timer_callback() {
            RCLCPP_INFO(this->get_logger(), "ROBOT CONTROLLER");

            // Define Twist message to publish in Cmd_Vel
            geometry_msgs::msg::Twist message;
            
            // Define proper linear velocity in X
            message.linear.x = 0.1;
            
            // Define proper angular velocity in Z
            message.angular.z = (direction_/2);

            //Publish the data
            this->publisher1_->publish(message);

            // Print the data 
            RCLCPP_INFO(this->get_logger(), "Linear Velocity in X: %f", message.linear.x);
            RCLCPP_INFO(this->get_logger(), "Angular Velocity in Z: %f", message.angular.z);

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
        float opposite_direction_ = 0.00;
};

int main (int argc, char *argv[]){
    rclcpp::init(argc, argv);

    std::shared_ptr<Patrol> patrol_node = std::make_shared<Patrol>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(patrol_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;

}