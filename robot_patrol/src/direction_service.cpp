// ROS
#include "rclcpp/rclcpp.hpp"
#include <memory>

// Service 
#include "std_srvs/srv/empty.hpp"
#include "robot_patrol/srv/get_direction.hpp"

// Message
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using Empty = std_srvs::srv::Empty;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node
{
public:
  DirectionService()
  : Node("direction_service")
  {

    srv_ = create_service<Empty>("moving", std::bind(&DirectionService::moving_callback, this, _1, _2));
    
    // Cmd_Vel PUB
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Laser Sub
        laser_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options1;
        options1.callback_group = laser_callback_group_;

        subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::Laser_callback, this, std::placeholders::_1),
        options1);

  }

private:

    // Service stuff
    rclcpp::Service<Empty>::SharedPtr srv_;

    void moving_callback(
      const std::shared_ptr<Empty::Request> request,
      const std::shared_ptr<Empty::Response>
          response) 
    {

        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.2;
        message.angular.z = 0.2;
        publisher_->publish(message);
    }

    // Laser Sub stuff
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
    
    // Define the subscription to the Laser Scan 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
    rclcpp::CallbackGroup::SharedPtr laser_callback_group_;

    // Define the publisher to the cmd vel
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Member Variables
    std::list<float> laser_ranges_;
    float direction_;
    float total_dist_sec_right = 0.0;
    float total_dist_sec_front = 0.0;
    float total_dist_sec_left = 0.0;
    float opposite_direction_ = 0.00;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  std::shared_ptr<DirectionService> direction_srv_node = std::make_shared<DirectionService>();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(direction_srv_node);
  executor.spin();
  
  rclcpp::shutdown();

  return 0;
}
