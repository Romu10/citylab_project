// ROS
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <rclcpp/rclcpp.hpp>

// Messages
#include "custom_interfaces/srv/get_direction.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using GetDirection = custom_interfaces::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;

class Patrol : public rclcpp::Node {

public:
  // Constructor
  Patrol() : Node("Robot_Patrol_Service") {

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
    publisher1_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

    // Timer Loop 10Hz
    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    this->wait_time = 1.0;
    timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::timer_callback, this), timer_callback_group_);

    // Service Client
    client = this->create_client<GetDirection>("/direction_service");

  }
    
    bool is_service_done() const {
      // inspired from action client c++ code
      return this->service_done_;
    }

private:
  // Laser Sub
  void Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Message received
    int range_total = msg->ranges.size();
    last_laser_ = msg;
    RCLCPP_INFO(this->get_logger(), "Total Laser Received: %i", range_total);
  }

  void timer_callback() {
    RCLCPP_INFO(this->get_logger(), "ROBOT CONTROLLER");

    while (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "Service not available, waiting again...");
    }

    // Create your custom request based on your service definition
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *last_laser_;

    service_done_ = false; // inspired from action client c++ code

    auto result_future = client->async_send_request(
        request, [this](rclcpp::Client<GetDirection>::SharedFuture future) {
            this->response_callback(future);
        }
    );
    return;
  }

  void response_callback(
      rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture future) {
    auto status = future.wait_for(1s);
    
    // Define Twist message to publish in Cmd_Vel
    geometry_msgs::msg::Twist message;
    
    if (status == std::future_status::ready) {

      service_done_ = true;

      auto result = future.get();

      if (result->direction == "Move forward") {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned Front");
        message.linear.x = 0.1;
        message.angular.z = 0.0;
      } else if (result->direction == "Turn right") {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned Right");
        message.linear.x = 0.1;
        message.angular.z = -0.5;
      } else if (result->direction == "Turn left") {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned Left");
        message.linear.x = 0.1;
        message.angular.z = 0.5;
      } else {
        // RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service
        // /direction_server");
      }

      // Publish the data
      this->publisher1_->publish(message);

      // Print the data
      RCLCPP_INFO(this->get_logger(), "Linear Velocity in X: %f",
                  message.linear.x);
      RCLCPP_INFO(this->get_logger(), "Angular Velocity in Z: %f",
                  message.angular.z);

    }

    else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
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
  LaserScan::SharedPtr last_laser_;
  bool service_done_ = false;

  // Server Client
  rclcpp::Client<GetDirection>::SharedPtr client;
  rclcpp::Client<GetDirection>::SharedFuture result_future;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<Patrol> patrol_node = std::make_shared<Patrol>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}