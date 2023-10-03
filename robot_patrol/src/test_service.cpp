#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/get_direction.hpp"  // Cambia esta inclusión a la de tu servicio personalizado
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>

using namespace std::chrono_literals;
using GetDirection = custom_interfaces::srv::GetDirection; // Cambia este alias a tu servicio personalizado
using LaserScan = sensor_msgs::msg::LaserScan;

bool laser_callback_executed = false;
LaserScan::SharedPtr laser_data_received;

void laser_callback(const LaserScan::SharedPtr msg) {
  if (!laser_callback_executed) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Laser data received");
    laser_data_received = msg;
    laser_callback_executed = true;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("movement_client");
  rclcpp::Client<GetDirection>::SharedPtr client =
    node->create_client<GetDirection>("/direction_service"); // Cambia el nombre del servicio si es diferente

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
  }

  // Create a subscriber for the LaserScan message
  rclcpp::Subscription<LaserScan>::SharedPtr laser_subscription =
    node->create_subscription<LaserScan>(
      "/scan", 1, laser_callback);

  // Wait for laser data
  while (!laser_callback_executed) {
    rclcpp::spin_some(node);
  }

  // Create your custom request based on your service definition
  auto request = std::make_shared<GetDirection::Request>();
  request->laser_data = *laser_data_received;

  auto result_future = client->async_send_request(request);
  
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result_future) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = result_future.get();
    // Handle the response based on your service definition
    // You might need to change this part according to your service response structure
    
    if (result->direction == "Front")
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned Front");
    }
    else if (result->direction == "Right")
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned Right");
    }
    else if (result->direction == "Left")
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service returned Left");
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service /direction_server");
  }

  rclcpp::shutdown();
  return 0;
}
