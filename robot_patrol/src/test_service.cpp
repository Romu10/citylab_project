// ROS
#include "custom_interfaces/srv/detail/get_direction__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <memory>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <iostream>

// Service 
#include "custom_interfaces/srv/get_direction.hpp"

// Message
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// Custom Interface
using GetDirection = custom_interfaces::srv::GetDirection;

// Place Holders
using std::placeholders::_1;
using std::placeholders::_2;

using namespace std::chrono_literals;

class ServiceClient : public rclcpp::Node
{
public:
    ServiceClient()
        : Node("direction_service")
    {
        // Creating Service Client
        client = create_client<GetDirection>("direction_service");

        // While Service Server is not Online Keep here
        while (!client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
        }

        // Laser Sub
        laser_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options1;
        options1.callback_group = laser_callback_group_;

        subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 1,
            std::bind(&ServiceClient::Laser_callback, this, std::placeholders::_1),
            options1);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service Client Ready");

    }

private:
    // Variables
    sensor_msgs::msg::LaserScan received_laser_scan;

    // Define Service Client
    rclcpp::Client<GetDirection>::SharedPtr client;

    // Define the subscription to the Laser Scan 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
    rclcpp::CallbackGroup::SharedPtr laser_callback_group_;

    // Variables
    bool laser_callback_executed;

    // CallBack Laser Sub
    void Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        
        // Verify
        if (laser_callback_executed) {
            return; // exit if executed
        }

        // Marcar como ejecutado
        laser_callback_executed = true;

        // Message received
        int range_total = msg->ranges.size();
        RCLCPP_INFO(this->get_logger(), "Total Laser Received: %i", range_total);

        // Crear una solicitud para el servicio
        auto request = std::make_shared<GetDirection::Request>();

        // Copiar los datos del mensaje láser al campo laser_data del request
        // Esto asume que los datos del láser son del mismo tipo que se espera en el servicio
        request->laser_data = *msg;

        // Enviar la solicitud al servicio
        auto result_future = client->async_send_request(request);

    }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServiceClient>());
    rclcpp::shutdown();
    return 0;
}
