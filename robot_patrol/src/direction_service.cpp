// ROS
#include "custom_interfaces/srv/detail/get_direction__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <memory>

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

class DirectionService : public rclcpp::Node
{
public:
  DirectionService()
  : Node("direction_service")
  {

    // Service Server
        srv_ = create_service<GetDirection>("direction_service", std::bind(&DirectionService::direction_callback, this, _1, _2));
        
    // Cmd_Vel PUB
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  }

private:

    // Service stuff
    rclcpp::Service<GetDirection>::SharedPtr srv_;

    // Define the publisher to the cmd vel
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // MEMBER VARIABLES //
    std::list<float> laser_ranges_;
    
    float direction_;
    float opposite_direction_ = 0.00;

    // Distance for each side
    const float total_dist_sec_right = 0.00;
    const float total_dist_sec_front = 0.00;
    const float total_dist_sec_left = 0.00;
    
    // Vector for storing received lasers values 
    std::vector<float> received_laser_data; 

    // Number of sections
    const int num_sections = 3;

    // Angle per section
    const float angle_per_section = (M_PI/3.0);

    void direction_callback(
      const std::shared_ptr<GetDirection::Request> request,
      const std::shared_ptr<GetDirection::Response> response) 
    {

        //auto message = geometry_msgs::msg::Twist();
        
        // For Received Laser Data 
        received_laser_data =  request->laser_data.ranges;

        // Defining the number of rays received 
        //int number_of_rays_received = received_laser_data.size();
        
        // For Divide the Laser Data Received 
        std::vector<std::vector<float>> divided_laser_data(num_sections);

        // Message received
int range_total = request->laser_data.ranges.size();
RCLCPP_INFO(this->get_logger(), "Total Laser Received: %i", range_total);

// Límites de las secciones
float front_start = -M_PI / 4.0;
float front_end = M_PI / 4.0;
float right_start = -M_PI;
float right_end = -M_PI / 4.0;
float left_start = M_PI / 4.0;
float left_end = M_PI;

// Initialize variables to keep track of the minimum ranges and their corresponding angles.
float min_front_range = std::numeric_limits<float>::max(); // Inicializar con un valor grande.
float min_front_angle = 0.0;
float min_right_range = std::numeric_limits<float>::max();
float min_right_angle = 0.0;
float min_left_range = std::numeric_limits<float>::max();
float min_left_angle = 0.0;

// Iterate through the laser scan data within each desired angle range.
for (size_t i = 0; i < request->laser_data.ranges.size(); ++i) {
    float angle = request->laser_data.angle_min + i * request->laser_data.angle_increment;
    float range = request->laser_data.ranges[i];

    // Check if the angle falls within the desired range.
    if (angle >= front_start && angle <= front_end) {
        // Check if the range is finite (not inf), less than the current minimum range, and within the specified threshold.
        if (std::isfinite(range) && range < min_front_range && range <= 2) {
            min_front_range = range;
            min_front_angle = angle;
        }
    } else if (angle >= right_start && angle <= right_end) {
        if (std::isfinite(range) && range < min_right_range && range <= 2) {
            min_right_range = range;
            min_right_angle = angle;
        }
    } else if (angle >= left_start && angle <= left_end) {
        if (std::isfinite(range) && range < min_left_range && range <= 2) {
            min_left_range = range;
            min_left_angle = angle;
        }
    }
}

    // Check if a close obstacle was detected in each section.
    if (min_front_range <= min_right_range && min_front_range <= min_left_range) {
        response->direction = "Front";
    } else if (min_right_range <= min_front_range && min_right_range <= min_left_range) {
        response->direction = "Right";
    } else if (min_left_range <= min_front_range && min_left_range <= min_right_range) {
        response->direction = "Left";
    } else {
        response->direction = "None";
    }

    std::string direction_to = response->direction;
    RCLCPP_INFO(this->get_logger(), "Direction: %s", direction_to.c_str());

    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DirectionService>());
    rclcpp::shutdown();
    return 0;
}
