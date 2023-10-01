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
        srv_ = create_service<GetDirection>("moving", std::bind(&DirectionService::direction_callback, this, _1, _2));
        
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
        int number_of_rays_received = received_laser_data.size();
        
        // For Divide the Laser Data Received 
        std::vector<std::vector<float>> divided_laser_data(num_sections);

        for (int i = 0; i < num_sections; ++i){
            
            const float startAngle = -angle_per_section / 2.0 + i * angle_per_section;
            const float endAngle = startAngle + angle_per_section;

            for (int j = 0; j < number_of_rays_received; ++j){
                
                float angle = received_laser_data[j];

                while (angle < -M_PI){
                    angle += 2.0 * M_PI;
                }
                while (angle > M_PI){
                    angle -= 2.0 * M_PI;
                }
                if (angle >= startAngle && angle <= endAngle){
                    divided_laser_data[i].push_back(received_laser_data[j]);
                }
            
            }
        
        }

        int number_of_rays_received_right = divided_laser_data[1].size();
        RCLCPP_INFO(this->get_logger(), "Total Laser Received Right: %i", number_of_rays_received_right);

        response->direction = "Left";
                
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DirectionService>());
    rclcpp::shutdown();
    return 0;
}
