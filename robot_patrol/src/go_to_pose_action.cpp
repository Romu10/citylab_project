#include "custom_interfaces/action/detail/go_to_pose__struct.hpp"
#include "custom_interfaces/action/go_to_pose.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/subscription.hpp"

class GoToPose : public rclcpp::Node
{
    public:
    
        using GoToPose = custom_interfaces::action::GoToPose;
        using GoalHandlePose = rclcpp_action::ServerGoalHandle<GoToPose>;

        explicit GoToPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("go_to_pose_action_server", options)
        {
            using namespace std::placeholders;

            this->action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "/go_to_pose",
            std::bind(&GoToPose::handle_goal, this, _1, _2),
            std::bind(&GoToPose::handle_cancel, this, _1),
            std::bind(&GoToPose::handle_accepted, this, _1));

            // Configuring Twist Publisher
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);


        }
    

    private:

        // Defining Twist Publisher
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

        // Defining Action 
        rclcpp_action::Server<GoToPose>::SharedPtr action_server_;

        // Defining Sub to Odom
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Subcriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoToPose>());
    rclcpp::shutdown();
    return 0;
}

