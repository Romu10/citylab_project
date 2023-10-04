#include "custom_interfaces/action/detail/go_to_pose__struct.hpp"
#include "custom_interfaces/action/go_to_pose.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/subscription.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "rclcpp_action/rclcpp_action.hpp"

class GoToPoseClass : public rclcpp::Node
{
    public:
    
        using GoToPose = custom_interfaces::action::GoToPose;
        using GoalHandlePose = rclcpp_action::ServerGoalHandle<GoToPose>;

        explicit GoToPoseClass(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("go_to_pose_action_server", options)
        {
            using namespace std::placeholders;

            this->action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "/go_to_pose",
            std::bind(&GoToPoseClass::handle_goal, this, _1, _2),
            std::bind(&GoToPoseClass::handle_cancel, this, _1),
            std::bind(&GoToPoseClass::handle_accepted, this, _1));

            // Configuring Twist Publisher
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

            // Configuring Odom Subscriber
            odom_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

            rclcpp::SubscriptionOptions options1;
            options1.callback_group = odom_callback_group_;

            Subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
            std::bind(&GoToPoseClass::Odom_callback, this, std::placeholders::_1), options1);
        }
    

    private:

        // Defining Twist Publisher
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

        // Defining Action 
        rclcpp_action::Server<GoToPose>::SharedPtr action_server_;

        // Defining Sub to Odom
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Subscriber_;
        rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

        // Variable
        geometry_msgs::msg::Pose2D current_pos_;

        void Odom_callback(nav_msgs::msg::Odometry::SharedPtr msg){

            // Convierte el cuaternión en ángulos de Euler
            tf2::Quaternion quat;
            tf2::fromMsg(msg->pose.pose.orientation, quat);
            double roll, pitch, yaw;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

            // Ahora "yaw" contiene el valor del ángulo en radianes
            current_pos_.theta = yaw;
            current_pos_.x = msg->pose.pose.position.x;
            current_pos_.y = msg->pose.pose.position.y;
        }

        void robot_control(float x, float y){
            // Define Twist message to publish in Cmd_Vel
            geometry_msgs::msg::Twist message;
            // Define proper linear velocity in X
            message.linear.x = x;
            // Define proper angular velocity in Z
            message.angular.z = y;
            //Publish the data
            this->publisher_->publish(message);
             // Print the data 
            RCLCPP_INFO(this->get_logger(), "Linear Velocity in X: %f", message.linear.x);
            RCLCPP_INFO(this->get_logger(), "Angular Velocity in Z: %f", message.angular.z);

        }

        // Función para calcular la diferencia de orientación entre dos ángulos en el rango [-π, π]
        double calculateYawDifference(double yaw1, double yaw2) {
            double diff = yaw2 - yaw1;
            while (diff > M_PI) {
                diff -= 2 * M_PI;
            }
            while (diff < -M_PI) {
                diff += 2 * M_PI;
            }
            return diff;
        }

        // Función para calcular la velocidad angular necesaria para girar hacia la orientación objetivo
        double calculateAngularVelocity(double current_yaw, double desired_yaw, double max_angular_velocity) {
            double yaw_difference = calculateYawDifference(current_yaw, desired_yaw);
            // Ajusta la velocidad angular para controlar la velocidad máxima permitida
            double angular_velocity = std::min(max_angular_velocity, std::max(-max_angular_velocity, yaw_difference));
            return angular_velocity;
        }

        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoToPose::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request");
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePose> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal - Stopping Robot");
            (void)goal_handle;
            robot_control(0.0, 0.0);
            return rclcpp_action::CancelResponse::ACCEPT;
        }

        void handle_accepted(const std::shared_ptr<GoalHandlePose> goal_handle)
        {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&GoToPoseClass::execute, this, _1), goal_handle}.detach();
        }

        void execute(const std::shared_ptr<GoalHandlePose> goal_handle)
        {
            RCLCPP_INFO(get_logger(), "Executing goal");

            // Obtiene el objetivo enviado por el cliente
            const auto goal = goal_handle->get_goal();

            // Inicializa mensajes de feedback y resultado
            auto feedback = std::make_shared<GoToPose::Feedback>();
            auto result = std::make_shared<GoToPose::Result>();

            // Inicializa un mensaje Twist para enviar comandos de movimiento al robot
            auto move = geometry_msgs::msg::Twist();

            // Inicializa una variable de velocidad lineal
            double linear_velocity = 0.2;  // 0.2 m/s
            
            // Calcula la diferencia de orientación entre current_pos_ y desired_pos_
            double max_angular_velocity = 0.49; 
            double stop_linear_velocity = 0.00;
            
            double current_yaw = current_pos_.theta;
            double desired_yaw = goal->goal_pos.theta;
            
            double error_percent = 10/100;
            double error_percent_linear = 0.080000;
            
            double down_margin = (desired_yaw-(desired_yaw * error_percent));
            double up_margin = (desired_yaw+(desired_yaw * error_percent));
            
            double yaw_difference = calculateYawDifference(current_yaw, desired_yaw);
            
            // Ajusta yaw_difference para que esté dentro del rango [-pi, pi]
            if (yaw_difference > M_PI) {
                yaw_difference -= 2 * M_PI;
            } else if (yaw_difference < -M_PI) {
                yaw_difference += 2 * M_PI;
            }
            
            double rounded_current_pos_x = std::round(current_pos_.x * 100) / 100.0;
            double rounded_goal_pos_x = std::round(goal->goal_pos.x * 100) / 100.0;
            double rounded_current_pos_y = std::round(current_pos_.y * 100) / 100.0;
            double rounded_goal_pos_y = std::round(goal->goal_pos.y * 100) / 100.0;
            
            double difference_x = std::abs(rounded_current_pos_x - rounded_goal_pos_x);
            double difference_y = std::abs(rounded_current_pos_y - rounded_goal_pos_y);
            
            // Calcula la velocidad angular necesaria para girar hacia la orientación objetivo
            double angular_velocity = calculateAngularVelocity(current_yaw, desired_yaw, max_angular_velocity);

            bool flag_opt = false;

            // Inicializa una tasa de bucle de 10 Hz (0.1 segundo)
            rclcpp::Rate loop_rate(20);

            // Realiza la acción principal aquí (puede ser un bucle o una secuencia de movimientos)
            
             RCLCPP_INFO(get_logger(), "Current X: %f",rounded_current_pos_x);
             RCLCPP_INFO(get_logger(), "Goal X: %f",rounded_goal_pos_x);
             RCLCPP_INFO(get_logger(), "Current Y: %f",rounded_current_pos_y);
             RCLCPP_INFO(get_logger(), "Goal Y: %f",rounded_goal_pos_y);
             RCLCPP_INFO(get_logger(), "Difference X: %f",difference_x);
             RCLCPP_INFO(get_logger(), "Difference Y: %f",difference_y);

            while ((difference_x > error_percent_linear || difference_y > error_percent_linear) && rclcpp::ok()) {
                // Comprueba si se ha solicitado la cancelación
                if (goal_handle->is_canceling()) {
                    result->status = false;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(get_logger(), "Goal canceled");
                    return;
                }

                RCLCPP_INFO(get_logger(), "Yaw Difference Gen: %f", yaw_difference);


                // Orientate the robot 
                if (std::abs(yaw_difference) > 0.1 && flag_opt == false) {
                    // Calcula el ángulo de orientación deseado para la posición objetivo actual
                    desired_yaw = std::atan2(goal->goal_pos.y - current_pos_.y, goal->goal_pos.x - current_pos_.x);

                    // Calcula la diferencia de orientación actualizada
                    yaw_difference = calculateYawDifference(current_yaw, desired_yaw);

                    RCLCPP_INFO(get_logger(), "Orienting Robot");
                    RCLCPP_INFO(get_logger(), "Yaw Difference: %f", yaw_difference);

                    move.linear.x = 0.0;  // Detiene el movimiento lineal

                    // Limita la velocidad angular si la diferencia angular es pequeña
                    if (yaw_difference < 0.00) {
                        move.angular.z = -0.05;
                    } else {
                        move.angular.z = 0.05;
                    }
                    
                    if (std::abs(yaw_difference) < 0.09) {
                        flag_opt = true;
                        move.angular.z = 0.0;  // Detiene el movimiento angular
                    }

                    publisher_->publish(move);
                } 
                else if (flag_opt == false) {
                    // El robot ya está orientado correctamente
                    flag_opt = true;
                    RCLCPP_INFO(get_logger(), "Robot Properly Oriented");
                }

                // Mueve el robot hacia adelante si ya está orientado correctamente
                if (flag_opt == true) {
                    RCLCPP_INFO(get_logger(), "Moving Robot Forward");
                    move.linear.x = linear_velocity;
                    move.angular.z = 0.0;
                    publisher_->publish(move);
                }

                // Publica retroalimentación (feedback)
                feedback->current_pos = current_pos_;
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(get_logger(), "Publish feedback");

                current_yaw = current_pos_.theta;

                rounded_current_pos_x = std::round(current_pos_.x * 100) / 100.0;
                rounded_current_pos_y = std::round(current_pos_.y * 100) / 100.0;

                difference_x = std::abs(rounded_current_pos_x - rounded_goal_pos_x);
                difference_y = std::abs(rounded_current_pos_y - rounded_goal_pos_y);

                RCLCPP_INFO(get_logger(), "Difference X: %f",difference_x);
                RCLCPP_INFO(get_logger(), "Difference Y: %f",difference_y);

                if (difference_x < error_percent_linear && difference_y < error_percent_linear){
                    RCLCPP_INFO(get_logger(), "Goal Reached");
                    break;
                }

                loop_rate.sleep();
            }

            // Comprueba si se completó el objetivo
            if (rclcpp::ok()) {
                result->status = true;
                move.linear.x = 0.0;
                move.angular.z = 0.0;
                publisher_->publish(move);
                goal_handle->succeed(result);
                RCLCPP_INFO(get_logger(), "Goal succeeded");
            }
          }
        };


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<GoToPoseClass> action_node = std::make_shared<GoToPoseClass>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

