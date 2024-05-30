#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "turtlesim/srv/teleport_relative.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurnTurtle : public rclcpp::Node
{
public:
    TurnTurtle() : Node("turn_turtle")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/turn_angle", 1, std::bind(&TurnTurtle::angle_callback, this, std::placeholders::_1));
        
        client_ = this->create_client<turtlesim::srv::TeleportRelative>("turtle1/teleport_relative");
        //cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
        drive_turtle_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/drive_turtle", 1);

        RCLCPP_INFO(this->get_logger(), "Started 'turn_turtle' node");
    }

private:
    void angle_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double angle_degrees = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received angle: %.2f degrees. Stopping and turning turtle.", angle_degrees);
        turn_turtle(angle_degrees);
    }

    void turn_turtle(double angle_degrees)
    {
        // Stop the turtle's current movement
        /*
        auto stop_message = geometry_msgs::msg::Twist();
        stop_message.linear.x = 0.0;
        stop_message.angular.z = 0.0;
        cmd_vel_publisher_->publish(stop_message);
        */
        rclcpp::sleep_for(std::chrono::seconds(1)); // Wait for a second to ensure it stops

        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Service not available. Make sure the turtlesim node is running.");
            return;
        }

        auto request = std::make_shared<turtlesim::srv::TeleportRelative::Request>();
        request->linear = 0.0;
        request->angular = angle_degrees * (M_PI / 180.0); // Convert degrees to radians

        auto result = client_->async_send_request(request);
        // Don't wait for the service to respond! It does not.

        // Notify that the turning is finished
        auto drive_message = std_msgs::msg::Bool();
        drive_message.data = true;
        rclcpp::sleep_for(std::chrono::seconds(1));
        drive_turtle_publisher_->publish(drive_message);
    }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::Client<turtlesim::srv::TeleportRelative>::SharedPtr client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drive_turtle_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurnTurtle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
