#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"

class DriveTurtleContinuously : public rclcpp::Node
{
public:
    DriveTurtleContinuously() : Node("drive_turtle_continuously"), drive_turtle_(false)
    {
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drive_turtle", 10, std::bind(&DriveTurtleContinuously::drive_callback, this, std::placeholders::_1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&DriveTurtleContinuously::publish_velocity, this));

        RCLCPP_INFO(this->get_logger(), "Started 'drive_turtle_continuously' node");
    }

private:
    void drive_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        drive_turtle_ = msg->data;
        if (!drive_turtle_) {
            // Immediately stop the turtle
            auto stop_msg = geometry_msgs::msg::Twist();
            cmd_vel_publisher_->publish(stop_msg);
            RCLCPP_INFO(this->get_logger(), "Received stop command. Stopping turtle.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Received drive command. Starting turtle.");
        }
    }

    void publish_velocity()
    {
        if (drive_turtle_)
        {
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = 3.0;  // Set linear x velocity to 1.0
            cmd_vel_publisher_->publish(twist_msg);
            //RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel to move turtle.");
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool drive_turtle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveTurtleContinuously>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
