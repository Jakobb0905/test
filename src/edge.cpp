#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

class Edge : public rclcpp::Node
{
public:
    Edge() : Node("edge"), should_look_for_edges_(false)
    {
        drive_turtle_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/drive_turtle", 1, std::bind(&Edge::drive_turtle_callback, this, std::placeholders::_1));
        turn_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/turn_angle", 1);
        drive_turtle_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/drive_turtle", 1);

        RCLCPP_INFO(this->get_logger(), "Started 'edge' node");
    }

private:
    void subscribe_to_rosout()
    {
        rosout_subscription_ = this->create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", 1, std::bind(&Edge::listener_callback, this, std::placeholders::_1));
    }

    void drive_turtle_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        should_look_for_edges_ = msg->data;
        if (should_look_for_edges_)
        {
            RCLCPP_INFO(this->get_logger(), "Received true on /drive_turtle. Subscribing to /rosout for edge detection.");
            subscribe_to_rosout();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Received false on /drive_turtle. Unsubscribing from /rosout.");
            rosout_subscription_.reset();
        }
    }

    void listener_callback(const rcl_interfaces::msg::Log::SharedPtr msg)
    {
        if (should_look_for_edges_ && msg->msg.find("Oh no! I hit the wall!") != std::string::npos)
        {
            RCLCPP_INFO(this->get_logger(), "Received message: 'Oh no! I hit the wall!'. Publishing angle and stopping turtle...");
            publish_drive_turtle(false);
            publish_turn_angle();
            // Unsubscribing from /rosout for 5 milliseconds to avoid multiple wall-hitting messages
            /**/
            rosout_subscription_.reset();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            //subscribe_to_rosout();
        }
    }

    void publish_turn_angle()
    {
        auto message = std_msgs::msg::Float64();
        message.data = -90.0; // 90 degrees clockwise
        turn_angle_publisher_->publish(message);
    }

    void publish_drive_turtle(bool data)
    {
        auto message = std_msgs::msg::Bool();
        message.data = data;
        drive_turtle_publisher_->publish(message);
    }

    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr drive_turtle_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr turn_angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drive_turtle_publisher_;
    bool should_look_for_edges_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Edge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
