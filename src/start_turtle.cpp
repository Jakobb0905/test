#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cstdlib> // Include the C standard library for random number generation

class StartTurtle : public rclcpp::Node
{
public:
    StartTurtle() : Node("start_turtle")
    {
        subscription_ = this->create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", 10, std::bind(&StartTurtle::log_callback, this, std::placeholders::_1));
        
        angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/turn_angle", 1);
        //drive_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/drive_turtle", 1);

        RCLCPP_INFO(this->get_logger(), "Started 'start_turtle' node");
    }

private:
    void log_callback(const rcl_interfaces::msg::Log::SharedPtr msg)
    {
        if (msg->msg.find("Spawning turtle") != std::string::npos)
        {
            RCLCPP_INFO(this->get_logger(), "Detected 'Spawning turtle' message. Proceeding with actions.");
            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            publish_random_angle();
            //std::this_thread::sleep_for(std::chrono::milliseconds(1500));
            //publish_drive_true();
            rclcpp::shutdown();
        }
    }

    void publish_random_angle()
    {
        // Generate a random angle in range of 45 to 90, 135 to 180, 225 to 270, 315 to 360 degrees
        // this ensures, that the turtle can turn away from the wall with one 90 degree clockwise turn,
        // thus keeping the visuals interesting.
        // (all other angles result in the turtle just drawing a straight line between two walls,
        // as it alsways ahs to perofrm 180 degree rotations)
        /*
        double angle_degrees = rand() % 45;
        int modifier = rand() % 4;
        angle_degrees *=  -1;
        angle_degrees += modifier * 90;
        */

        double angle_degrees = generate_random_angle();

        auto angle_msg = std_msgs::msg::Float64();
        angle_msg.data = angle_degrees;
        angle_publisher_->publish(angle_msg);

        RCLCPP_INFO(this->get_logger(), "Published random angle: %.2f degrees", angle_msg.data);
    }

    void publish_drive_true()
    {
        auto drive_msg = std_msgs::msg::Bool();
        drive_msg.data = true;
        drive_publisher_->publish(drive_msg);

        RCLCPP_INFO(this->get_logger(), "Published 'true' to /drive_turtle");
    }

    double generate_random_angle()
    {
        // Seed the random number generator
        srand(time(nullptr));

        // Define the ranges for each quadrant
        int lower_bounds[] = {45, 135, 225, 315};
        int upper_bounds[] = {90, 180, 270, 360};

        // Randomly select a quadrant
        int quadrant = rand() % 4;

        // Generate a random angle within the selected quadrant
        int angle = rand() % (upper_bounds[quadrant] - lower_bounds[quadrant] + 1) + lower_bounds[quadrant];

        return angle;
    }

    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr drive_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StartTurtle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
