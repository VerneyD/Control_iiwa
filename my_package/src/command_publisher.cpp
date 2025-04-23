#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/header.hpp"

class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher()
        : Node("command_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);

        // Timer to send the messages at 30 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // 30 Hz
            std::bind(&CommandPublisher::publish_command, this)
        );
    }

private:
    void publish_command()
    {
        auto msg = geometry_msgs::msg::TwistStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "iiwa_base";

        msg.twist.linear.x = 0.0;
        msg.twist.linear.y = 0.0;
        msg.twist.linear.z = 0.0;
        msg.twist.angular.x = 0.2;
        msg.twist.angular.y = 0.2;
        msg.twist.angular.z = 0.2;

        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing TwistStamped message");
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
