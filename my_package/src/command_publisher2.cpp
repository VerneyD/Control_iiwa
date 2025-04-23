#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher()
        : Node("command_publisher")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&CommandPublisher::joy_callback, this, std::placeholders::_1));  
            // subscription to the controller's state: this will allow us to get the position of the joysticks
            // It is a message of type sensor_msgs/msg/joy on the topic  /joy

        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/servo_node/delta_twist_cmds", 10);
            // this programm will publish a command in velocity. It needs to be a message of type
            // geometry_msgs/msg/twist_stamped on the topic /servo_node/delta_twist_cmds

        scale_linear_ = 0.2; // maximum linear velocity (found in .yaml file)
        scale_angular_ = 0.2; // maximum angular velocity (found in .yaml file)

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(&CommandPublisher::publish_command, this)); //the message will be published at a 30Hz rate 
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        last_joy_msg_ = msg; //renaming the message from the joystick for clarity purpose
    }

    void publish_command()
    {
        if (!last_joy_msg_) return; // if we get some data from the controller

        auto msg = geometry_msgs::msg::TwistStamped(); // we create the message that we will publish (type eometry_msgs/msg/twist_stamped)
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "iiwa_base"; // the base frame for the velocity's value
            

        if (last_joy_msg_->buttons[4]== 0 ){
            msg.twist.linear.x = scale_linear_ * last_joy_msg_->axes[0]; // left horizontal joystick controls the x-axis movement
            msg.twist.linear.y = scale_linear_ * last_joy_msg_->axes[1]; // left vertical joystick controls the y-axis movement
            msg.twist.linear.z = scale_linear_ * last_joy_msg_->axes[4]; // right vertical joystick controls the z-axis movement
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing TwistStamped message, controlling position of the effector"); 
            // we print that the message was successfully published in the terminal
        }
        
        if (last_joy_msg_->buttons[4]== 1 ){  //if LB pressed continuously we command the effector's orientation 
            msg.twist.angular.x = scale_angular_ * last_joy_msg_->axes[0];
            msg.twist.angular.y = scale_angular_ * last_joy_msg_->axes[1];
            msg.twist.angular.z = scale_angular_ * last_joy_msg_->axes[4];
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing TwistStamped message, controlling orientation of the effector"); 
            // we print that the message was successfully published in the terminal
        }
        
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;  
    double scale_linear_;
    double scale_angular_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
