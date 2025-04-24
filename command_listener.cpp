#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class CommandListener : public rclcpp::Node
{
public:
  CommandListener()
  : Node("command_listener")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", 10, std::bind(&CommandListener::servo_node/delta_twist_cmds_callback, this, std::placeholders::_1));
  }

private:
  void servo_node/delta_twist_cmds_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->twist.c_str());
  }
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandListener>());
  rclcpp::shutdown();
  return 0;
}