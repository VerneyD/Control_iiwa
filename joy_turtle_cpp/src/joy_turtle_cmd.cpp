#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

class JoyToCmdVel : public rclcpp::Node
{
public:
    JoyToCmdVel() : Node("joy_to_cmd_vel")
    {
        // Paramètres de la manette
        axis_linear_ = 1;  // Stick gauche (haut/bas)
        axis_angular_ = 0; // Stick gauche (gauche/droite)

        scale_linear_ = 2.0;   // vitesse max en m/s
        scale_angular_ = 2.0;  // vitesse angulaire max en rad/s

        // Création du subscriber pour écouter le topic /joy
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&JoyToCmdVel::joy_callback, this, std::placeholders::_1));

        // Création du publisher pour publier sur le topic /cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist = geometry_msgs::msg::Twist();
        
        // Calcul de la vitesse linéaire et angulaire en fonction des axes de la manette
        twist.linear.x = msg->axes[axis_linear_] * scale_linear_;
        twist.angular.z = msg->axes[axis_angular_] * scale_angular_;
        
        // Publier la commande sur le topic /cmd_vel
        publisher_->publish(twist);
        RCLCPP_INFO(this->get_logger(), "Commande publiée : lin = %.2f, ang = %.2f", twist.linear.x, twist.angular.z);
    }

    int axis_linear_;
    int axis_angular_;
    double scale_linear_;
    double scale_angular_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyToCmdVel>());
    rclcpp::shutdown();
    return 0;
}
