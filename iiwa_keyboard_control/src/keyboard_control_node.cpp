#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class TrajectoryPublisherNode : public rclcpp::Node
{
public:
    TrajectoryPublisherNode()
    : Node("trajectory_publisher_node")
    {
        // Crée le publisher
        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/iiwa_arm_controller/joint_trajectory", 10);

        // Appelle la fonction pour envoyer la trajectoire au démarrage
        send_trajectory();
    }

private:
    void send_trajectory()
    {
        // Créer le message de trajectoire
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.joint_names = {"joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"};
        
        // Définir la position des joints (comme dans le code Python)
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = {1.0, 0.5, 0.0, -0.5, -0.5, 0.0, 0.1}; // Exemple de positions des joints
        point.time_from_start = rclcpp::Duration::from_seconds(10.0);

        // Ajouter le point à la trajectoire
        traj_msg.points.push_back(point);
        
        // Publier le message de trajectoire
        pub_->publish(traj_msg);
        RCLCPP_INFO(this->get_logger(), "Trajectory published");
    }

    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Créer et exécuter le nœud
    rclcpp::spin(std::make_shared<TrajectoryPublisherNode>());

    // Arrêter proprement
    rclcpp::shutdown();
    return 0;
}
