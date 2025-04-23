#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/iiwa_arm_controller/joint_trajectory', 10)


    def send_trajectory(self , i):
        msg = JointTrajectory()
        msg.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6', 'joint_a7']
        
        point = JointTrajectoryPoint()
        point.positions = [0.5*i, 0.5*i,0.5*i,0.1*i,0.1*i,0.1*i, 1.0*i]
        point.time_from_start.sec = 1
        point.time_from_start.nanosec = 0
        
        msg.points.append(point)
        
        self.publisher.publish(msg)
        self.get_logger().info('Trajectory published')


def main(args=None):
    rclpy.init(args=args)

    # Créer l'instance du nœud
    trajectory_publisher = TrajectoryPublisher()

    # Envoyer la trajectoire
    i = 0
    while (i<10) : 
        i+=1
        trajectory_publisher.send_trajectory(i)
        time.sleep(2)


    # Garder le nœud en vie (pratique pour qu'il reste actif et puisse envoyer des messages)
    rclpy.spin(trajectory_publisher)

    # Arrêter proprement
    trajectory_publisher.destroy_node()
    rclpy.shutdown()
