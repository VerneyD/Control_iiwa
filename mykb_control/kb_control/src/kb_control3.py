#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import curses
import numpy as np

def dh_transform(a, alpha, d, theta):
    """Retourne la matrice de transformation homogène utilisant les paramètres DH modifiés."""
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0, 0, 0, 1]
    ])
# Paramètres DH modifiés pour le LBR iiwa 14
a_list = [0, 0, 0, 0, 0, 0, 0]
alpha_list = [0,-np.pi/2, np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, np.pi/2]
d_list = [0.1575, 0, 0.2025, 0, 0.2045, 0, 0.1600]


def forward_kinematics(q): #Renvoie la matrice de transformation calculée à un point correspondant à ces angles.

    T = np.eye(4)
    for i in range(7):
        T_i = dh_transform(a_list[i], alpha_list[i], d_list[i], q[i])
        T = np.dot(T, T_i)
    return T

def end_effector_position(q):
    """Retourne uniquement la position (x, y, z) de l'effecteur en fonction des 7 angles."""
    T = forward_kinematics(q)
    position = T[0:3, 3]  # On prend la 4e colonne, lignes 0 à 2
    return position


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/iiwa_arm_controller/joint_trajectory', 10)
        self.get_logger().info('TrajectoryPublisher node initialized')

    def send_trajectory(self):
        self.get_logger().info("Sending trajectory...")

        msg = JointTrajectory()
        msg.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6', 'joint_a7']
        # current_position = [0.0,1.2,3.2,1.0,0.0,-0.2,0.0] C'est une position intuitive au début 
        current_position = [0.0,0.0,0.0,0.0,0.0,0.0,0.0] #bras droit

        self.get_logger().info(f"Position de l'effecteur: {end_effector_position(current_position)}")

        #On met le robot en position initiale dès le début
        point = JointTrajectoryPoint()
        point.positions = current_position
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        msg.points = []  
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"Trajectory published: {current_position}")
        
        step = 0.2  # valeur d'incrément

        stdscr = curses.initscr()
        curses.cbreak()
        stdscr.keypad(True)
        
        try:
            self.get_logger().info("Press Z to exit" \
            "Press buttons to command position" 
            )
            key = stdscr.getch()
            
            while key != ord('z'):
                while key != ord('a') and key != ord('e') and key != ord('z') and key != ord('r') and key != ord('t') and key != ord('y') and key != ord('u') and key != ord('i') and key != ord('o') and key != ord('s') and key != ord('q') and key != ord('d') and key != ord('f') and key != ord('g') and key != ord('h'):
                    self.get_logger().info("Try again")
                    key = stdscr.getch()

                if key == ord('z'):
                    self.get_logger().info("Exit requested")
                    break

                # Créer un nouveau point à chaque fois
                point = JointTrajectoryPoint()

#tous les cas possibles, à chaque appui sur le bouton
                if key == ord('a'):
                    self.get_logger().info("Incrementing position joint 1")
                    current_position[0] += step

                elif key == ord('e'):
                    self.get_logger().info("Decrementing position joint 1")
                    current_position[0] -= step


                elif key == ord('r'):
                    self.get_logger().info("Decrementing position joint 2")
                    current_position[1] += step

            
                elif key == ord('t'):
                    self.get_logger().info("Decrementing position joint 2")
                    current_position[1] -= step


                elif key == ord('y'):
                    self.get_logger().info("Decrementing position joint 3")
                    current_position[2] += step


                elif key == ord('u'):
                    self.get_logger().info("Decrementing position joint 3")
                    current_position[2] -= step


                elif key == ord('i'):
                    self.get_logger().info("Decrementing position joint 4")
                    current_position[3] += step


                elif key == ord('o'):
                    self.get_logger().info("Decrementing position joint 4")
                    current_position[3] -= step


                elif key == ord('q'):
                    self.get_logger().info("Decrementing position joint 5")
                    current_position[4] += step


                elif key == ord('s'):
                    self.get_logger().info("Decrementing position joint 5")
                    current_position[4] -= step


                elif key == ord('d'):
                    self.get_logger().info("Decrementing position joint 6")
                    current_position[5] += step


                elif key == ord('f'):
                    self.get_logger().info("Decrementing position joint 6")
                    current_position[5] -= step


                elif key == ord('g'):
                    self.get_logger().info("Decrementing position joint 7")
                    current_position[6] += step

                
                elif key == ord('h'):
                    self.get_logger().info("Decrementing position joint 8")
                    current_position[6] -= step



                

                point.positions = current_position
                point.time_from_start.sec = 2
                point.time_from_start.nanosec = 0

                msg.points = []  # <-- très important : ne pas accumuler
                msg.points.append(point)
                self.publisher.publish(msg)
                self.get_logger().info(f"Trajectory published: {current_position}")
                self.get_logger().info(f"Position de l'effecteur: {end_effector_position(current_position)}")

                
                # Attendre la prochaine touche
                key = stdscr.getch()
        
        finally:
            curses.nocbreak()
            stdscr.keypad(False)
            curses.echo()
            curses.endwin()


def main(args=None):
    rclpy.init(args=args)

    trajectory_publisher = TrajectoryPublisher()
    trajectory_publisher.send_trajectory()

    rclpy.spin(trajectory_publisher)

    trajectory_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
