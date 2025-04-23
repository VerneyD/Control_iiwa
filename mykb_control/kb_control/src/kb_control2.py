#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import curses


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher = self.create_publisher(JointTrajectory, '/iiwa_arm_controller/joint_trajectory', 10)
        self.get_logger().info('TrajectoryPublisher node initialized')

    def send_trajectory(self):
        self.get_logger().info("Sending trajectory...")

        msg = JointTrajectory()
        msg.joint_names = ['joint_a1', 'joint_a2', 'joint_a3', 'joint_a4', 'joint_a5', 'joint_a6', 'joint_a7']
        current_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
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
            "Press A and E to command joint 1" \
            "Press R and T to command joint 2" \
            "Press Y and U to command joint 3" \
            "Press I and O to command joint 4" \
            "Press Q and S to command joint 5" \
            "Press D and F to command joint 6" \
            "Press G and H to command joint 7"
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
