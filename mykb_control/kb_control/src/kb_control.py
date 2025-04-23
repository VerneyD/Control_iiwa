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
        
        step = 0.1  # valeur d'incrément

        stdscr = curses.initscr()
        curses.cbreak()
        stdscr.keypad(True)
        
        try:
            self.get_logger().info("Press A to move to position 1, E to reset, Z to exit")
            key = stdscr.getch()
            
            while key != ord('z'):
                while key != ord('a') and key != ord('e') and key != ord('z'):
                    self.get_logger().info("Try again")
                    key = stdscr.getch()

                if key == ord('z'):
                    self.get_logger().info("Exit requested")
                    break

                # Créer un nouveau point à chaque fois
                point = JointTrajectoryPoint()

                if key == ord('a'):
                    self.get_logger().info("Incrementing position")
                    current_position = [pos + step for pos in current_position]

                elif key == ord('e'):
                    self.get_logger().info("Decrementing position")
                    current_position = [pos - step for pos in current_position]

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
