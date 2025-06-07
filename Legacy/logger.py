#!/usr/bin/env python3

import rclpy
import math
import csv
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile

class OdomLogger(Node):
    def __init__(self):
        super().__init__('odom_logger')
        self.get_logger().info("Logger node has been started...")

        # Crear archivo CSV
        self.file = open('odom_log.csv', mode='w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['timestamp',
                              'robot1_x', 'robot1_y', 'robot1_theta_z',
                              'robot2_x', 'robot2_y', 'robot2_theta_z'])

        qos = QoSProfile(depth=10)

        # Subscripciones
        self.sub_odom1 = self.create_subscription(
            Odometry, '/ground_truth', self.odom1_callback, qos)
        self.sub_odom2 = self.create_subscription(
            Odometry, '/odom', self.odom2_callback, qos)

        self.last_odom1 = None
        self.last_odom2 = None

        # Temporizador para guardar cada 0.2s
        self.timer = self.create_timer(0.2, self.timer_callback)

    def odom1_callback(self, msg):
        self.last_odom1 = msg

    def odom2_callback(self, msg):
        self.last_odom2 = msg

    def timer_callback(self):
        if self.last_odom1 is None or self.last_odom2 is None:
            return

        # Extraer posiciones
        def extract_pose(msg):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            quat = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            return x, y, yaw

        x1, y1, yaw1 = extract_pose(self.last_odom1)
        x2, y2, yaw2 = extract_pose(self.last_odom2)
        timestamp = self.get_clock().now().nanoseconds * 1e-9

        # Escribir al CSV
        self.writer.writerow([timestamp, x1, y1, yaw1, x2, y2, yaw2])

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Finalizando por usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()