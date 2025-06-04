#!/usr/bin/env python3

import rclpy, math, time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from aruco_opencv_msgs.msg import ArucoDetection  # ← Tipo corregido
from tf_transformations import quaternion_from_euler


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_kalman")
        self.get_logger().info("Odometry node with Kalman filter has started...")

        # Namespace Parameters
        self.namespace = self.declare_parameter('namespace', "").value

        # QoS para simulación
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Subscriptions
        self.subR = self.create_subscription(Float32, "/VelocityEncR", self.callback_encR, qos_profile)
        self.subL = self.create_subscription(Float32, "/VelocityEncL", self.callback_encL, qos_profile)
        self.sub_ar = self.create_subscription(ArucoDetection, "/aruco_detections", self.aruco_callback, 1)
        self.sub_cmd = self.create_subscription(Twist, self.namespace + "/cmd_vel", self.get_req_velocities, 1)

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, self.namespace + "/odom", 10)

        # Timers
        self.create_timer(0.1, self.odometry_callback)

        # Inicial variables

        #Inicial State
        self.estado = np.zeros((3, 1))  # x, y, theta

        #Confiability Matrix
        self.E = np.array(
            [[0.0, 0.0, 0.0 ],
             [0.0, 0.0, 0.0 ],
             [0.0, 0.0, 0.0 ]])


        #Covariance Matrix from the Camera
        self.Rk = np.array([[0.061, 0.0022],
                             [0.0022, 0.008]])  # Covarianza de medición

        self.R = 0.0505  # Radio rueda
        self.D = 0.17    # Distancia entre ruedas
        self.kl = 0.1333 #self.kl = 0.1333
        self.kr = 0.1351 #self.kr = 0.1351

        #Time variables
        self.now = self.get_clock().now().seconds_nanoseconds()
        self.tin = self.now[0] + self.now[1] * 1e-9

        self.tfin = self.tin

        #Variables de velocidad de llantas y velocidad linear
        self.vel = 0.0
        self.wR = 0.0
        self.wL = 0.0
        
        #Arucos
        
        self.Detected_AR = []
        
        self.arucodict = {
            #0:  [1.75, 2.0],
            1:  [-1.53, 0.15],
            2:  [1.435, -0.15],
            3:  [0.0, -1.03],
            4:  [0.0, 1.03],
            5:  [-1.53, -0.15],
            #6:  [1.075, 0.0],
            #7:  [-2.31996, 0.4594],
            #8:  [-2.5379, 1.6768],
            #9:  [-1.4336, -1.4504],
            #10: [-0.4928, -1.5174]
        }

        # Camera to Robot Transform
        self.trans = np.array([
            [0.0, 0.0, 1.0, 0.07],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.08],
            [0.0, 0.0, 0.0, 1.0]
        ])


    #Information Callbacks
    def aruco_callback(self, msg):
        self.Detected_AR = msg.markers 

    def callback_encR(self, msg):
        self.wR = msg.data

    def callback_encL(self, msg):
        self.wL = msg.data

    def get_req_velocities(self, msg):
        self.vel = msg.linear.x



    def odometry_callback(self):
        self.now = self.get_clock().now().seconds_nanoseconds()
        self.tfin = self.now[0] + self.now[1] * 1e-9
        delta = self.tfin - self.tin
        self.tin = self.tfin

        # Estimated movement by odometry
        lin_vel = self.R * (self.wL + self.wR) / 2
        ang_vel = self.R * (self.wR - self.wL) / self.D
        theta = self.estado[2][0] + ang_vel * delta
        theta = math.atan2(math.sin(theta), math.cos(theta))  # Angle normalization 
        x = self.estado[0][0] + lin_vel * math.cos(theta) * delta
        y = self.estado[1][0] + lin_vel * math.sin(theta) * delta

        # Matriz covarianza motores
        sigma = np.diag([self.kr * abs(self.wR), self.kl * abs(self.wL)])

        # Matriz de gradientes (nabla)
        nabla = (self.R * delta / 2) * np.array([
            [math.cos(self.estado[2][0]), math.cos(self.estado[2][0])],
            [math.sin(self.estado[2][0]), math.sin(self.estado[2][0])],
            [2 / self.D, -2 / self.D]
        ])

        Qk = nabla @ sigma @ nabla.T #Noise Calculation


        H = np.array([
            [1.0, 0.0, -delta * lin_vel * math.sin(self.estado[2][0])],
            [0.0, 1.0, delta * lin_vel * math.cos(self.estado[2][0])],
            [0.0, 0.0, 1.0]
        ])

        #Prediction of State
        self.E = H @ self.E @ H.T + Qk
        self.estado = np.array([[x], [y], [theta]])

        # Corrección of State with Aruco
        for aruco in self.Detected_AR:
            id = aruco.marker_id

            #Check for false positives
            if id not in self.arucodict:
                continue

            # Frame Transformation
            p = np.array([[aruco.pose.position.x], [aruco.pose.position.y], [aruco.pose.position.z], [1]])
            p_robot = self.trans @ p

            #Check for distance to Aruco
            if np.linalg.norm(p_robot[:2]) > 10.0:
                continue

            # Observed measurment conditioning
            zik = np.array([
                [np.linalg.norm(p_robot[:2])],
                [math.atan2(p_robot[1][0], p_robot[0][0])]
            ])

            #Expected measurment calculation
            dx = self.arucodict[id][0] - self.estado[0][0]
            dy = self.arucodict[id][1] - self.estado[1][0]
            dist = math.hypot(dx, dy)
            angle = math.atan2(math.sin(math.atan2(dy, dx) - self.estado[2][0]),
                               math.cos(math.atan2(dy, dx) - self.estado[2][0]))

            g = np.array([[dist], [angle]])

            #Linarization matrix
            G = np.array([
                [-dx / dist, -dy / dist, 0],
                [dy / (dx ** 2 + dy ** 2), -dx / (dx ** 2 + dy ** 2), -1]
            ])

            #Correction of Confidence and State
            S = G @ self.E @ G.T + self.Rk
            K = self.E @ G.T @ np.linalg.inv(S)
            innovation = zik - g
            self.estado += K @ innovation
            self.E = self.E - K @ G @ self.E

        # Message Publishing
        odom_msg = Odometry()
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = self.namespace + "/base_footprint"
        odom_msg.pose.pose.position.x = self.estado[0][0]
        odom_msg.pose.pose.position.y = self.estado[1][0]
        odom_msg.pose.pose.position.z = self.R

        q = quaternion_from_euler(0, 0, self.estado[2][0])
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        cov = odom_msg.pose.covariance
        cov[0] = self.E[0][0]
        cov[1] = self.E[0][1]
        cov[5] = self.E[0][2]
        cov[6] = self.E[1][0]
        cov[7] = self.E[1][1]
        cov[11] = self.E[1][2]
        cov[30] = self.E[2][0]
        cov[31] = self.E[2][1]
        cov[35] = self.E[2][2]

        self.pub_odom.publish(odom_msg)
        self.Detected_AR = []  # Reset detecciones

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node terminated by user.")
