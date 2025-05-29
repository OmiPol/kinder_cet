#!/usr/bin/env python3

import rclpy, math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from aruco_opencv_msgs.msg import ArucoDetection  # ← Tipo corregido
from tf_transformations import quaternion_from_euler
import tf_transformations as tf


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_kalman")
        self.get_logger().info("Odometry node with Kalman filter has started...")

        # Parámetro de namespace
        self.namespace = self.declare_parameter('namespace', "").value

        # QoS para simulación
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Suscripciones
        self.subR = self.create_subscription(Float32, "/VelocityEncR", self.callback_encR, qos_profile)
        self.subL = self.create_subscription(Float32, "/VelocityEncL", self.callback_encL, qos_profile)
        self.sub_ar = self.create_subscription(ArucoDetection, "/aruco_detections", self.aruco_callback, 1)
        self.sub_cmd = self.create_subscription(Twist, self.namespace + "/cmd_vel", self.get_req_velocities, 1)
        self.sub_clock = self.create_subscription(Clock, "/clock", self.get_time, 1)

        # Publicador
        self.pub_odom = self.create_publisher(Odometry, self.namespace + "/odom", 10)

        # Timer
        self.create_timer(0.1, self.odometry_callback)

        # Variables iniciales
        self.estado = np.array([[0.0],
                                [0.0],
                                [0.0]
                                ])  # x, y, theta


        self.E = np.array(
            [[0.0, 0.0, 0.0 ],
             [0.0, 0.0, 0.0 ],
             [0.0, 0.0, 0.0 ]])

        self.Rk = np.array([[1.0, -0.0002376, 0.5],
                             [-0.0002376, 1.0, 0.5],
                             [-0.0002376, -0.0002376, 2.35]
                             ])  # Covarianza de medición

        self.R = 0.0505  # Radio rueda
        self.D = 0.17    # Distancia entre ruedas
        self.kl = 0.8 #self.kl = 0.1333
        self.kr = 0.8 #self.kr = 0.1351

        self.tiempo = 0.0
        self.tin = 0.0
        self.tfin = 0.0
        self.vel = 0.0
        self.wR = 0.0
        self.wL = 0.0

        self.Detected_AR = []
        self.arucodict = {
            0:  [4.0, 0.0, math.pi],
            1:  [1.6464, -2.6464, 2.3592],
            #2:  [0.925, 0.0, -1.5708],
            3:  [0.0, 4.0, -1.5708],
            4:  [0.0, -4.0,1.5708],
            #5:  [-4.0, 0.0],
            #6:  [1.075, 0.0],
            7:  [-2.31996, 0.4594,0.165],
            8:  [-2.5379, 1.6768,0.165],
            #9:  [-1.4336, -1.4504],
            10: [-0.4928, -1.5174,0.7641]
        }

        # Transformación estática de la cámara al robot
        self.puzzle_camara = np.array([
            [0.0, 0.0, 1.0, 0.07],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.08],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.camera_puzzle = np.linalg.inv(self.puzzle_camara)

    def get_time(self, msg):
        self.tiempo = msg.clock.sec + msg.clock.nanosec / 1e9

    def aruco_callback(self, msg):
        self.Detected_AR = msg.markers 

    def callback_encR(self, msg):
        self.wR = msg.data

    def callback_encL(self, msg):
        self.wL = msg.data

    def get_req_velocities(self, msg):
        self.vel = msg.linear.x

    def odometry_callback(self):
        self.tfin = self.tiempo
        delta = self.tfin - self.tin
        self.tin = self.tfin

        # Movimiento estimado por odometría
        lin_vel = self.R * (self.wL + self.wR) / 2
        ang_vel = self.R * (self.wR - self.wL) / self.D
        theta = self.estado[2][0] + ang_vel * delta
        theta = math.atan2(math.sin(theta), math.cos(theta))  # Normalización
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

        Qk = nabla @ sigma @ nabla.T
        H = np.array([
            [1.0, 0.0, -delta * lin_vel * math.sin(self.estado[2][0])],
            [0.0, 1.0, delta * lin_vel * math.cos(self.estado[2][0])],
            [0.0, 0.0, 1.0]
        ])
        self.E = H @ self.E @ H.T + Qk
        self.estado = np.array([[x], [y], [theta]])

        # Corrección con ArUco
        for aruco in self.Detected_AR:
            id = aruco.marker_id
            if id not in self.arucodict:
                continue

            # Transformar al universo
            #matriz de rotación del aruco
            univ_aruco_rot1 = np.array([
                 [0.0, 0.0, 1.0, self.arucodict[id][0]],
                 [1.0, 0.0, 0.0, self.arucodict[id][1]],
                 [0.0, 1.0, 0.0, 0.075],
                 [0.0, 0.0, 0.0, 1.0]
            ])
            #Matriz transicionaria para la ubicación de aruco con respecto del universo
            mat_trans = tf.euler_matrix(self.arucodict[id][2],0.0,0.0,'syxz')

            #Matriz de rotación particular (en z universal) del aruco
            univ_aruco_rot2 = np.array([ 
                 [mat_trans[0][0], mat_trans[0][1], mat_trans[0][2], 0.0],
                 [mat_trans[1][0], mat_trans[1][1], mat_trans[1][2], 0.0],
                 [mat_trans[2][0], mat_trans[2][1], mat_trans[2][2], 0.0],
                 [0.0, 0.0, 0.0, 1.0]
                                         ])
            
            rotation = tf.quaternion_matrix([aruco.pose.orientation.x,
                                             aruco.pose.orientation.y, 
                                             aruco.pose.orientation.z, 
                                             aruco.pose.orientation.w])
            aruco_camara = np.array([

                [rotation[0,0],rotation[0,1],rotation[0,2],aruco.pose.position.x],
                [rotation[1,0],rotation[1,1],rotation[1,2],aruco.pose.position.y],
                [rotation[2,0],rotation[2,1],rotation[2,2],aruco.pose.position.z],
                [0.0,0.0,0.0,1.0]
            ])

            uni_puzzle = univ_aruco_rot1 @ univ_aruco_rot2 @ aruco_camara @ self.camera_puzzle

            angles_uni_puzzle = tf.euler_from_matrix(uni_puzzle,'sxyz')

            if math.hypot(aruco.pose.position.z, aruco.pose.position.x) > 10.0:
                continue

            # Medida observada
            zik = np.array([
                [uni_puzzle[0][3]],
                [uni_puzzle[0][2]],
                [angles_uni_puzzle[2]]
            ])

            

            g = np.array([[self.estado[0][0]],
                          [self.estado[1][0]],
                          [self.estado[2][0]]
                          ])

            G = np.array([
                [-1.0, 0.0, 0.0],
                [0.0, -1.0, 0.0],
                [0.0, 0.0, -1.0]
            ])
            Suma = G @ self.E @ G.T + self.Rk
            K = self.E @ G.T @ np.linalg.inv(Suma)
            innovation = zik - g
            #print(K @ innovation)

            self.estado = self.estado + K @ innovation
            self.E = self.E - K @ G @ self.E
           

        # Publicar mensaje Odometry


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
