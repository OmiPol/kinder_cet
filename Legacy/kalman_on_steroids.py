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

        self.Rk = np.array([[0.2211, 0.0019, 0.0058],
                             [0.0019, 1.0, 0.0014],
                             [0.0058, 0.0014, 0.042]
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

        self.arucodictfull = {}

        for i in range(len(self.arucodict)):
            
            id = list(self.arucodict.keys())[i]
            print (id)
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
            uni_ar = univ_aruco_rot1 @ univ_aruco_rot2
            self.arucodictfull[id] =  uni_ar

        for i in range(len(self.arucodictfull)):
            id = list(self.arucodict.keys())[i]
            print(id)
            print(self.arucodictfull[id])

        # Transformación estática de la cámara al robot
        self.puzzle_camera = np.array([
            [0.0, 0.0, 1.0, 0.07],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.08],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.camera_puzzle = np.linalg.inv(self.puzzle_camera)

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

            if math.hypot(aruco.pose.position.z, aruco.pose.position.x) > 10.0:
                continue
            
            aruco_matrix = tf.quaternion_matrix([aruco.pose.orientation.x,
                                             aruco.pose.orientation.y, 
                                             aruco.pose.orientation.z, 
                                             aruco.pose.orientation.w])
            
            #fijo_movil
            camera_aruco = np.array([

                [aruco_matrix[0,0],aruco_matrix[0,1],aruco_matrix[0,2],aruco.pose.position.x],
                [aruco_matrix[1,0],aruco_matrix[1,1],aruco_matrix[1,2],aruco.pose.position.y],
                [aruco_matrix[2,0],aruco_matrix[2,1],aruco_matrix[2,2],aruco.pose.position.z],
                [0.0,0.0,0.0,1.0]
            ])

            
            

            puzzlerot = tf.euler_matrix(0.0,0.0,self.estado[2][0],'syxz')
            
            uni_puzzle = np.array([
                [puzzlerot[0][0],puzzlerot[0][1],puzzlerot[0][2],self.estado[0][0]],
                [puzzlerot[1][0],puzzlerot[1][1],puzzlerot[1][2],self.estado[1][0]],
                [puzzlerot[2][0],puzzlerot[2][1],puzzlerot[2][2],0.0],
                [0.0, 0.0, 0.0, 1.0]
            ])
            
            aruco_uni = np.linalg.inv(self.arucodictfull[id])

            camera_aruco = aruco_uni @ uni_puzzle @ self.puzzle_camera
            
            camera_aruco_rot = np.array([[camera_aruco[0][0],camera_aruco[0][1],camera_aruco[0][2]],
                                         [camera_aruco[1][0],camera_aruco[1][1],camera_aruco[1][2]],
                                         [camera_aruco[2][0],camera_aruco[2][1],camera_aruco[2][2]]])
            
            

            #Estimación de lectura con base al estado aproximado del robot
            camera_aruco_euler = tf.euler_from_matrix(camera_aruco_rot,'syxz')
            
            aruco_euler = tf.euler_from_quaternion([aruco.pose.orientation.x,
                                                   aruco.pose.orientation.y,
                                                   aruco.pose.orientation.z,
                                                   aruco.pose.orientation.w],
                                                   'syxz')
            # Medida observada
            zik = np.array([
                [aruco.pose.position.z],
                [aruco.pose.position.x],
                [aruco_euler[0]]
            ])

            
            #lectura en z en y x y angulo de rot en y
            g = np.array([[camera_aruco[2][3]],
                          [camera_aruco[0][3]],
                          [camera_aruco_euler[0]]
                          ])
            
            print("---zik----")
            print(zik)
            print("----g----")
            print(g)
            

            
            #TODO terminar de escribir matriz G
            G = np.array([
                [aruco_uni[2][0], aruco_uni[2][1], -0.07 * (aruco_uni[2][0]* math.sin(self.estado[2][0]) + aruco_uni[2][1] * math.cos(self.estado[2][0]))],
                [aruco_uni[0][0], aruco_uni[0][1], -0.07 * (aruco_uni[0][0]* math.sin(self.estado[2][0]) + aruco_uni[0][1] * math.cos(self.estado[2][0]))],
                [0.0, 0.0, (-aruco_uni[2][1] * math.sin(self.estado[2][0]) + aruco_uni[2][0] * math.cos(self.estado[2][0]))/math.sqrt(1-math.pow(aruco_uni[2][1] * math.cos(self.estado[2][0]) + aruco_uni[2][0] * math.sin(self.estado[2][0]),2))]
            ])
            Suma = G @ self.E @ G.T + self.Rk
            K = self.E @ G.T @ np.linalg.inv(Suma)

            angle_innovation  = zik[0][0] - g [0][0]
            angle_innovation_rectified = math.atan2(math.sin(angle_innovation),math.cos(angle_innovation))
            
            innovation = np.array([
                [zik[0][0] - g [0][0]],
                [zik[1][0] - g [1][0]],
                [angle_innovation_rectified]
            ])

            #innovation = zik - g
            print("---ino----")
            print(innovation)
            print("inok")
            print(K @ innovation)
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

    def print_frame(self,frame,frame_name):
        rot = np.array([
            [frame[0][0],frame[0][1],frame[0][2]],
            [frame[1][0],frame[1][1],frame[1][2]],
            [frame[2][0],frame[2][1],frame[2][2]]
        ])
        angles = tf.euler_from_matrix(rot,'sxyz')
        print ("----" + frame_name + "------")
        print("orientation:")
        print("   xrot: " + str(math.degrees(angles[0])))
        print("   yrot: " + str(math.degrees(angles[1])))
        print("   zrot: " + str(math.degrees(angles[2])))
        print("position")
        print("   x: " + str(frame[0][3]))
        print("   y: " + str(frame[1][3]))
        print("   z: " + str(frame[2][3]))

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node terminated by user.")
