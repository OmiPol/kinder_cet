#!/usr/bin/env python3
import rclpy, math, time, tf_transformations
import numpy as np
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import  Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from aruco_opencv_msgs.msg import ArucoDetection
from tf_transformations import quaternion_matrix



class Odometry_node(Node):
    def __init__(self):
        super().__init__("Odometry")
        self.get_logger().info("Odometry node has been started...")
        
        self.namespaceparam = self.declare_parameter('namespace',"")
        self.namespace = self.get_parameter('namespace').value
        
        
        qos_profile = QoSProfile(
         reliability = ReliabilityPolicy.BEST_EFFORT,
         durability = DurabilityPolicy.VOLATILE,
         depth = 1)
        

        #TO DO: Change single topic 2 one topic for each velocity
        self.subR= self.create_subscription(Float32,"/VelocityEncR",self.callback_encR,qos_profile)
        self.subL= self.create_subscription(Float32,"/VelocityEncL",self.callback_encL,qos_profile)
        self.subAr = self.create_subscription(ArucoDetection,"/aruco_detections",self.aruco_callback,1)
        self.t0 = self.get_clock().now()
    
        self.cmdvel = self.create_subscription(Twist,self.namespace+ "/cmd_vel",self.get_req_velocities,1)
        self.watch = self.create_subscription(Clock,"/clock", self.get_time,1)
        self.create_timer(0.1,self.odometry_callback)
        
        #self.pub2=self.create_publisher(Pose,self.namespace+"/odom_position",10)
        self.pub3 = self.create_publisher(Odometry,self.namespace+"/odom",10)
        self.vels = None
        #Variables Puzzlebot
        
        self.pR = 0.0 #Posición llanta derecha
        self.pL = 0.0 #Posición llanta izquierda
        
        
        self.x = 0.0  #Posición en x
        self.y = 0.0  #Posición en y
        
        
        self.theta =0.0 #roll
        self.R = 0.0505 #radio de llanta
        self.D = 0.17  #distancia entre llantas
        
        self.Diam =2*self.R #diametro de la llanta
       
        #Variables de tiempo real
        self.tin =time.time()
        #Simulación
        # self.tin = 0.0 #tiempo inicial
        self.tfin = 0.0 #Tiempo final
        
        self.LinVel = 0.0
        self.AngVel = 0.0
        
        #Ganancias de incertidumbre:
        self.kr = 0.0232
        self.kl = 0.019
        
        self.wR = 0.0
        self.wL = 0.0
        
        #Velocidad lineal requerida por el controlador
        self.vel = 0.0
        
        #Tiempo no real
        self.tiempo = 0.0
        

        #Arucos
        self.Detected_AR = []
        self.arucodict = {
            "0":  [4.0,0.0],
            "1":  [1.6464, -2.6464],
            "2":  [0.925,0.0],
            "3":  [0.0, 4.0],
            "4":  [0.0, -4.0],
            "5":  [-4.0, 0.0],
            "6":  [1.075, 0.0],
            "7":  [-2.31996, 0.4594],
            "8":  [-2.5379, 1.6768],
            "9":  [-1.4336, -1.4504],
            "10": [-0.4928, -1.5174]
        }

        self.trans = np.array([
                [0.0, 0.0, 1.0, 0.07],
                [-1.0, 0.0, 0.0, 0.0],
                [0.0, -1.0, 0.0, 0.08],
                [0.0, 0.0, 0.0, 1.0]
            ])
      
        #Matriz de incertidubmre
        
        self.E = np.array(
            [[1.0, 0.0, 0.0 ],
             [0.0, 1.0, 0.0 ],
             [0.0, 0.0, 1.0 ]])
        
        self.Rk = np.array([
              [0.0105, 0.00001],
              [0.00001, 13.3455]  
            ])
        #mensajes de simulación
     
    def get_time(self,msg):
        sec = msg._clock.sec
        #nano = msg._clock.nanosec
        nano = float(msg._clock.nanosec)/1000000000.0
        self.tiempo = sec + nano
        
    def aruco_callback(self,msg):
        self.Detected_AR = []
        for i in range(len(msg.markers)):
            self.Detected_AR.append(msg[i])
        pass
           
        
    def callback_encR(self,msg):  
      
      self.wR = msg.data
      
      
    def callback_encL(self,msg):  
      
      self.wL = msg.data   
      
      
    def get_req_velocities(self,msg):
        
        self.vel = msg.linear.x
        
        
    def odometry_callback(self):
        
        #Simulación 
        #self.tfin = self.tiempo
        #Real
        self.tfin = time.time()
        msg=Pose()
        
        msg2 = Odometry()
        
        if True:
          
            
            
            delta = self.tfin - self.tin #calcula delta
            
            
            #obbtención de velocidades de las llantas

            
            self.LinVel = (self.R*(self.wL+self.wR))/2
            self.AngVel = (self.R*(self.wR-self.wL))/self.D
            
            #calcula posición y angulo del puzzlebot
            theta = (self.theta) + (delta * self.AngVel)     
            x = (self.x) + (delta * self.LinVel * math.cos(theta))
            y = (self.y) + (delta * self.LinVel * math.sin(theta))
            
            #calcula posición de llantas
            pL = (self.pL) + (delta * self.wL)
            pR = (self.pR) + (delta * self.wR)
            
            #update de datos de posición
            self.x = x
            self.y = y
            self.pL = pL
            self.pR = pR
            self.theta = theta
            
            
            
    
 
            #Constantes

            sigma = np.array([   
                [self.kr*abs(self.wR), 0.0],
                [0.0, self.kl*abs(self.kl)]    
            ])

            

            

            #Paso predictivo
            #Gradiente (Delta al revés)
            delgrad = np.array(
                ((self.R*delta)/(2))*np.array([
                [math.cos(self.theta), math.cos(self.theta)],
                [math.sin(self.theta), math.sin(self.theta)],
                [2/self.D, -2/self.D]
            ]))
            
            Qk = delgrad@sigma@delgrad.T
            
            H = np.array([
                [1.0, 0.0, -delta*self.LinVel*math.sin(self.theta)],
                [0.0, 1.0, delta*self.LinVel*math.cos(self.theta)],
                [0.0, 0.0, 1.0]
            ])
            
            self.E = H@self.E@H.T + Qk


            #Paso de correción
            if len(self.Detected_AR) != 0:
                
                puzzle_trans = np.array([
                    [math.cos(self.theta), -math.sin(self.theta), 0.0, self.x],
                    [math.sin(self.theta), math.cos(self.theta), 0.0, self.y],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0]
                ])

                estado = np.array([
                    [self.x],
                    [self.y],
                    [self.theta]
                ])

                for i in range(len(self.Detected_AR)):
                    id = self.arucodict[self.Detected_AR[i].id]



                    vector = np.array([
                        [self.Detected_AR[i].pose.position.x],
                        [self.Detected_AR[i].pose.position.y],
                        [self.Detected_AR[i].pose.position.z],
                        [1]
                    ])

                    rotation = quaternion_matrix(self.Detected_AR[i].orientation)

                    reference_aruco = np.array([
                        [rotation[0,0],rotation[0,1],rotation[0,2],vector[0][0]],
                        [rotation[1,0],rotation[1,1],rotation[1,2],vector[0][1]],
                        [rotation[2,0],rotation[2,1],rotation[2,2],vector[0][2]],
                        [0.0, 0.0, 0.0, 1.0]
                    ])

                    #Aruco con relación al puzzlebot     
                    ar_puzzle = self.trans@reference_aruco

                    yaw = math.atan2(ar_puzzle[1][0], ar_puzzle[0][0])

                    deltax = self.arucodict[id][0] - self.x
                    deltay = self.arucodict[id][1] - self.y

                    g = np.array([
                        [math.hypot(deltax, deltay)],
                        [math.atan2(deltay, deltax)-self.theta]  

                    ])
                    
                    G = np.array([
                        [-deltax/math.hypot(deltax,deltay),  -deltay/math.hypot(deltax,deltay),    0.0],
                        [deltay/(deltax ** 2 + deltay ** 2), -deltax/(deltax ** 2 + deltay ** 2), -1  ]
                    ])

                    #PASO 1 Calcular la ganancia de Kalman
                    inter = G @ self.E @ G.T + self.Rk
                    Kk = self.E @ G.T @ np.linalg.inv(inter)


                    #Correción del estado
                    zik = np.array([
                        [math.hypot(ar_puzzle[0][0], ar_puzzle[1][0])],
                        [yaw]
                    ])

                    error_pred = zik - g
                    estado = estado + Kk @ error_pred

                    #Correción de incertidumbre
                    self.E = self.E - Kk 
                 

            
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        #self.pub2.publish(msg)
        
        msg2.header.frame_id = "world"
        msg2.child_frame_id = self.namespace + "/base_footprint"
        msg2.pose.pose.position.z = self.R
       # msg2.pose.covariance = [self.E[0][0], self.E[0][1], 0.0, 0.0, 0.0, self.E[0][2],
        #                        self.E[1][0], self.E[1][1], 0.0, 0.0, 0.0, self.E[1][2],
        #                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        #                        self.E[2][0], self.E[2][1], 0.0, 0.0, 0.0, self.E[2][2]]
        
        msg2.pose.covariance = [0.0]*36
        msg2.pose.covariance[0] = self.E[0][0]#5.16 ##xx  
        msg2.pose.covariance[1] = self.E[0][1]#-3.16 #xy
        msg2.pose.covariance[5] = self.E[0][2]#0.0 #xt
        
        msg2.pose.covariance[6] = self.E[1][0]#-3.6 #yx
        msg2.pose.covariance[7] = self.E[1][1]#8.5 #yy
        msg2.pose.covariance[11] = self.E[1][2]#0 #yt
        
        msg2.pose.covariance[30] = self.E[2][0]#0 #tx
        msg2.pose.covariance[31] = self.E[2][1]#0 #ty
        msg2.pose.covariance[35] = self.E[2][2]#0.5 #tt
        
        msg2.pose.pose.position.x = self.x
        msg2.pose.pose.position.y = self.y
        
        q = tf_transformations.quaternion_from_euler(0.0, 0.0,self.theta)
        msg2.pose.pose.orientation.x = q[0]
        msg2.pose.pose.orientation.y = q[1]
        msg2.pose.pose.orientation.z = q[2]
        msg2.pose.pose.orientation.w = q[3]
        
        self.pub3.publish(msg2)
        self.tin = self.tfin#actualiza valor de intervalo

        print("X: " +str(self.x))
        print("Y: " + str(self.y))
            

def main(args=None):
    rclpy.init(args=args)
    nodeh = Odometry_node()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user...")

if __name__ == "__main__":
    main()