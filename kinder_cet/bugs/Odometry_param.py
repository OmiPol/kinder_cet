#!/usr/bin/env python3
import rclpy, math, time, tf_transformations
import numpy as np
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import  Twist
from nav_msgs.msg import Odometry
from my_interfaces.msg import WheelV
from std_msgs.msg import Float32
from sympy import symbols, Abs, sin, cos, Matrix
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock



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
        self.tin = 0.0 #tiempo inicial
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
        
      
        #Matriz de incertidubmre
        
        self.E = np.array([[1.0, 0.0, 0.0 ],
             [0.0, 1.0, 0.0 ],
             [0.0, 0.0, 1.0 ]])
        
        #mensajes de simulación
     
    def get_time(self,msg):
        sec = msg._clock.sec
        #nano = msg._clock.nanosec
        nano = float(msg._clock.nanosec)/1000000000.0
        self.tiempo = sec + nano
        
           
        
    def callback_encR(self,msg):  
      
      self.wR = msg.data
      
      
    def callback_encL(self,msg):  
      
      self.wL = msg.data   
      
      
    def get_req_velocities(self,msg):
        
        self.vel = msg.linear.x
        
        
    def odometry_callback(self):
        
        self.tfin = self.tiempo
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
            
            
            #CALCULO DE MATRIZ Q = Lambda * sigma * Lambda transpuesta
            
            #Q = [[(0.25 * self.R**2 * delta**2 * (self.kl * abs(wL)+ self.kr * abs(wR))*math.cos(self.theta)**2), (0.125 * self.R**2 * delta**2 * (self.kl * abs(wL)+ self.kr * abs(wR))*math.sin(2*self.theta)),(self.R**2 * delta**2 * (-self.kl * abs(wL)+ self.kr * abs(wR))*math.cos(self.theta))/2*self.L],
            #     [(0.125 * self.R**2 * delta**2 * (self.kl * abs(wL)+ self.kr * abs(wR))*math.sin(2*self.theta)),(0.25 * self.R**2 * delta**2 * (self.kl * abs(wL)+ self.kr * abs(wR))*math.sin(self.theta)**2),(self.R**2 * delta**2 * (-self.kl * abs(wL)+ self.kr * abs(wR))*math.sin(self.theta))/2*self.L],
            #     [(self.R**2 * delta**2 * (-self.kl * abs(wL)+ self.kr * abs(wR))*math.cos(self.theta))/2*self.L,(self.R**2 * delta**2 * (-self.kl * abs(wL)+ self.kr * abs(wR))*math.sin(self.theta))/2*self.L,(self.R**2 * delta**2 * (self.kl * abs(wL)+ self.kr * abs(wR)))/self.L**2]]
            
            #SAN CHAT
            
            # Definición de variables simbólicas

            #La otra cosa
            
    
 
            #print("lo logre")
            sigma = np.array([   
                [self.kr*abs(self.wR), 0.0],
                [0.0, self.kl*abs(self.kl)]    
            ])
            
            delgrad = np.array(((self.R*delta)/(2))*np.array([
                [math.cos(self.theta), math.cos(self.theta)],
                [math.sin(self.theta), math.sin(self.theta)],
                [2/self.D, -2/self.D]
            ]))
            #print("voa hacer la sumacion")
            
            Qk = delgrad@sigma@delgrad.T
            
            H = np.array([
                [1.0, 0.0, -delta*self.LinVel*math.sin(self.theta)],
                [0.0, 1.0, delta*self.LinVel*math.cos(self.theta)],
                [0.0, 0.0, 1.0]
            ])
            
            self.E = H@self.E@H.T + Qk
            
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
            

def main(args=None):
    rclpy.init(args=args)
    nodeh = Odometry_node()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user...")

if __name__ == "__main__":
    main()