#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

#from my_interfaces.msg import WheelV



class Odometry(Node):
    def __init__(self):
        super().__init__("Odometry")
        self.get_logger().info("Odometry node has been started...")
        
        qos_profile = QoSProfile(
         reliability = ReliabilityPolicy.BEST_EFFORT,
         durability = DurabilityPolicy.VOLATILE,
         depth = 1)
        

        #TO DO: Change single topic 2 one topic for each velocity
        self.subR= self.create_subscription(Float32,"/VelocityEncR",self.callback_encR,qos_profile)
        self.subL= self.create_subscription(Float32,"/VelocityEncL",self.callback_encL,qos_profile)
      
        self.create_timer(0.1,self.odometry_callback)
        
        self.pub2=self.create_publisher(Pose,"/odom_position",10)
        
        self.t0 = self.get_clock().now()
        
        
        #Variables Puzzlebot
        
        self.x = 0.0  #Posición en x
        self.y = 0.0  #Posición en y       
        self.theta =0.0 #roll
        
        # TO DO: verify distances of puzzlebot
        self.R = 0.0505 #radio de llanta
        self.D = 0.186 #distancia entre llantas
        
        self.Diam =2*self.R #diametro de la llanta
       
        #Variables de tiempo real
        self.tin = time.time() #tiempo inicial
        self.tfin = 0.0 #Tiempo final
        
        self.LinVel = 0.0
        self.AngVel = 0.0
        
        #variables de mensaje
        self.wR=0.0
        self.wL=0.0
        

    def callback_encR(self,msg):  
      
      self.wR = msg.data
      
      
    def callback_encL(self,msg):  
      
      self.wL = msg.data       
        
    def odometry_callback(self):
        
        self.tfin = time.time()
        msg=Pose()
                      
        delta = self.tfin - self.tin #calcula delta
        
        #calcula velocidades lineales y angulares       
        self.LinVel = (self.R*(self.wL+self.wR))/2
        self.AngVel = (self.R*(self.wR-self.wL))/self.D
        
        #calcula posición y angulo del puzzlebot
        theta = (self.theta) + (delta * self.AngVel)     
        x = (self.x) + (delta * self.LinVel * math.cos(theta))
        y = (self.y) + (delta * self.LinVel * math.sin(theta))
        
       #update de datos de posición
        self.x = x
        self.y = y
        self.theta = theta
        
        #creación de mensaje de posición
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        self.pub2.publish(msg)
        
        #actualiza valor de intervalo
        self.tin = self.tfin
            

def main(args=None):
    rclpy.init(args=args)
    nodeh = Odometry()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user...")

if __name__ == "__main__":
    main()