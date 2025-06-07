#!/usr/bin/env python3
import rclpy
import numpy as np
import time
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import csv
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        qos_profile = QoSProfile(
        reliability = ReliabilityPolicy.BEST_EFFORT,
        durability = DurabilityPolicy.VOLATILE,
        depth = 1)
        

        #TO DO: Change single topic 2 one topic for each velocity
        self.subR= self.create_subscription(Float32,"/VelocityEncR",self.callback_encR,qos_profile)
        self.subL= self.create_subscription(Float32,"/VelocityEncL",self.callback_encL,qos_profile)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.param = self.declare_parameter("vel",0.0)
        
        self.timer = self.create_timer(0.05,self.data_callback,)
        self.get_logger().info('Nodo Listener iniciado, esperando mensajes...')
        
        #Variables Puzzlebot
        self.R = 0.0505 #radio de llanta
        
        #Variables de tiempo real
        self.r_vel = 0.0
        self.l_vel = 0.0
        ###################
        self.des_vel = self.param.value
        self.dr_vel, self.dl_vel = [self.des_vel,self.des_vel]
        
        
        #Variables de CSV
        self.test_time = 12.0    
        self.file_name = 'encoder_data_1_6.csv'
        self.encoder_r_values = []
        self.encoder_l_values = []
        self.desired_r_values = []
        self.desired_l_values = []
        self.timestamp = []

        self.start_time = time.time()

    
    def callback_encR(self,msg):  
      
      self.r_vel = msg.data
      
      
    def callback_encL(self,msg):  
      
      self.l_vel = msg.data   
    
    def data_callback(self):
        elapsed_time = time.time()- self.start_time 

        if elapsed_time >= self.test_time:
            msg = Twist()
            linear_vel = 0.0
            msg.linear.x = linear_vel
            self.pub.publish(msg)
            
            
            #self.get_logger().info('CREATING CS FILE FOR ENCODERS DATA.')
            #self.save_to_csv()
            self.calcstd()
            #print("Node (presumably) killing itself")
            self.destroy_node()
            #print("If you are reading this, the node is still alive")
            return 

        
        msg = Twist()
        linear_vel = (self.R*(self.dr_vel+self.dl_vel))/2
        msg.linear.x = linear_vel
        self.pub.publish(msg)
        if elapsed_time > 2.0:
            #print(elapsed_time)
            self.encoder_r_values.append(self.r_vel)
            self.encoder_l_values.append(self.l_vel)
            self.desired_r_values.append(self.dr_vel)
            self.desired_l_values.append(self.dl_vel)
            self.timestamp.append(round(elapsed_time,3))

    def calcstd(self):
        desvr = np.std(self.encoder_r_values)
        desvl = np.std(self.encoder_l_values)
        print("Wheel Volocity: "  + str(self.des_vel))
        print("Lin. Vel: " + str((self.R*(self.dr_vel+self.dl_vel))/2))
        print("-----------")
        print("Std. dev r: " + str(desvr))
        print("Std. dev l: " + str(desvl))
        print("-----------")
        print("Std. dev² r: " + str(desvr**2))
        print("Std. dev² l: " + str(desvl**2))
        

        


    def save_to_csv(self):
        try:
            
            with open(self.file_name, 'w', newline='') as file: 
                writer = csv.writer(file)
                writer.writerow(['Tiempo', 'Encoder_r', 'Encoder_l', 'Desired_r', 'Desired_l'])

                for t, r, l, dr, dl in zip(self.timestamp, self.encoder_r_values, self.encoder_l_values, self.desired_r_values, self.desired_l_values):

                    writer.writerow([t, r, l, dr, dl])
                self.get_logger().info("DATA HAVE BEEN STORED SUCCESFULLY")

        except Exception as e:
            self.get_logger().info("ERROR AT CREATING THE CSV FILE")
            self.get_logger().error(e)

    

def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()
    try: rclpy.spin(node)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user...")

if __name__ == '__main__':
    main()