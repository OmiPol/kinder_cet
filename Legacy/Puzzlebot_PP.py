#! /usr/bin/env python3
import rclpy,math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Controler(Node):
   def __init__(self):
    super().__init__("Puzzle_PosePursuit")
    self.get_logger().info("Pose Pursuit node has been started...")
    #Creación de subscriptores y publicadores
    self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
    self.subpath= self.create_subscription(Pose,"/Path",self.get_path,1)
    self.subpos = self.create_subscription(Pose,"/odom_position",self.get_position,1) #cambio
    self.time = self.create_timer(0.1,self.move)
    
   
    
    self.actualPose = Pose()
    #PMAx velocity parameters
    self.linMax = 0.1 #m/s
    self.angMax = 0.3 #rad/s
    
    #Error tolerances
    self.linTol = 0.05 #mts
    self.angTol = 0.06981 #rads
    
    #Desired point
    self.Xdes = 0.0 #desired x
    self.Ydes = 0.0 #desired Y
    
    #control paramters
    self.angP = 2.0  #proportional of angular error
    self.linP = 1.8 #proportional of linear error
    
    #flags gotopoint/angle
    self.angR = False #angle reached
    self.linR = False #linear reached

   def get_path(self,msg):
      self.Xdes = msg.x
      self.Ydes = msg.y
      
   def get_position(self,msg):
      self.actualPose = msg
      
   def move(self):  #pose pursuit doble proporcional
      msg = Twist()
      if self.actualPose is not None:
         #Error calculation
         diff_x, diff_y = self.Xdes - self.actualPose.x, self.Ydes - self.actualPose.y
         e_dist = math.sqrt(diff_x**2 + diff_y**2)
         #print(e_dist)
         angle_diff = np.arctan2(diff_y,diff_x)
         angle_error = np.arctan2(math.sin(angle_diff-self.actualPose.theta),math.cos(angle_diff-self.actualPose.theta))
         print("edist: " + str(e_dist) + "eang: " + str(angle_error))

            #Checks if current position is within tolerance
            #if it is, stop movment, if it is not, continue movement
         if abs(e_dist) > self.linTol:
            #saturation of max velocities
             msg.linear.x = max(min(self.linP * e_dist,self.linMax),-self.linMax)
             msg.angular.z = max(min(self.angP * angle_error,self.angMax),-self.angMax)  
         else:
            msg.linear.x = 0.0
            msg.angular.z =0.0
         
         #publishing message
         self.pub.publish(msg)
         
   def move2(self): #go to point go to angle 
      msg = Twist()
      if self.actualPose is not None:
         #Error calculation
         diff_x, diff_y = self.Xdes - self.actualPose.x, self.Ydes - self.actualPose.y
         e_dist = math.sqrt(diff_x**2 + diff_y**2)
         #print(e_dist)
         angle_diff = np.arctan2(diff_y,diff_x)
         angle_error = np.arctan2(math.sin(angle_diff-self.actualPose.theta),math.cos(angle_diff-self.actualPose.theta))
         print("edist: " + str(e_dist) + "eang: " + str(angle_error))

            #Checks if current position is within tolerance
            #if it is, stop movment, if it is not, continue movement
         if abs(angle_error) > self.angTol :
            #saturation of max velocities
             #msg.linear.x = max(min(self.linP * e_dist,self.linMax),-self.linMax)
             msg.angular.z = max(min(self.angP * angle_error,self.angMax),-self.angMax)  
         elif abs(e_dist) > self.linTol:
            msg.linear.x = max(min(self.linP * e_dist,self.linMax),-self.linMax)
            #msg.angular.z = max(min(self.angP * angle_error,self.angMax),-self.angMax) 
         else:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            
         #publishing message
         self.pub.publish(msg)

def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = Controler()
   
   try: rclpy.spin(nodeh)
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
