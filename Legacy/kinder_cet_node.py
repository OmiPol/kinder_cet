#! /usr/bin/env python3
import rclpy,time,math,sys
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Controler(Node):
   def __init__(self):
      
      super().__init__("Controler_CL") #declara el nodo
      self.get_logger().info("Node initiated")
      self.pub = self.create_publisher(Twist, "turtle1/cmd_vel", 1)
      self.sub= self.create_subscription(Pose,"turtle1/pose",self.callback_turtle_pose,1)
      
      self.pose = None
      self.Kv = 2.0
      rclpy.spin_once(self)
      
      self.lista = [(0,0),(5,7),(7,1),(1,1)]
      self.L = 0.66
      self.multi = 0.0  
   def callback_turtle_pose(self,msg):
      self.pose = msg
      
   def go_to_point(self,target_x,target_y):
      msg = Twist()
      while True:
         if self.pose is not None:
            Dx, Dy = target_x-self.pose.x, target_y-self.pose.y
            e_dist = math.sqrt(Dx**2 + Dy**2)
            
            if abs(e_dist) <= 0.2: self.pub.publish(Twist()); print("Target reached") ;break
            e_ang = math.atan2(Dy,Dx) - self.pose.theta
            a_ang = math.atan2(math.sin(e_ang),math.cos(e_ang))
            msg.linear.x = 1.0
            msg.angular.z = self.Kv * a_ang
            self.pub.publish(msg)
            rclpy.spin_once(self) #Checa los mensajes
            time.sleep(0.02)
         
   def go_to_angle(self,target_theta):
      msg = Twist()
      while True:
         if self.pose is not None:
            e_ang = target_theta - self.pose.theta
            a_ang = math.atan2(math.sin(e_ang),math.cos(e_ang))
            if abs(a_ang) <= 0.05: 
               self.pub.publish(Twist())
               print("Target reached")
               break
            #print(a_ang)
            msg.angular.z = max(min(self.Kv * a_ang,0.45),-0.45)
            self.pub.publish(msg)
            rclpy.spin_once(self) #Checa los mensajes
            time.sleep(0.02)
   
   def pose_pursuit(self,target_x,target_y):
      msg = Twist()
      while True:
         if self.pose is not None:
            diff_x, diff_y = target_x - self.pose.x, target_y - self.pose.y
            e_dist = math.sqrt(diff_x**2 + diff_y**2)
            if abs(e_dist) < 0.025: 
               self.pub.publish(Twist()); 
               #print("Target reached");
               break
            sq = math.sin(self.pose.theta)
            cq = math.cos(self.pose.theta)
            
            w = ((diff_y * cq) - (diff_x * sq)) / self.L
            v = (diff_x + (self.L * w * sq)) / cq 
            msg.linear.x = max(min(v,0.4),-0.4) #lineal v
            msg.angular.z = max(min(w,0.4),-0.4) # anular w
            self.pub.publish(msg)
            rclpy.spin_once(self) #checa mensajes
            time.sleep(0.02) #Ahorra ciclos de procesamiento
  	     #print("cerca cerquita vamos")
           
   def angleandpursuit(self,target_x,target_y):
      print("Target: x: " + str(target_x) + " y: " + str(target_y) )
      actual_x = self.pose.x
      actual_y = self.pose.y

      errx = target_x-actual_x
      erry = target_y-actual_y

      angle = math.atan2(erry,errx)
      self.go_to_angle(angle)
      time.sleep(0.5)
      self.acc_pose_pursuit(target_x,target_y)
      print("Target Reached")
      time.sleep(1)

   def acc_go_to_angle(self,target_theta):
         msg = Twist()
         while True:
            if self.pose is not None:
               e_ang = target_theta - self.pose.theta
               a_ang = math.atan2(math.sin(e_ang),math.cos(e_ang))
               if abs(a_ang) <= 0.05: self.pub.publish(Twist()); print("Target reached") ;break
               #print(a_ang)
               msg.angular.z = max(min(self.Kv * a_ang,0.45),-0.45)
               self.pub.publish(msg)
               rclpy.spin_once(self) #Checa los mensajes
               time.sleep(0.02)
   
   def acc_pose_pursuit(self,target_x,target_y):
      msg = Twist()
      while True:

         

         if self.pose is not None:
            diff_x, diff_y = target_x - self.pose.x, target_y - self.pose.y
            e_dist = math.sqrt(diff_x**2 + diff_y**2)


            if abs(e_dist) < 0.025: 
               
               self.pub.publish(Twist()); 
               #print("Target reached");
               break
            
            sq = math.sin(self.pose.theta)
            cq = math.cos(self.pose.theta)
            

            if(e_dist > 0.2 and self.multi < 1):
               self.multi +=  0.005
               #print("Acelerando")
               #print(self.multi)
            
            if(e_dist <= 0.2 and self.multi > 0.4 ):
               self.multi-=0.01
               #print("frenando")
               #print(self.multi)




            w = self.multi*(((diff_y * cq) - (diff_x * sq)) / self.L)
            v = self.multi*((diff_x + (self.L * w * sq)) / cq )


            msg.linear.x = max(min(v,0.8),-0.8) #lineal v
            msg.angular.z = max(min(w,0.4),-0.4) # anular w
            self.pub.publish(msg)
            rclpy.spin_once(self) #checa mensajes
            time.sleep(0.02) #Ahorra ciclos de procesamiento
  	     #print("cerca cerquita vamos")

   def circle_x(self,radius,target_x): #radio de giro, objetivo de x
      msg = Twist()
      
      w = 0.5 #velocidad angular //dterminar signo de velocidad angular dependiendo de dirección
      v = radius*w #calculo de velocidad linear
      while True:
         if self.pose is not None:
            diff_x = target_x - self.pose.x 
               


            if abs(diff_x) > 0.025: 
               msg.linear.x = v
               msg.angular.z = w
               self.pub.publish(msg)
               rclpy.spin_once(self) #checa mensajes
               time.sleep(0.02) #Ahorra ciclos de procesamiento realizar este proceso a 50hz
            else:
               print("finished circle")
               break      
       
   def traj5(self,p0,pf,v0,vf,t0,tf):
      matrix1=np.array([[1,t0,pow(t0,2),pow(t0,3),pow(t0,4),pow(t0,5)],
                       [1,tf,pow(tf,2),pow(tf,3),pow(tf,4),pow(tf,5)],
                       [0,1,2*t0,3*pow(t0,2),4*pow(t0,3),5*pow(t0,4)],
                       [0,1,2*tf,3*pow(tf,2),4*pow(tf,3),5*pow(tf,4)],
                       [0,0,2,6*t0,12*pow(t0,2),20*pow(t0,3)],
                       [0,0,2,6*tf,12*pow(tf,2),20*pow(tf,3)]])
      matrix2 = np.array([[p0],
                         [pf],
                         [v0],
                         [vf],
                         [0],
                         [0]])
      Inv = np.linalg.inv(matrix1)
      
      poli = np.array([0,0,0,0,0,0])
      
      print(poli)
      
      np.matmul(Inv,matrix2,out=poli)
      
       
   def principal(self):
     print("begin...")
     self.traj5(-1.5708,0.6750,0.07/0.8,0.09/0.8,0,7)
     time.sleep(2)
     self.angleandpursuit(7.0,5.555)#primera linea
     self.circle_x(0.8,7.5) #primer radio
     self.angleandpursuit(8,6.0983) #segunda linea
     self.circle_x(0.3,7.8158)#segundoradio
     self.angleandpursuit(5.6,6.6319)#tercerlinea
     self.circle_x(0.32,5.3)#tercer radio
     self.angleandpursuit(5.3,5.9)
     self.circle_x(0.32,5.6)
     
     #self.angleandpursuit(8.0,8.0)
     #self.angleandpursuit(8.0,2.0)
     self.go_to_angle(0)
     print("finished")
     
     
     #self.pose_pursuit(1,0)
     #self.go_to_angle(math.pi/2)
     #self.pose_pursuit(1,1)
     #self.go_to_angle(math.pi)
     #self.pose_pursuit(0,1)
     #self.go_to_angle(-math.pi/2)
     #self.pose_pursuit(0,0)
     #self.go_to_angle(0)
     #self.pose_pursuit(0.5,0.5)
     
     
     
      
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = Controler()
   
   try: nodeh.principal()
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
