#!/usr/bin/env python3
import rclpy, math, tf_transformations, time, random
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_interfaces.msg import WheelV



class Puzzle_move(Node):
    def __init__(self):
        super().__init__("Puzzle_move")
        self.get_logger().info("Move my robot node has been started...")
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer2 = self.create_timer(0.1, self.publish_position)
        self.pub_tf = TransformBroadcaster(self)
        self.pos = self.create_publisher(Pose,"/sim/position",10)
        self.pub = self.create_publisher(JointState,'/sim/joint_states',10)
        self.sub= self.create_subscription(Twist,"/cmd_vel",self.get_velocity,1) 
        
        
        self.create_timer(0.1,self.math_callback)
        
        self.t0 = self.get_clock().now()
        
        self.vels = None
        #Variables Puzzlebot
        
        self.pR = 0.0 #Posición llanta derecha
        self.pL = 0.0 #Posición llanta izquierda
        
        
        self.x = 0.0  #Posición en x
        self.y = 0.0  #Posición en y
        
        
        self.theta =0.0 #roll
        self.R = 0.0505 #radio de llanta
        self.D = 0.186 #distancia entre llantas
        
        self.Diam =2*self.R #diametro de la llanta
       
        #Variables de tiempo real
        self.tin = time.time() #tiempo inicial
        self.tfin = 0.0 #Tiempo final
        
        #mensajes de simulación
        
        
    def get_velocity(self,msg):
        
        self.vels=msg
        
        
    def publish_position(self):    
        #enviar mensaje de posición
        msg=Pose()
        msg.x = self.x
        msg.y = self.y
        msg.theta = self.theta
        self.pos.publish(msg)
        
        
    def math_callback(self):
        
        self.tfin = time.time()

        if self.vels is not None:
          
            
            
            delta = self.tfin - self.tin #calcula delta
            
            
            #obbtención de datos de velocidad deseada
            velL = self.vels.linear.x #velocidad linear en x
            velA = self.vels.angular.z #Velocidad angular en z
            
            #calculo de velocidades de motores con base en diferencial drive
            
            wL = (2*velL - self.D*velA)/self.Diam
            wR = (2*velL + self.D*velA)/self.Diam
            
            #calcula posición y angulo del puzzlebot
            theta = (self.theta) + (delta * velA)     
            x = (self.x) + (delta * velL * math.cos(theta))
            y = (self.y) + (delta * velL * math.sin(theta))
            
            #calcula posición de llantas
            pL = (self.pL) + (delta * wL)
            pR = (self.pR) + (delta * wR)
            
            #update de datos de posición
            self.x = x
            self.y = y
            self.pL = pL
            self.pR = pR
            self.theta = theta 


            
        self.tin = self.tfin#actualiza valor de intervalo
             
    
    def timer_callback(self):
        msg = TransformStamped()
        t = (self.get_clock().now()-self.t0).nanoseconds/1e9
        
        #Control de posición y rotación del puzzlebot
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "/sim/base_footprint"
        msg.transform.translation.x = self.x
        msg.transform.translation.y = self.y
        msg.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0.0, 0.0,self.theta)
        msg.transform.rotation.x = q[0]
        msg.transform.rotation.y = q[1]
        msg.transform.rotation.z = q[2]
        msg.transform.rotation.w = q[3]
        self.pub_tf.sendTransform(msg)
        
        #Move Joints
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ["base_to_r_wheel","base_to_l_wheel"]
        js.position = [self.pR,self.pL]
        self.pub.publish(js)       

def main(args=None):
    rclpy.init(args=args)
    nodeh = Puzzle_move()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user...")

if __name__ == "__main__":
    main()