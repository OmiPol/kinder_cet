#!/usr/bin/env python3
import rclpy, math, tf_transformations, time, random
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_interfaces.msg import WheelV



class Puzzle_move(Node):
    def __init__(self):
        super().__init__("Puzzle_move")
        self.get_logger().info("Move my robot node has been started...")
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.pub_tf = TransformBroadcaster(self)
        self.pos = self.create_publisher(Pose,"/real/position",10)
        self.pub = self.create_publisher(JointState,'/real/joint_states',10)
        self.sub= self.create_subscription(Odometry,"/ground_truth",self.get_position,1) 
        
       
        
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

        
        #mensajes de simulación
        

    def get_position(self,msg):
        angles =tf_transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                  msg.pose.pose.orientation.y,
                                                  msg.pose.pose.orientation.z,
                                                  msg.pose.pose.orientation.w])
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = angles[2]

      
       
    
    def timer_callback(self):
        msg = TransformStamped()
        t = (self.get_clock().now()-self.t0).nanoseconds/1e9
        
        #Control de posición y rotación del puzzlebot
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = "/real/base_footprint"
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
        js.name = ["wheel_right_joint","wheel_left_joint"]
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