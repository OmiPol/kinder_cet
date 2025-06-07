import rclpy,math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class Generator(Node):
    def __init__(self):
     super().__init__("Path_gen")
     self.get_logger().info("Path Generator node has been started...")
     #Creación de subscriptores y publicadores
     self.pub = self.create_publisher(Pose, "/Path", 1)
     self.time = self.create_timer(0.1,self.gen_path)
     #self.statemachine = self.create_timer(0.1,self.sm)
     self.t0 = self.get_clock().now()
     self.t=0.0
     
     #state machine
     self.state = 1
     
     #interval
     self.interval = 18.0

    def gen_path(self):
        msg = Pose()
        time = min(self.t/self.interval,1.0)
        #print(time)
        
        msg.x = -1.0
        msg.y = -1.0
        
        self.pub.publish(msg)
 
def main(args=None):
   rclpy.init(args=args) #inicializa
   nodeh = Generator()
   
   try: rclpy.spin(nodeh)
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node killed unu")
if __name__=="__main__":
   main()
