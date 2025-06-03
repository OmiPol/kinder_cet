#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from turtlesim.msg import Pose

class TestLidarClass(Node):
    def __init__(self):
        super().__init__("Test_Lidar_node")
        self.get_logger().info("Test_lidar node has been started...")
        self.create_timer(0.2,self.stateMachine)
        self.create_subscription(LaserScan, '/scan',self.lidar_callback,1)
        self.state_sub = self.create_subscription(String, "/state",self.recieve_state,1)
        self.target_pub = self.create_publisher(Pose,"/target",1)
        self.imperative_pub = self.create_publisher(String,"/imperative",1)
        self.command_sub = self.create_subscription(String,"/command",self.recieve_command)
        
        self.command = "stop"
        self.bug_state = "stop_robot"
        self.loading_station = [1.0, 0.0]
        self.alignment_point = [0.5, 0.0]
        self.discharging_station = [-1.0, 0.0]
        self.home = [0.0,0.0]


        self.high_state = "stop"
        self.next_state = "stop"


        self.point_published = True


    def recieve_state(self,msg):
        self.bug_state = msg.data
    def recieve_command(self,msg):
        self.command = msg.data

    def publish_point(self,point, imp):
        if self.point_published == False:
            msg = Pose()
            msg.x = point[0]
            msg.y = point[1]
            self.target_pub.publish(msg)
            self.point_published = True

            impertive = String()
            impertive.data = imp
            self.imperative_pub.publish(impertive)
            

    
    def stateMachine(self):

        if self.high_state == "stop" and self.command == "go":
            self.next_state = "go_to_align"
        
        elif self.high_state == "go_to_align" and self.bug_state == "stop_robot":
            self.next_state = "grab_box"
        
        elif self.high_state == "grab_box" and self.bug_state == "stop_robot":
            self.next_state = "go_to_drop"
        
        elif self.high_state == "go_to_drop" and self.bug_state == "stop_robot":
            self.next_state = "go_home"

        elif self.high_state == "go_home" and self.bug_state == "stop_robot":
            self.next_state = "stop"
            self.command = "stop"
        
        if self.command == "stop": self.next_state = "stop"
            


        if self.high_state == "stop":
                msg = String()
                msg.data = "Stop"
                self.imperative_pub(msg)

        

        elif self.high_state == "go_to_align":
            self.publish_point(self.alignment_point,"Go")

        elif self.high_state == "go_to_align":
            self.publish_point(self.alignment_point,"Go")

        elif self.high_state == "grab_box":

            self.publish_point(self.loading_station,"Grab")

        elif self.high_state == "go_to_drop":
            self.publish_point(self.discharging_station,"Go")

        elif self.high_state == "go_home":
            self.publish_point(self.home,"Go")

         



        if self.high_state != self.next_state:
            self.point_published = False
            self.high_state = self.next_state

        


        

    
 
 
        
def main(args=None):
    rclpy.init(args=args)
    nodeh = TestLidarClass()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user...")

if __name__ == "__main__":
    main()