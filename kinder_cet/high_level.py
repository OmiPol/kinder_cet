#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from turtlesim.msg import Pose

class HighSM(Node):
    def __init__(self):
        super().__init__("High_State_Machine")
        self.get_logger().info("High SM node has been started...")
        self.create_timer(1.0,self.stateMachine)
        self.state_sub = self.create_subscription(String, "/state",self.recieve_state,1)
        self.target_pub = self.create_publisher(Pose,"/target",1)
        self.imperative_pub = self.create_publisher(String,"/imperative",1)
        self.command_sub = self.create_subscription(String,"/command",self.recieve_command,1)
        self.high_state_pub = self.create_publisher(String,"high_state",1)
        self.servo_pub = self.create_publisher(String,"/ServoPos",1)
        
        self.command = "stop"
        self.bug_state = "stop_robot"
        self.last_bug_state = "stop_robot"
        self.finish_task = False


        self.loading_station = [1.01, 0.0]
        self.alignment_point = [0.5, 0.0]
        self.discharging_station = [-1.0, 0.0]





        self.home = [0.0,0.0]


        self.high_state = "stop"
        self.next_state = "stop"


        self.point_published = True

        self.t0 = 0.0
        self.now = 0.0
        self.tact = 0.0
        


    def recieve_state(self,msg):
        self.bug_state = msg.data
        if self.bug_state == "stop_robot" and self.bug_state != self.last_bug_state:
            self.finish_task = True
        self.last_bug_state = self.bug_state

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
        msg = String()
        msg.data = "bug: " + self.bug_state
        self.high_state_pub.publish(msg)
        self.now = self.get_clock().now().seconds_nanoseconds()
        self.tact = self.now[0] + self.now[1] * 1e-9
           
        if self.high_state == "stop" and self.command == "go":
            self.next_state = "go_to_align"
        
        elif self.high_state == "go_to_align" and self.finish_task == True:
            self.next_state = "grab_box"
        
        elif self.high_state == "grab_box" and self.finish_task == True:
            self.next_state = "finish_grab"
            self.t0 = self.now[0] + self.now[1] * 1e-9

        elif self.high_state == "finish_grab" and (self.tact - self.t0) > 3.0:
            self.next_state = "go_to_drop"
        
        elif self.high_state == "go_to_drop" and self.finish_task == True and (self.tact - self.t0) > 9.0:
            self.next_state = "go_back"
            self.t0 = self.now[0] + self.now[1] * 1e-9

        elif self.high_state == "go_back" and (self.tact - self.t0) > 3.0:
            self.next_state = "go_home" 

        elif self.high_state == "go_home" and self.finish_task == True:
            self.next_state = "stop"
            self.command = "stop"
        
        self.finish_task = False

        msg = String()
        if self.high_state == "stop":

                msg.data = "Stop"
                self.imperative_pub.publish(msg)


        elif self.high_state == "go_to_align":
            self.publish_point(self.alignment_point,"Go")
            self.servo_pub.publish(String(data = "high"))

        elif self.high_state == "grab_box":

            self.publish_point(self.loading_station,"Grab")
            self.servo_pub.publish(String(data = "low"))

        elif self.high_state == "finish_grab":
            self.publish_point(self.discharging_station,"Fin")
            self.servo_pub.publish(String(data = "low"))

        elif self.high_state == "go_to_drop":
            self.publish_point(self.discharging_station,"Go")
            self.servo_pub.publish(String(data = "high"))

        elif self.high_state =="go_back":
            self.imperative_pub.publish(String(data = "Back"))
            self.servo_pub.publish(String(data = "low"))
        
        elif self.high_state == "go_home":
            self.publish_point(self.home,"Go")
            self.servo_pub.publish(String(data = "high"))


        msg.data = "high: " + self.high_state

        self.high_state_pub.publish(msg)


        if self.high_state != self.next_state:
            self.high_state = self.next_state
            self.point_published = False
            return 
        
def main(args=None):
    rclpy.init(args=args)
    nodeh = HighSM()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user...")

if __name__ == "__main__":
    main()