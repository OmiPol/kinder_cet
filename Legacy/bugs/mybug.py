#!/usr/bin/env python3
import rclpy, math, tf_transformations
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose


class BugAlgorithClass(Node):
    def __init__(self):
        super().__init__("Buuug")
        self.get_logger().info("Bug node has been started...")
        self.create_timer(0.1,self.stateMachine)
        self.pub = self.create_publisher(Twist, "/cmd_vel",1)
        self.odom = self.create_subscription(Odometry, "/odom", self.odom_callback,1)
        self.create_subscription(Pose,"/target",self.target_callback,1)
        self.create_subscription(LaserScan,"/scan",self.lidar_callback,1)
        self.state_pub = self.create_publisher(String, "/state",1)
        self.current_pose = []
        self.target_pose = []
        self.got_new_target = False
        self.msg1 = Twist()
        
        #Lidar
        self.robotView = []
              
        #Maquina de estados
        self.state = "stop_robot"
        self.first_time_flag = True
        
        #Misc
        self.tolerance_to_target = 0.05   
        self.allowed_distance_to_obstacle = 0.40 
        self.histerisis = 0.1
        
        #control paramters
        self.angP = 2.0  #proportional of angular error
        self.linP = 1.8 #proportional of linear error  
        
        self.linMax = 0.1 #m/s
        self.angMax = 0.3 #rad/s  
        
        
    def lidar_callback(self,data):
        #print("Frame = " + str(data.header.frame_id))
        #print("Range min [m] = " + str(data.range_min))
        #print("Range max [m] = " + str(data.range_max))
        #print(data.ranges)
        
        
        ranges = list(data.ranges)
        for i in range(len(ranges)):
            if ranges[i] > data.range_max: ranges[i] = data.range_max +0.01
            if ranges[i] < data.range_min: ranges[i] = data.range_min-0.01
        self.robotView = {
            'front' : min(min(ranges[0:22]),min(ranges[338:359])),
            'front_right' : min(ranges[68:112]),
            'front_left' : min(ranges[248:292]),
            'right' : min(ranges[248:292]),
            'back' : min(ranges[113:247]),
            'left' : min(ranges[23:67]),
            'front_left' : min(ranges[248:292])
            
                
        }
        
    def target_callback(self,msg):

        new_target = [msg.x,msg.y,msg.theta]
        
        if len(self.target_pose) == 0 or self.target_pose != new_target:
            self.target_pose = new_target 
            self.got_new_target = True
            print("I got new target")  
            
                
    def odom_callback(self,msg):
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w]
        __,__,yaw = tf_transformations.euler_from_quaternion(q)
        self.current_pose = [x,y,yaw]
    
    def move_robot(self,v,w):
        self.msg1.linear.x = v
        self.msg1.angular.z = w
        self.pub.publish(self.msg1)
        
    def stop_robot(self):
        self.move_robot(0.0,0.0)
        if self.first_time_flag == True: 
            print("Stopin' robot")
            self.first_time_flag = False
        
    def go_to_goal(self):
         #Error calculation
        Ex =self.target_pose[0] - self.current_pose[0]
        Ey = self.target_pose[1] - self.current_pose[1]
        distance_to_target = math.hypot(Ex,Ey)
        #print(e_dist)
        angle_diff = np.arctan2(Ey,Ex)
        angle_error = np.arctan2(math.sin(angle_diff-self.current_pose[2]),math.cos(angle_diff-self.current_pose[2]))
        
            #Checks if current position is within tolerance
            #if it is, stop movment, if it is not, continue movement
        if abs(distance_to_target) > self.tolerance_to_target:
            #saturation of max velocities
             x = max(min(self.linP * distance_to_target,self.linMax),-self.linMax)
             z = max(min(self.angP * angle_error,self.angMax),-self.angMax)  
        else:
            x = 0.0
            z =0.0
         
         #publishing message
        self.move_robot(x,z)
        
    
    def gotNewTarget(self):
        return self.got_new_target
    
    def atTarget(self):
        Ex =self.target_pose[0] - self.current_pose[0]
        Ey = self.target_pose[1] - self.current_pose[1]
        distance_to_target = math.hypot(Ex,Ey)
        print('\x1b[2k', end='\r')
        print("Distance to target = " + str(distance_to_target), end = '\r')

        if distance_to_target < self.tolerance_to_target:
            print("\nArrived to target")
            self.first_time_flag = True
            self.target_pose = []
            self.got_new_target = False
            return True
        else:
            return False
            
    def avoid_obstacle(self):
        if self.first_time_flag == True: 
            print("Avoiding Obstacle")
            self.first_time_flag = False
        
        angles = [90,45,0,-45,-90]
        readings = [self.robotView.get("left"),
                    self.robotView.get("front_left"),
                    self.robotView.get("front"),
                    self.robotView.get("front_right"),
                    self.robotView.get("right")]
        num, den = 0.0, 0.0
        for i in range(len(angles)):
            num += angles[i]*readings[i]
            den += readings[i]
                
        v = 0.05
        w = 1.0 * math.radians(num/den)
        self.move_robot(v,w)

        
    
    def isObstacleTooClose(self):
        readings = [
                        self.robotView.get("front_left"),
                        self.robotView.get("front"),
                        self.robotView.get("front_right"),
        ]
        if min(readings) <= self.allowed_distance_to_obstacle - self.histerisis:
            return True
        else: return False
        
    def isObstacleCleared(self):
        
        return not(self.isObstacleTooClose())
        
        if self.robotView.get("front") > self.allowed_distance_to_obstacle + self.histerisis:
            return True
        else:
            return False
        
    
       
    def stateMachine(self):
        if len(self.current_pose) > 0 and len(self.robotView) > 0: 
			#States
            if self.state == "stop_robot": self.stop_robot()
            if self.state == "go_to_goal": self.go_to_goal()
            if self.state == "avoid_obstacle": self.avoid_obstacle()

			#Transitions
            if self.state == "stop_robot" and self.gotNewTarget(): self.state = "go_to_goal"
            if self.state == "go_to_goal" and self.atTarget(): self.state = "stop_robot"
            if self.state == "go_to_goal" and self.isObstacleTooClose(): self.state = "avoid_obstacle"
            if self.state == "avoid_obstacle" and self.isObstacleCleared(): self.state = "go_to_goal"
            msg = String()
            msg.data = self.state + str(self.robotView)
            self.state_pub.publish(msg)
            #print(self.gotNewTarget())
            

        
        
def main(args=None):
    rclpy.init(args=args)
    nodeh = BugAlgorithClass()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user...")

if __name__ == "__main__":
    main()