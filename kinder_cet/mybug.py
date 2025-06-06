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
        
        #Odemetry
        self.current_pose = []
        self.target_pose = []
        self.got_new_target = False
        self.msg1 = Twist()
        
        #Lidar
        self.robotView = []
              
        #Maquina de estados
        self.state = "stop_robot"
        self.next_state = self.state
        self.first_time_flag = True
        self.wall_following_direction = "left"
        self.closest_dist_to_goal_on_wall = float('inf')
        
        #Misc
        self.tolerance_to_target = 0.05   
        self.tolerance_to_line = 0.05
        self.allowed_distance_to_obstacle = 0.30

        self.histerisis = 0.05
        
        #control paramters
        self.angP = 0.3 #proportional of angular error
        self.linP = 0.2 #proportional of linear error  
        
        self.linMax = 0.1 #m/s
        self.angMax = 0.2 #rad/s 
        
        #Follow wall parameters
        self.Dwall = 0.35
        self.beta = 0.75
        self.Kfw = 0.99 #ganancia follow wall
        
        #Bug 2 Variables
        self.PointStart = [0.0, 0.0]
        self.PointGoal = [0.0, 0.0]
        self.Distance2Goal = 0.0
            #Line equation variables
        self.a = 0.0
        self.b = 0.0
        self.c = 0.0   
    
    def lidar_callback(self,data):
        #print("Frame = " + str(data.header.frame_id))
        #print("Range min [m] = " + str(data.range_min))
        #print("Range max [m] = " + str(data.range_max))
        #print(data.ranges)
    
      #Puzzlebot Real
      #  ranges = list(data.ranges)
      #  for i in range(len(ranges)):
      #      if ranges[i] > data.range_max: ranges[i] = data.range_max +0.01
      #      if ranges[i] < data.range_min: ranges[i] = data.range_min-0.01
      #  self.robotView = {
      #      'front' : min(min(ranges[0:66]),min(ranges[1014:1080])),
      #      'front_right' : min(ranges[877:1013]),
      #      'front_left' : min(ranges[67:201]),
      #      'right' : min(ranges[202:336]),
      #      'back' : min(ranges[337:741]),
      #      'left' : min(ranges[741:876])
      #  }
      #Puzzlebot Simulado  
        
        ranges = list(data.ranges)
        for i in range(len(ranges)):
            if ranges[i] > data.range_max: ranges[i] = data.range_max +0.01
            if ranges[i] < data.range_min: ranges[i] = data.range_min-0.01
        self.robotView = {
            'front' : min(min(ranges[0:22]),min(ranges[338:359])),
            'front_right' : min(ranges[294:337]),
            'front_left' : min(ranges[23:67]),
            'right' : min(ranges[248:292]),
            'back' : min(ranges[113:247]),
            'left' : min(ranges[68:112])            
        }
        

    def target_callback(self,msg):

        new_target = [msg.x,msg.y,msg.theta]
        self.PointGoal = [msg.x, msg.y]
        self.PointStart = [self.current_pose[0], self.current_pose[1]]
        self.Distance2Goal = abs(math.hypot(self.PointGoal[0]-self.PointStart[0], self.PointGoal[1]-self.PointStart[1]))
        self.a = self.PointGoal[1] - self.PointStart[1]
        self.b = self.PointGoal[0] - self.PointStart[0]
        self.c = -(self.a * self.PointStart[0] + self.b * self.PointStart[1])
        print(self.PointStart)
        
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

        v = min(self.linMax,max(v,-self.linMax))
        w = min(self.angMax,max(w,-self.angMax))
        self.msg1.linear.x = v
        self.msg1.angular.z = w


        self.pub.publish(self.msg1)
        
    def stop_robot(self):
        self.move_robot(0.0,0.0)
        if self.first_time_flag == True: 
            print("Stopin' robot")
            self.first_time_flag = False
        
    def go_to_goal(self):
        if self.first_time_flag == True: 
            print("Goin' to goal")
            self.first_time_flag = False
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
     
    def follow_wall(self,direction):
        if self.first_time_flag == True: 
            print("Follow Wall")
            
            self.first_time_flag = False
        
        if direction == "left":
            r1,t1 = self.robotView.get("left"), math.radians(90)
            r2,t2 = self.robotView.get("front_left"), math.radians(45)
        
        else: 
            r1,t1 = self.robotView.get("right"), math.radians(-90)
            r2,t2 = self.robotView.get("front_right"), math.radians(-45)

        P1 = [r1 * math.cos(t1), r1 * math.sin(t1)]
        P2 = [r2 * math.cos(t2), r2 * math.sin(t2)]
        Tan = [P2[0] - P1[0], P2[1] - P1[1]]
        hypTan = math.hypot(Tan[0], Tan[1])
        NormTan = [Tan[0]/hypTan, Tan[1]/hypTan]
        #Calcular U perpendicualr
        dot = P1[0] * NormTan[0] + P1[1] * NormTan[1]
        Uper = [P1[0] - dot * NormTan[0], P1[1] - dot * NormTan[1]]
        #Calcular vector unitario de uper
        hypPer = math.hypot(Uper[0], Uper[1])
        NormPer = [Uper[0]/hypPer, Uper[1]/hypPer]
        
        #Calcular follow wall angle
        E_per = [Uper[0]-self.Dwall*NormPer[0], Uper[1]-self.Dwall*NormPer[1]]
        angle_per = math.atan2(E_per[1],E_per[0])
        angle_tan = math.atan2(NormTan[1], NormTan[0])
        fw_angle = self.beta * angle_tan + (1 - self.beta) * angle_per
        fw_angle = math.atan2(math.sin(fw_angle), math.cos(fw_angle)) #obtener distancia mas corta del angulo
        #v = 0.1 if abs(fw_angle) > 0.1 else 0.04
        if abs(fw_angle) > 0.1:
            v = 0.04
        else:
            v = 0.1
        w = self.Kfw * fw_angle
        #print("distance to wall: " +str(hypPer) + " fw_angle: " + str(fw_angle))
        msg = String()
        msg.data = "Error to wall: " + str(E_per) + " fw_angle: " + str(fw_angle)
        #self.state_pub.publish(msg)
        self.move_robot(v,w)
        
         
    def gotNewTarget(self):
        return self.got_new_target
    
    def atTarget(self):
        Ex =self.target_pose[0] - self.current_pose[0]
        Ey = self.target_pose[1] - self.current_pose[1]
        distance_to_target = math.hypot(Ex,Ey)
        #print('\x1b[2k', end='\r')
        #print("Distance to target = " + str(distance_to_target), end = '\r')

        if distance_to_target < self.tolerance_to_target:
            #print("\nArrived to target")
            self.first_time_flag = True
            self.target_pose = []
            self.got_new_target = False
            return True
        else:
            return False
            
    def avoid_obstacle(self):
        if self.first_time_flag == True: 
            #print("Avoiding Obstacle")
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
        if min(readings) <= self.allowed_distance_to_obstacle:
            return True
        else: return False
        
    def isObstacleCleared(self):

        if self.robotView.get("front") > self.allowed_distance_to_obstacle + self.histerisis:
            return True
        else:
            return False
        
    def mLineAgainWithProgress(self):
         
        eq = abs(self.a * self.current_pose[0] + self.b * self.current_pose[1] + self.c)
        dist2line = eq / math.hypot(self.a, self.b)
        ActDist2goal = math.hypot(self.current_pose[0] - self.PointGoal[0], self.current_pose[1] - self.PointGoal[1])
        if self.state == "follow_wall_left" or self.state == "follow_wall_right":
            print(dist2line, end = "\r", flush = True)
        
        inx = self.current_pose[0] <= self.PointStart[0] +0.05 and self.current_pose[0] >= self.PointStart[0]-0.05

        iny = self.current_pose[1] <= self.PointStart[1] +0.05 and self.current_pose[1] >= self.PointStart[1]-0.05
        
        if dist2line < self.tolerance_to_line and ActDist2goal < self.Distance2Goal:
            return True
        
        
        else:
            return False
        
    def choose_follow_direction(self):
        if self.robotView.get("front_right") < self.robotView.get("front_left"):
            return "right"
        else:
            return "left"
    
         
       
    def stateMachine(self):
        if len(self.current_pose) > 0 and len(self.robotView) > 0:

            # Ejecutar acción según estado
            if self.state == "stop_robot":
                self.stop_robot()

            elif self.state == "go_to_goal":
                self.go_to_goal()

            elif self.state == "follow_wall_left":
                self.follow_wall("left")

            elif self.state == "follow_wall_right":
                self.follow_wall("right")

            # Transiciones
            if self.state == "stop_robot" and self.gotNewTarget():
                self.next_state = "go_to_goal"

            elif self.state == "go_to_goal":
                if self.atTarget():
                    self.next_state = "stop_robot"
                elif self.isObstacleTooClose():
                    self.closest_dist_to_goal_on_wall = math.hypot(
                        self.current_pose[0] - self.PointGoal[0],
                        self.current_pose[1] - self.PointGoal[1]
                    )
                    self.wall_following_direction = self.choose_follow_direction()
                    self.next_state = f"follow_wall_{self.wall_following_direction}"

            elif self.state in ["follow_wall_left", "follow_wall_right"]:
                if self.mLineAgainWithProgress():
                    self.next_state = "go_to_goal"

            # Aplicar transición
            if self.next_state != self.state:
                self.state = self.next_state
                self.first_time_flag = True

            # Publicar estado actual
            msg = String()
            msg.data = self.state
            self.state_pub.publish(msg)
            

        
        
def main(args=None):
    rclpy.init(args=args)
    nodeh = BugAlgorithClass()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user...")

if __name__ == "__main__":
    main()