#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class TestLidarClass(Node):
    def __init__(self):
        super().__init__("Test_Lidar_node")
        self.get_logger().info("Test_lidar node has been started...")
        self.create_timer(0.1,self.stateMachine)
        self.create_subscription(LaserScan, '/scan',self.lidar_callback,1)
        self.robotView = []
        
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
            'front' : min(min(ranges[0:66]),min(ranges[1014:1080])),
            'front_right' : min(ranges[877:1013]),
            'front_left' : min(ranges[67:201]),
            'right' : min(ranges[202:336]),
            'back' : min(ranges[337:741]),
            'left' : min(ranges[741:876])
            }
        #print(self.robotView["front"])
                
        
        
        
    
    def stateMachine(self):
        if(len(self.robotView) > 0):
            #print(self.robotView)

            lowest_key = min(self.robotView, key=self.robotView.get)
            print(lowest_key)          # Output: 'b'
            print(self.robotView[lowest_key]) # Output: 5

    
 
 
        
def main(args=None):
    rclpy.init(args=args)
    nodeh = TestLidarClass()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user...")

if __name__ == "__main__":
    main()