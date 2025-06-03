#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class TestLidarClass(Node):
    def __init__(self):
        super().__init__("Test_Lidar_node")
        self.get_logger().info("Test_lidar node has been started...")
        self.create_subscription(LaserScan, '/scan',self.lidar_callback,1)
        self.pub = self.create_publisher(LaserScan,"/scan_rect",1)
        self.robotView = []
        
    def lidar_callback(self,data):


        msg = data
        msg.header.frame_id = "sim/laser_frame"
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        


        #print(self.robotView["front"])
                
        
        
        

    
 
 
        
def main(args=None):
    rclpy.init(args=args)
    nodeh = TestLidarClass()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node terminated by user...")

if __name__ == "__main__":
    main()