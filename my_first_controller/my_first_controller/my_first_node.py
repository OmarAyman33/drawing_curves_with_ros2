#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class myNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.__counter = 0
        self.get_logger().info("Hello from ROS2") # how to log a message onto the terminal.
        self.create_timer(1.0,self.timer_callback) # creating a timer that runs a function y every interval x.
    
    def timer_callback(self):
        self.get_logger().info("Hello! " + str(self.__counter))
        self.__counter +=1
            
def main(args = None):
    rclpy.init(args = args) # initialize ros2 (must be placed in the beginning of all ROS2 source files)
    
    node = myNode() # creating an object of the node class we made.
    
    rclpy.spin(node) # keeps node alive indefinetly until killed
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()

