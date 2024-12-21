#!/usr/bin/python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu

class detect(Node):

    def __init__(self):
        super().__init__("detecter")
        self.sub = self.create_subscription(Imu,"imu",self.printer,10)
        self.count =0
        self.sec = 0
        
    def printer(self,msg):
        if msg.header.stamp.sec == self.sec:
            self.count +=1
            print(msg.header.stamp,self.count)
        else:
            self.count = 0
            self.sec = msg.header.stamp.sec
            
        
        
def main(args=None):
    try:
        rclpy.init()
        node = detect()
        rclpy.spin(node)
        rclpy.shutdown()
    finally:
        pass