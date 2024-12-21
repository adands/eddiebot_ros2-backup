#!/usr/bin/python3


from email import header
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist ,TransformStamped, Transform
from std_msgs.msg import Header
from math import *
from tf2_ros import TransformBroadcaster

class odom_dect(Node):

    def __init__(self):
        super().__init__("odom_dect")
        self.sub = self.create_subscription(Twist,"cmd_vel",self.save_status,10)
        self.timer = self.create_timer(1/30,self.odom_count)
        self.sub_odom = self.create_subscription(Odometry,"odometry/filtered",self.odom_error,10)
        self.del_th = 0
        self.pos_x = 0
        self.pos_y = 0
        self.linear = 0
        self.angular = 0

        # self.tf_broadcaster = TransformBroadcaster(self)
        # self.header = None
        # self.pose =  None
        # self.ro = None
        # self.child = None
    
    def save_status(self,msg):
        self.linear = msg.linear.x
        self.angular = msg.angular.z

        # self.header = msg.header
        # self.child = msg.child_frame_id
        # self.pose = msg.pose.pose.position
        # self.ro = msg.pose.pose.orientation

    def odom_count(self):
        duration = 1/30
        if duration > 0:
          self.del_th += self.angular*duration
          self.pos_x += self.linear*duration * cos(self.del_th)
          self.pos_y += self.linear*duration * sin(self.del_th)
        #print(f"current position : ({self.pos_x , self.pos_y})\n",flush=True)

        # trans = TransformStamped()

        # t = self.get_clock().now()
        # head = Header()
        # head.stamp = self.header.stamp
        # head.frame_id = self.header.frame_id
        
        # trans.header = head
        # trans.child_frame_id = self.child
        # trans.transform.translation = self.pose

        # trans.transform.rotation = self.ro

        # self.tf_broadcaster.sendTransform(trans)

    def odom_error(self,msg):
        if abs(self.pos_x - msg.pose.pose.position.x) > 0.5:
            print("x position error",flush=True)
        if abs(self.pos_y - msg.pose.pose.position.y) > 0.5:
            print("y position error",flush=True)        

def main(args=None):
    try:
        rclpy.init(args=args)
        node = odom_dect()
        rclpy.spin(node)
        rclpy.shutdown()
    finally:
        print("shutdown... \n byebye~~~")