#!/usr/bin/python3

import select
import termios
import sys
import tty
import rclpy
from geometry_msgs.msg import Twist

# variables
max_linear = 0.93 # m/s
max_angular = 4.75 # rad/s
start_msg = """
This is a teleop program for lab412 eddiebot.
Here is the tutorial of this controller
            w
        a   s   d
            x
use w ro x to go forward or go reverse
use a or d to rotate left or right
use s to stop all moving

w and x will change the linear.x value
a and d will change the angular.z value
use q to quit the program
"""

setting = termios.tcgetattr(sys.stdin)
# functions

def get_key():

    tty.setraw(sys.stdin.fileno())
    rlist, _ ,_ =select.select([sys.stdin],[],[],0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    return key

def checklimit_linear(value):
    if value < -(max_linear):
        value = -max_linear
    elif value > max_linear:
        value = max_linear
    else:
        value = value
    return value


def checklimit_angular(value):
    if value < -(max_angular):
        value = -max_angular
    elif value > max_angular:
        value = max_angular
    else:
        value = value
    return value

def main(args = None):

    rclpy.init(args=args)
    node = rclpy.create_node("teleop")
    pub = node.create_publisher(Twist,"cmd_vel",10)
    msg = Twist()
    current_linear = 0.0
    current_angular = 0.0
    print(start_msg)
    print(f"\rcurrent_linear: {current_linear}  current_angular: {current_angular} ")
    try:
        while True:
            key = get_key()
            if key == "w":
                if current_linear == 0:
                    current_linear += 0.24
                else:
                    current_linear = checklimit_linear(current_linear + 0.01)
                print(f"\rcurrent_linear: {current_linear}  current_angular: {current_angular}")
            elif key == "x":
                if current_linear == 0:
                    current_linear -= 0.24
                else:
                    current_linear = checklimit_linear(current_linear - 0.01)
                print(f"\rcurrent_linear: {current_linear}  current_angular: {current_angular} ")
            elif key == "a":
                current_angular = checklimit_angular(current_angular + 0.1)
                print(f"\rcurrent_linear: {current_linear}  current_angular: {current_angular} ")
            elif key == "d":
                current_angular = checklimit_angular(current_angular - 0.1)
                print(f"\rcurrent_linear: {current_linear}  current_angular: {current_angular} ")
            elif key == "s":
                current_linear = 0.0
                current_angular = 0.0
                print(f"\rcurrent_linear: {current_linear}  current_angular: {current_angular} ")
            elif key == "q":
                print("\rquit")  
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                pub.publish(msg)  
                break
            msg.linear.x = current_linear
            msg.angular.z = current_angular
            pub.publish(msg)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, setting)

