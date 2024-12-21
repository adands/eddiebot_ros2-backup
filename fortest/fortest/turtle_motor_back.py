#!/usr/bin/python3

#import
import rclpy
import pigpio
from geometry_msgs.msg import Twist, Pose ,Point ,Vector3 ,Quaternion, TransformStamped, Transform
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from rclpy.node import Node
import math
from tf2_ros import TransformBroadcaster
import sys
import time
from tf_transformations import quaternion_from_euler


#functions
class motor_control(Node):

  def __init__(self):
    self.pi = pigpio.pi()
    super().__init__("motor_node")
    if not self.pi.connected:
      exit()
    self.sub = self.create_subscription(Twist,"cmd_vel",self.motor_direction,10)
    self.pub = self.create_publisher(Odometry,"odom_diff",10)
    self.timer = self.create_timer(1/20,self.speed_count)
    self.timer1 = self.create_timer(0.1,self.speed_control)
    self.timer_odom = self.create_timer(1/200,self.odom_count)
    self.timer_pub = self.create_timer(1/200,self.publish_odom)


    # variable and pins setup
    self.dc_value_r = 0
    self.dc_value_l = 0
    self.x = 0
    self.z = 0
    self.pos_x = 0.0
    self.pos_y = 0.0
    self.del_th = 0
    self.status = 0
    self.pre_status = 0
    self.left_count = 0
    self.right_count = 0

    # for PID
    self.kp = 40
    self.ki = 0
    self.kd = 15
    
    self.kp_r = 40
    self.ki_r = 0
    self.kd_r = 15

    self.error_r =0
    self.del_error_r = 0
    self.error_plus_r = 0
    self.error_l =0
    self.del_error_l = 0
    self.error_plus_l = 0

    # speed counting
    self.prev_time = time.time()    # previous time
    self.speed_Right = 0.0          # speed of right wheel
    self.speed_Left = 0.0           # speed of left wheel

        ## right    
    self.PWM_A = 12
    self.PIN_A_PO = 26
    self.PIN_A_NE = 16
    self.encoder_right = 17

        ## left
    self.PWM_B = 13
    self.PIN_B_PO = 5
    self.PIN_B_NE = 6
    self.encoder_left = 24

    #self.pi setup
    self.pi.set_mode(self.PWM_A, pigpio.OUTPUT)
    self.pi.set_mode(self.PWM_B, pigpio.OUTPUT)

    self.pi.set_mode(self.PIN_A_PO,pigpio.OUTPUT)
    self.pi.set_mode(self.PIN_A_NE,pigpio.OUTPUT)
    self.pi.set_mode(self.PIN_B_PO,pigpio.OUTPUT)
    self.pi.set_mode(self.PIN_B_NE,pigpio.OUTPUT)


    self.pi.set_mode(self.encoder_right,pigpio.INPUT)
    self.pi.set_mode(self.encoder_left,pigpio.INPUT)

    self.pi.set_PWM_frequency(self.PWM_A,2000)
    self.pi.set_PWM_frequency(self.PWM_B,2000)
    self.pi.set_PWM_dutycycle(self.PWM_A,0)
    self.pi.set_PWM_dutycycle(self.PWM_B,0)

    self.pi.set_pull_up_down(self.encoder_left,pigpio.PUD_UP)
    self.pi.set_pull_up_down(self.encoder_right,pigpio.PUD_UP)

    # encoder interrupt
    self.cb1 =self.pi.callback(self.encoder_left , pigpio.FALLING_EDGE, self.encoder_read_left )
    self.cb2 =self.pi.callback(self.encoder_right, pigpio.FALLING_EDGE, self.encoder_read_right)

    # stanby
    self.pi.write(self.PIN_A_NE,0)
    self.pi.write(self.PIN_A_PO,0)
    self.pi.write(self.PIN_B_NE,0)
    self.pi.write(self.PIN_B_PO,0)

  # encoder interrupt callback functions
  def encoder_read_right(self,gpio,level,tick):
    self.right_count += 1

  def encoder_read_left(self,gpio,level,tick):
    self.left_count += 1



  def speed_count(self):

    self.speed_Right = (self.speed_Right * 0.8) + (0.2*((self.right_count/918)*(0.125*math.pi))/(1/20))
    self.speed_Left = (self.speed_Left*0.8) + (0.2*((self.left_count/918)*(0.125*math.pi))/(1/20))

    self.right_count = 0 ; self.left_count = 0


  def stop(self):
    self.pi.write(self.PIN_A_PO,0)
    self.pi.write(self.PIN_A_NE,0)
    self.pi.write(self.PIN_B_PO,0)
    self.pi.write(self.PIN_B_NE,0)
    self.error_r =0
    self.del_error_r = 0
    self.error_plus_r = 0
    self.error_l =0
    self.del_error_l = 0
    self.error_plus_l = 0


  # motor direction function
  def motor_direction(self,msg):
    self.x = msg.linear.x
    self.z = msg.angular.z

    if self.x > 0 :
      self.pi.write(self.PIN_A_PO,1)
      self.pi.write(self.PIN_A_NE,0)
      self.pi.write(self.PIN_B_PO,1)
      self.pi.write(self.PIN_B_NE,0)
      self.status = 1
    elif self.x < 0:
      self.pi.write(self.PIN_A_PO,0)
      self.pi.write(self.PIN_A_NE,1)
      self.pi.write(self.PIN_B_PO,0)
      self.pi.write(self.PIN_B_NE,1)
      self.status = -1
    else:
      if self.z > 0 :
        self.pi.write(self.PIN_A_PO,1)
        self.pi.write(self.PIN_A_NE,0)
        self.pi.write(self.PIN_B_PO,0)
        self.pi.write(self.PIN_B_NE,1)
        self.status = 2
      elif self.z< 0:
        self.pi.write(self.PIN_A_PO,0)
        self.pi.write(self.PIN_A_NE,1)
        self.pi.write(self.PIN_B_PO,1)
        self.pi.write(self.PIN_B_NE,0)
        self.status = -2
      else:
        self.pi.set_PWM_dutycycle(self.PWM_A,0)
        self.pi.set_PWM_dutycycle(self.PWM_B,0)
        self.dc_value_r = 0
        self.dc_value_l = 0
        self.stop()
        self.speed_Left = 0
        self.speed_Right = 0
    if self.pre_status != self.status:
      #print(self.status)
      self.stop()

    self.pre_status = self.status

# speed control function (PID)
  def speed_control(self):
    #print("左輪PWM: ",self.dc_value_l,"右輪PWM: ",self.dc_value_r)
    #print(f"左輪速度:{self.speed_Left}\t右輪速度:{self.speed_Right}")

    # linear(m/s) = (r + l)/ 2
    # angular(radian/s) = (r - l)/wheel_separation  .
    # wheel separation = 0.383 m
    # if z > 0, the robot will turn left and the left wheel need to go reverse so the speed will be negative

    r = (2*self.x + 0.383*self.z)/2
    l = (2*self.x - 0.383*self.z)/2

    self.error_l = abs(l) - abs(self.speed_Left)
    self.error_r = abs(r) - abs(self.speed_Right)

    self.error_plus_l += self.error_l
    derivative_left  = self.error_l - self.del_error_l
    self.del_error_l = self.error_l
    
    self.error_plus_r += self.error_r
    derivative_right = self.error_r - self.del_error_r
    self.del_error_r = self.error_r

    if abs(self.speed_Left) != abs(l):
      self.dc_value_l += self.MIN_MAX((self.error_l * self.kp + self.error_plus_l * self.ki+ (derivative_left)/0.1*self.kd),self.dc_value_l)
      self.pi.set_PWM_dutycycle(self.PWM_B,self.dc_value_l)

    if abs(self.speed_Right) != abs(r):

      self.dc_value_r += self.MIN_MAX((self.error_r * self.kp_r + self.error_plus_r * self.ki_r+ (derivative_right)/0.1*self.kd_r),self.dc_value_r)
      self.pi.set_PWM_dutycycle(self.PWM_A,self.dc_value_r)

  def MIN_MAX(self,value,duty):
    if value + duty <= 255 and value + duty >= 0:
        return value
    elif value + duty > 255:
        #print("more")
        return (255 - duty)
    else:
        #print("less")
        return -duty

  # count odometry
  def odom_count(self):

    duration = 1/200
    speed_l = self.speed_Left
    speed_r = self.speed_Right

    if self.x<0:
      speed_l*= -1 ; speed_r*=-1
    elif self.x ==0:
      if self.z > 0:
        speed_l*= -1
      elif self.z < 0:
        speed_r *= -1  

    linear = (speed_r + speed_l)/2
    angular = (speed_r -speed_l)/0.383

    if duration > 0:
      self.del_th += angular*duration
      self.pos_x += linear*duration * math.cos(self.del_th)
      self.pos_y += linear*duration * math.sin(self.del_th)

  # publish odometry
  def publish_odom(self):
    t = self.get_clock().now()
    odom = Odometry()

    # header
    odom.header.stamp = t.to_msg()
    odom.header.frame_id = "odom"

    # child_frame_id
    odom.child_frame_id = "base_footprint"

    # position
    pos = odom.pose.pose.position
    pos.x = self.pos_x ; pos.y = self.pos_y

    orie = odom.pose.pose.orientation
    theta = self.del_th *(180/math.pi)
    x,y,z,w = quaternion_from_euler(0,0,self.del_th)
    orie.x = x ; orie.y = y ; orie.z = z ; orie.w = w

    # twist
    odom.twist.twist.linear.x = (self.speed_Right + self.speed_Left)/2
    odom.twist.twist.angular.z = (self.speed_Right -self.speed_Left)/0.383

    self.pub.publish(odom)


# main code
def main(args=None):
  try:
    rclpy.init(args = args)
    node = motor_control()
    rclpy.spin(node)
    rclpy.shutdown()
  finally:
    node.stop()
    node.cb1.cancel()
    node.cb2.cancel()
    node.pi.stop()

