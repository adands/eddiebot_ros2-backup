 #!/usr/bin/python3

import rclpy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import Float32MultiArray , MultiArrayLayout ,MultiArrayDimension

from rclpy.node import Node
import math

class encoder(Node):

  def __init__(self):
    
    # Ros2 setting
    super().__init__("encoder")
    self.pub = self.create_publisher(Float32MultiArray,"speed",10)
    self.timer1 = self.create_timer(1,self.count_speed)
    self.timer2 = self.create_timer(1,self.odometry_conut)
    self.odom = Float32MultiArray()
    self.msg = Float32MultiArray()

    # variable
    self.prev_time = time.time()    # previous time 
    self.prev_right_count = 0       # previous right wheel pulse
    self.prev_left_count = 0        # previous left wheel pulse
    self.encoder_count_left = 0     # current left wheel pulse
    self.encoder_count_right = 0    # current left wheel pulse
    self.speed_Right = 0.0          # speed of right wheel
    self.speed_Left = 0.0           # speed of left wheel
    self.pos_x = 0
    self.pos_y = 0

    # GPIO setting
    GPIO.setmode(GPIO.BCM)
    self.encoder_right = 17
    self.encoder_left = 24
    GPIO.setup(self.encoder_right,GPIO.IN,pull_up_down = GPIO.PUD_UP)
    GPIO.setup(self.encoder_left,GPIO.IN,pull_up_down = GPIO.PUD_UP)
    GPIO.add_event_detect(self.encoder_right,GPIO.RISING,  callback=self.encoder_read_right)
    GPIO.add_event_detect(self.encoder_left, GPIO.RISING, callback=self.encoder_read_left)
    
  # callback functions
  def encoder_read_left(self,channel):
    self.encoder_count_left +=1

  def encoder_read_right(self,channel):
    self.encoder_count_right +=1 
    
  # speed counting function
  def count_speed(self):
    current_time = time.time()
    #print("pre_time: ",self.prev_time,"current_time: ",current_time)
    time_elapsed = current_time - self.prev_time
    pulse_change_right = self.encoder_count_right - self.prev_right_count
    pulse_change_left = self.encoder_count_left - self.prev_left_count 

    if time_elapsed >0:
      self.speed_Right = (pulse_change_right / time_elapsed) * ((0.1524*math.pi)/36)
      self.speed_Left = (pulse_change_left / time_elapsed) * ((0.1524*math.pi)/36)
    else:
      pass
      
    self.prev_time = current_time
    self.prev_right_count = self.encoder_count_right
    self.prev_left_count = self.encoder_count_left
    
    self.msg.data = [self.speed_Right,self.speed_Left]
    print("左輪速度: ",self.speed_Left,"右輪速度: ",self.speed_Right)
    self.pub.publish(self.msg)

  # publish function
  def odometry_conut(self):
    linear = (self.speed_Right + self.speed_Left)/2
    angular = (self.speed_Right - self.speed_Left)/0.39
    self.pos_x += linear*math.cos(angular)
    self.pos_y += linear*math.sin(angular)
    self.odom.data = [self.pos_x,self.pos_y]
    print(f"odometry:[{self.pos_x},{self.pos_y}]")

# main function
def main(args=None):
  try:
    rclpy.init(args = args)
    node = encoder()      
    rclpy.spin(node)
    rclpy.shutdown()
  finally:

    GPIO.cleanup()
