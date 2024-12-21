#!/usr/bin/python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

class turtlesim_motor(Node):
  def __init__(self):
    super().__init__("sim_motor_node")
    self.sub = self.create_subscription(Twist,"turtle1/cmd_vel",self.moving,10)
    self.get_logger().info("Program Start ...")
    
    self.timer = self.create_timer(5,self.status)
    self.timer.cancel()


    self.direction = None

    self.PWM_A = 12
    self.PIN_A_PO = 26
    self.PIN_A_NE = 16

    self.PWM_B = 13
    self.PIN_B_PO = 5
    self.PIN_B_NE = 6

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(self.PWM_A, GPIO.OUT)
    GPIO.setup(self.PWM_B, GPIO.OUT)

    GPIO.setup(self.PIN_A_PO,GPIO.OUT)
    GPIO.setup(self.PIN_A_NE,GPIO.OUT)
    GPIO.setup(self.PIN_B_PO,GPIO.OUT)
    GPIO.setup(self.PIN_B_NE,GPIO.OUT)

    self.pwm_a = GPIO.PWM(self.PWM_A, 10000)
    self.pwm_b = GPIO.PWM(self.PWM_B, 10000)

    # pwm start
    self.pwm_a.start(50)
    self.pwm_b.start(50)
    # stanby
    GPIO.output(self.PIN_A_NE,GPIO.LOW)
    GPIO.output(self.PIN_A_PO,GPIO.LOW)
    GPIO.output(self.PIN_B_NE,GPIO.LOW)
    GPIO.output(self.PIN_B_PO,GPIO.LOW)

  def stop(self):
    GPIO.output(self.PIN_A_NE,GPIO.LOW)
    GPIO.output(self.PIN_A_PO,GPIO.LOW)
    GPIO.output(self.PIN_B_NE,GPIO.LOW)
    GPIO.output(self.PIN_B_PO,GPIO.LOW)
  def forward(self):
    GPIO.output(self.PIN_A_PO,GPIO.HIGH)
    GPIO.output(self.PIN_A_NE,GPIO.LOW)
    GPIO.output(self.PIN_B_PO,GPIO.HIGH)
    GPIO.output(self.PIN_B_NE,GPIO.LOW)
  def reverse(self):
    GPIO.output(self.PIN_A_PO,GPIO.LOW)
    GPIO.output(self.PIN_A_NE,GPIO.HIGH)
    GPIO.output(self.PIN_B_PO,GPIO.LOW)
    GPIO.output(self.PIN_B_NE,GPIO.HIGH)
  def turn(self,direction):
    if direction == "right":
      GPIO.output(self.PIN_A_PO,GPIO.LOW)
      GPIO.output(self.PIN_A_NE,GPIO.HIGH)
      GPIO.output(self.PIN_B_PO,GPIO.HIGH)
      GPIO.output(self.PIN_B_NE,GPIO.LOW)
    elif direction == "left":
      GPIO.output(self.PIN_A_PO,GPIO.HIGH)
      GPIO.output(self.PIN_A_NE,GPIO.LOW)
      GPIO.output(self.PIN_B_PO,GPIO.LOW)
      GPIO.output(self.PIN_B_NE,GPIO.HIGH)


  def moving(self,msg):
    linear = msg.linear.x
    angular = msg.angular.z
    if linear >0:
      self.forward()
    elif linear <0:
      self.reverse()
    else:
      if angular > 0:
        self.turn("left")
      elif angular < 0:
        self.turn("right")
    self.timer.reset()
    
    
  def status(self):
    self.stop()
    self.timer.cancel()


def main(args = None):
  try:
    rclpy.init(args=args)
    node = turtlesim_motor()
    rclpy.spin(node)
    rclpy.shutdown()
  finally:
    GPIO.cleanup()