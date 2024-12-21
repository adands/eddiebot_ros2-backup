#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from math import *
import time
from geometry_msgs.msg import Twist

class PID(Node):
    def __init__(self):
        super().__init__("PID")
        self.sub = self.create_subscription(Twist,"cmd_vel",self.count_target,10)
        self.timer = self.create_timer(0.1,self.PID_controller)
        self.left_target = 0.0
        self.right_target = 0.0
        self.kp = 100
        self.ki = 1
        self.kd = 15
        self.left_speed_list = []
        self.right_speed_list = []
        
        self.kp_r = 100
        self.ki_r = 1
        self.kd_r = 15
        
        self.duty_left = 0
        self.l_error =0
        self.l_del_error = 0
        self.status = 1
        self.l_error_plus = 0 

        self.duty_right = 0
        self.r_error =0
        self.r_del_error = 0
        self.r_error_plus = 0 

        self.encoder_count_left = 0
        self.encoder_count_right = 0

        self.encoder_right = 17
        self.encoder_left = 24

        self.prev_left_time = time.time()
        self.prev_right_time = time.time()

        self.speed_Left = 0
        self.speed_Right = 0
        
        self.PWM_A = 13
        self.PIN_A_PO = 26
        self.PIN_A_NE = 16

        self.PWM_B = 12
        self.PIN_B_PO = 5
        self.PIN_B_NE = 6
    
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.encoder_right,GPIO.IN,pull_up_down = GPIO.PUD_UP)
        GPIO.setup(self.encoder_left,GPIO.IN,pull_up_down = GPIO.PUD_UP)
        GPIO.setup(self.PWM_A, GPIO.OUT)
        GPIO.setup(self.PWM_B, GPIO.OUT)
        GPIO.setup(self.PIN_A_PO,GPIO.OUT)
        GPIO.setup(self.PIN_A_NE,GPIO.OUT)
        GPIO.setup(self.PIN_B_PO,GPIO.OUT)
        GPIO.setup(self.PIN_B_NE,GPIO.OUT)
        
        self.pwm_a = GPIO.PWM(self.PWM_A, 10000)
        self.pwm_b = GPIO.PWM(self.PWM_B, 10000)

        GPIO.add_event_detect(self.encoder_right,GPIO.RISING, callback=self.encoder_read_right)
        GPIO.add_event_detect(self.encoder_left, GPIO.RISING, callback=self.encoder_read_left)

        self.pwm_a.start(0)
        self.pwm_b.start(0)

        GPIO.output(self.PIN_A_PO,GPIO.LOW)
        GPIO.output(self.PIN_A_NE,GPIO.LOW)
        GPIO.output(self.PIN_B_PO,GPIO.LOW)
        GPIO.output(self.PIN_B_NE,GPIO.LOW)
        
    def count_target(self,msg):
        linear = msg.linear.x
        angular = msg.angular.z
    
        self.right_target = (2*linear + 0.39*angular)/2
        self.left_target = (2*linear - 0.39*angular)/2
        
        if linear == 0:
            GPIO.output(self.PIN_A_PO,GPIO.LOW)
            GPIO.output(self.PIN_A_NE,GPIO.LOW)
            GPIO.output(self.PIN_B_PO,GPIO.LOW)
            GPIO.output(self.PIN_B_NE,GPIO.LOW)
        else:
            GPIO.output(self.PIN_A_PO,GPIO.HIGH)
            GPIO.output(self.PIN_A_NE,GPIO.LOW)
            GPIO.output(self.PIN_B_PO,GPIO.HIGH)
            GPIO.output(self.PIN_B_NE,GPIO.LOW)
        
        
    def PID_controller(self):
        self.PID_controller_left()
        self.PID_controller_right()
        print(self.status)
        print(f"l_target:{self.left_target} \t r_target:{self.right_target}")
        print(f"LEFT:{self.speed_Left}\t RIGHT:{self.speed_Right}")
        print(f'left_duty:{self.duty_left}\tright_duty:{self.duty_right}')
        self.status += 1
        self.left_speed_list.append(self.speed_Left)
        self.right_speed_list.append(self.speed_Right)

    def PID_controller_left(self):
        self.l_error = self.left_target -self.speed_Left
        if self.speed_Left != self.left_target:
            self.duty_left += self.MIN_MAX(int(self.l_error * self.kp + self.l_error_plus * self.ki+ (self.l_error-self.l_del_error)/0.1*self.kd),self.duty_left)
            self.pwm_a.ChangeDutyCycle(self.duty_left)
        self.l_error_plus += self.l_error
        self.l_del_error = self.l_error

    def PID_controller_right(self):
        self.r_error = self.right_target -self.speed_Right
        if self.speed_Right != self.right_target:
            self.duty_right += self.MIN_MAX(int(self.r_error * self.kp_r + self.r_error_plus * self.ki_r + (self.r_error-self.r_del_error)/0.1*self.kd_r),self.duty_right)
            self.pwm_b.ChangeDutyCycle(self.duty_right)
        self.r_error_plus += self.r_error
        self.r_del_error = self.r_error



    
    def MIN_MAX(self,value,duty):
        if duty + value < 100 and duty + value >0:
            return value
        else:
            return 0

    def encoder_read_left(self,channel):
        self.speed_counting_l()

    def encoder_read_right(self,channel):
        self.speed_counting_r()

    def speed_counting_l(self):
        current_time = time.time()
        time_elapsed = current_time - self.prev_left_time

        if time_elapsed>0.01:
            self.speed_Left = (0.8*abs(self.speed_Left)) + (0.2*(1 / time_elapsed) * ((0.1524*pi)/36))
        self.prev_left_time = current_time     


    def speed_counting_r(self):
        current_time = time.time()
        time_elapsed = current_time - self.prev_right_time

        if time_elapsed>0.01:
            self.speed_Right = (0.8*abs(self.speed_Right)) + (0.2*(1 / time_elapsed) * ((0.1524*pi)/36))
        self.prev_right_time = current_time 

def main(arg=None):
    try:
        rclpy.init()
        node = PID()
        rclpy.spin(node)
        rclpy.shutdown()
        
    finally:
        with open("/home/adan/left.txt",'w') as l:
            l.write(f"speed_l = {node.left_speed_list} \nspeed_r = {node.right_speed_list}")
        GPIO.cleanup()

