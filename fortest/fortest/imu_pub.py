#!usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3 ,Quaternion,TransformStamped, Transform
import board
import busio
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C
from tf_transformations import quaternion_from_euler
from math import *
import numpy as np
from tf2_ros import TransformBroadcaster
import math

class imu_pub(Node):

    def __init__(self):
        super().__init__("imu_pub")
        self.get_logger().info("start imu....")
        self.pub = self.create_publisher(Imu,"imu",10)
        self.get_logger().info("setting i2c....")
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.icm = BNO08X_I2C(self.i2c)
        self.icm.soft_reset()
        self.get_logger().info("setting imu....")
        self.icm.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
        self.icm.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self.icm.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        self.icm.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        self.icm.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        self.icm.enable_feature(adafruit_bno08x.BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)

        self.get_logger().info("start calibration....")
        self.timer0 = self.create_timer(1/1000,self.correction)

        self.timer = self.create_timer(1/200,self.imu_publish)
        self.timer.cancel()
        
        #self.timer2 = self.create_timer(1,self.imu_print)
        #self.tf_broadcaster = TransformBroadcaster(self)
        
        self.del_th_z = 0

        self.gyro_sum = 0.0
        self.gyro_offset = 0.0
        self.acc_sum = [0.0,0.0]
        self.acc_offset = [0.0,0.0]
        self.acc_x = np.array([],dtype = float)
        self.acc_y = np.array([],dtype = float)
        self.times = 0
        self.acc_list=[]
        
        self.sample_counter = 0
        
        #self.icm.initialize()


    def calculate_yaw(self, mag_x, mag_y):
        yaw = math.atan2(mag_y, mag_x)  
        yaw_degrees = math.degrees(yaw)  
        if yaw_degrees < 0:
            yaw_degrees += 360
        # self.del_th_z = yaw_degrees
        self.del_th_z = (self.del_th_z * 0.8) + (yaw_degrees   * 0.2)

    def correction(self):
        if self.sample_counter <= 8000:
            self.gyro_sum += self.icm.gyro[2]
            acc_data = self.icm.linear_acceleration
            self.acc_x = np.append(self.acc_x,acc_data[0])
            self.acc_y = np.append(self.acc_y,acc_data[1])
            self.sample_counter += 1
        else:
            self.gyro_offset = self.gyro_sum/self.sample_counter
            self.acc_offset[0] = np.mean(np.sort(self.acc_x)[1300:6700])
            self.acc_offset[1] = np.mean(np.sort(self.acc_y)[1300:6700])
            self.get_logger().info(f"acc_offset:{self.acc_offset} , gyro_offset:{self.gyro_offset}")
            self.get_logger().info("IMU Correction Done")
            self.timer0.cancel()
            self.timer.reset()

    def imu_publish(self):

        imu_msg=Imu()
        transform = TransformStamped()
        t = self.get_clock().now()
        acc = self.icm.linear_acceleration
        ang = self.icm.gyro
        #mag = self.icm.magnetic
        orien = self.icm.quaternion

        imu_msg.header.stamp = t.to_msg()
        imu_msg.header.frame_id = "imu_link"
        transform.header.stamp = t.to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = "imu_link"

        #self.calculate_yaw(mag[0],mag[1])
        #print(self.del_th_z)

        imu_msg.orientation.x = orien[0]
        imu_msg.orientation.y = orien[1]
        imu_msg.orientation.z = orien[2]
        imu_msg.orientation.w = orien[3]

        imu_msg.angular_velocity.z = ang[2] - self.gyro_offset
        imu_msg.linear_acceleration.x = acc[0] - self.acc_offset[0]
        imu_msg.linear_acceleration.y = acc[1] - self.acc_offset[1]

        self.pub.publish(imu_msg)
        self.acc_list.append(acc[0] - self.acc_offset[0])

        #self.tf_broadcaster.sendTransform(transform)



    def imu_print(self):
        print("Acceleration: X:%.5f, Y: %.5f, Z: %.5f m/s^2" % (self.icm.acceleration))
        print("Gyro X:%.2f, Y: %.2f, Z: %.5f rads/s" % (self.icm.gyro))
        #print("Magnetometer X:%.2f, Y: %.2f, Z: %.2f uT" % (self.icm.magnetic))



def main(args=None):
    try:
        rclpy.init(args=args)
        node = imu_pub()
        rclpy.spin(node)
        rclpy.shutdown()

    finally:
        with open("/home/adan/acc.txt","w") as f:
            f.write(f"{node.acc_list}")
        pass
