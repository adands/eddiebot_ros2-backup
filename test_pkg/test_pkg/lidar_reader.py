#!/usr/bin/python3


import rclpy
from rclpy.node import Node
from rplidar import RPLidar
from  sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class lidar(Node):
  def __init__(self):
    super().__init__("lidar_node")
    self.publisher = self.create_publisher(LaserScan,"scan",10)
    self.get_logger().info("hello")
    self.timer = self.create_timer(1/5,self.lidar_read)
    self.scan = []
    self.ranges = []
    self.intensities = []
    self.lidar_usb = RPLidar("/dev/ttyUSB0")
    

  def testing(self):
    for i in range(5):
      print("hello")
      break

  def lidar_read(self):
    status = 0
    t = self.get_clock().now()
    scan_msg = LaserScan()

    scan_msg.header.stamp = t.to_msg()
    scan_msg.header.frame_id = "odom"

    scan_msg.angle_min = 0.0
    scan_msg.angle_min = 6.28
    scan_msg.angle_increment = 0.017453292500000002
    scan_msg.range_min = 0.15
    scan_msg.range_max = 12.0

    scan_msg.time_increment = 0.0
    scan_msg.scan_time = 0.0

    for i, scan in enumerate(self.lidar_usb.iter_measures()):
      self.scan.append(scan)
      print(status)
      if scan[0] == True:
        status += 1
        if len(self.ranges) == 0:
          self.ranges.append(scan[3])
          self.intensities.append(float(scan[1]))
      if status == 1:
        print(scan)
        self.ranges.append(scan[3])
        self.intensities.append(float(scan[1]))
      if status == 2:
        print("publish")
        scan_msg.ranges = self.ranges
        scan_msg.intensities = self.intensities
        self.publisher.publish(scan_msg)
        self.ranges = []
        self.intensities = []
        status = 0


def main(args = None):
  try:
    rclpy.init(args=args)
    node = lidar()
    rclpy.spin(node)
    rclpy.shutdown()
  finally:
    node.lidar_usb.stop()
    node.lidar_usb.clean_input()
    node.lidar_usb.stop_motor()
    node.lidar_usb.disconnect()
    
    
    