import os

from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument , TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
   
  lidar_pkg_dir = "/home/adan/ros2_ws/src/sllidar_ros2/launch/"
  
  robot_launch_dir = "/home/adan/ros2_ws/src/fortest/launch/"
         
  motor_node = Node(
  package="fortest",
  executable="motor",
  output = "screen"
  )
  
  odom_estimate = Node(
  package="fortest",
  executable="odom_dect",
  output = "screen"
  )
  
  imu = Node(
  package="fortest",
  executable="imu",
  output="screen"
  )

  lidar_launch =   IncludeLaunchDescription(
  PythonLaunchDescriptionSource([lidar_pkg_dir,"sllidar_a1_launch.py"])
  )
  
  robot_joint = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [robot_launch_dir, '/eddiebot_state_publisher.launch.py'])
  )
  
###delay
  delay_imu = TimerAction(
	period = 3.0,
	actions=[imu]
	)
  delay_motor_node = TimerAction(
	period = 10.0,
	actions=[motor_node]
	)
  delay_robot_joint = TimerAction(
	period = 10.0,
	actions=[robot_joint]
	)
 
 
  return LaunchDescription([
  
  lidar_launch,
  delay_imu,
  #odom_estimate,
  delay_motor_node,
  delay_robot_joint,
  
  

  ])
