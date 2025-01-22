import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument , TimerAction
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import  FindPackageShare

def generate_launch_description():
  
  ## launch file directory
  eddiebot_dir = "/home/adan/ros2_ws/src/fortest/launch/"
  pkg_share = FindPackageShare(package="mybot").find("mybot")

  ## include launch file
  #    motor controller, lidar and joint_state_publisher
  eddiebot =   IncludeLaunchDescription(
  PythonLaunchDescriptionSource([eddiebot_dir,"eddiebot.launch.py"])
  )

  #    slam_toolbox
  slam_toolbox = IncludeLaunchDescription(
  PythonLaunchDescriptionSource(["/opt/ros/humble/share/slam_toolbox/launch/","online_async_launch.py"])
  )
  #    na2_bringup navigation_launch.py
  navigation = IncludeLaunchDescription(
  PythonLaunchDescriptionSource(["/opt/ros/humble/share/nav2_bringup/launch/","navigation_launch.py"])
  )
  
  ##  node
  
  robot_localization = Node(
  package="robot_localization",
  executable="ekf_node",
  name="ekf_local_node",
  output="screen",
  parameters=["/home/adan/ros2_ws/src/mybot/config/ekf_local.yaml"],
  arguments=["--ros-args","--param","use_sim_time:=false"]
  )
  
  
 ###delay
 
  delay_slam_toolbox = TimerAction(
	period = 30.0,
	actions=[slam_toolbox]
	)
  delay_robot_localization = TimerAction(
	period = 25.0,
	actions=[robot_localization]
	) 
 
  return LaunchDescription([

  
  eddiebot,
  #navigation,
  delay_robot_localization,
  delay_slam_toolbox,
  ])
