import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument , TimerAction
from launch.actions import IncludeLaunchDescription 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import  FindPackageShare

def generate_launch_description():
  
  map_name = input("\nPlease enter your map name ex: my_world\n")
  map_dir = f"/home/adan/map/{map_name}.yaml"
  param_file = "/home/adan/ros2_ws/src/mybot/param/eddie.yaml"
  
  ## launch file directory
  eddiebot_dir = "/home/adan/ros2_ws/src/fortest/launch/"
  pkg_share = FindPackageShare(package="mybot").find("mybot")

  ## include launch file
  #    motor controller, lidar and joint_state_publisher
  eddiebot =   IncludeLaunchDescription(
  PythonLaunchDescriptionSource([eddiebot_dir,"eddiebot.launch.py"])
  )

  #    na2_bringup navigation_launch.py
  navigation = IncludeLaunchDescription(
  PythonLaunchDescriptionSource(["/opt/ros/humble/share/nav2_bringup/launch/","bringup_launch.py"]),
  launch_arguments={
      "map": map_dir,
      "params_file": param_file,
      "use_sim_time":"false",
      }.items()
  )
  nmea_navsat = IncludeLaunchDescription(
  PythonLaunchDescriptionSource(["/home/adan/ros2_ws/src/nmea_navsat_driver/launch","nmea_serial_driver.launch.py"]),
  launch_arguments={
      "remappings":[("/fix","/gps/fix")]
  
      }.items()
  )
  
  ##  node
  
  robot_localization = Node(
  package="robot_localization",
  executable="ekf_node",
  name="ekf_local_node",
  output="screen",
  parameters=["/home/adan/ros2_ws/src/mybot/config/ekf_local.yaml"],
  arguments=["--ros-args","--param","use_sim_time:=false"],
  remappings=[
      ("/odometry/filtered","/odometry/filtered/local    ")
      ]
  )

  robot_localization2 = Node(
  package="robot_localization",
  executable="ekf_node",
  name="ekf_global_node",
  output="screen",
  parameters=["/home/adan/ros2_ws/src/mybot/config/ekf_global.yaml"],
  arguments=["--ros-args","--param","use_sim_time:=false"],
  remappings=[
      ("/odometry/filtered","/odometry/filtered/global")
      ]
  )  
  
  navsat_transform = Node(
  package="robot_localization",
  executable="navsat_transform_node",
  name="navsat_transform_node",
  output="screen",
  arguments=["--ros-args","--param","use_sim_time:=false"],
  remappings=[
      ("/odometry/filtered","/odometry/filtered/global")
      ]
  
  )
  
 ###delay
 
  delay_navigation = TimerAction(
	period = 20.0,
	actions=[navigation]
	)
  delay_robot_localization = TimerAction(
	period = 12.0,
	actions=[robot_localization]
	) 
  delay_robot_localization2 = TimerAction(
	period = 13.0,
	actions=[robot_localization2]
	) 
  delay_navsat_transform =TimerAction(
    period = 12.0,
    actions=[navsat_transform]
  )
  delay_nmea_navsat = TimerAction(
    period = 10.0,
    actions = [nmea_navsat]
  )
  
  return LaunchDescription([

  
  eddiebot,
  delay_nmea_navsat,
  delay_robot_localization,
  delay_navsat_transform,
  delay_robot_localization2,
  delay_navigation,
  ])
