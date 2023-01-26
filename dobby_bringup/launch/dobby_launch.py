#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    LDS_LAUNCH_FILE = '/ydlidar_launch.py' 
    MOTOR_LAUNCH_FILE = '/doro_mobile_robot_node.launch.py'


    # dobby_param_dir = LaunchConfiguration(
    #     'dobby_param_dir',
    #     default=os.path.join(
    #         get_package_share_directory('dobby_bringup'),
    #         'param',
    #         'dobby.yaml'))

    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'launch'))

    motor_pkg_dir = LaunchConfiguration(
        'motor_pkg_dir',
        default=os.path.join(get_package_share_directory('doro_mobile'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value=use_sim_time,
        #     description='Use simulation (Gazebo) clock if true'),

        # DeclareLaunchArgument(
        #     'usb_port',
        #     default_value=usb_port,
        #     description='Connected USB port with OpenCR'),

        # DeclareLaunchArgument(
        #     'dobby_param_dir',
        #     default_value=dobby_param_dir,
        #     description='Full path to dobby parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/dobby_state_publisher_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            # launch_arguments={'port': '/dev/ttyUSB0','frame_id': 'lidar_link'}.items(), #안되면, ydlidar_ros2_driver/param 파일에서  frame_id를 lidar_link로 변경필요 
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([motor_pkg_dir, MOTOR_LAUNCH_FILE]),
        ),

        Node(
            package='dobby_node',
            executable='odom_pub',
            output='screen'),

        Node(
            package='dobby_node',
            executable='odom_to_tf',
            output='screen'),
    ])