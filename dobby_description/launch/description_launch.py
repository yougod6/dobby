import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    urdf_path = PathJoinSubstitution(
        [FindPackageShare("dobby_description"),
         "urdf", "dobby_description.xacro"]
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('dobby_description'),
         'rviz', 'description.rviz']
    )

    urdf_arg = DeclareLaunchArgument(
        name='urdf',
        default_value=urdf_path,
        description='URDF path'
    )

    joint_state_arg = DeclareLaunchArgument(
        name='publish_joints',
        default_value='true',
        description='Launch joint_states_publisher'
    )

    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='true',
        description='Run rviz'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'robot_description': Command(['xacro ', LaunchConfiguration('urdf')])
                }
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(LaunchConfiguration("publish_joints"))
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )


    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        urdf_arg,
        joint_state_arg,
        rviz_arg,
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
