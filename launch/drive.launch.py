#!/usr/bin/env python3
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    # Launch configuration
    robot_ns = LaunchConfiguration('robot_ns')
    left_motor_port = LaunchConfiguration('left_motor_port')
    right_motor_port = LaunchConfiguration('right_motor_port')
    wheel_radius = LaunchConfiguration('wheel_radius')
    base_distance = LaunchConfiguration('base_distance')

    # Launch arguments
    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value = 'rp0'
    )
    left_motor_port_launch_arg = DeclareLaunchArgument(
        'left_motor_port',
        default_value = 'C'
    )
    right_motor_port_launch_arg = DeclareLaunchArgument(
        'right_motor_port',
        default_value = 'B'
    )
    wheel_radius_launch_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value = '0.0275'
    )
    base_distance_launch_arg = DeclareLaunchArgument(
        'base_distance',
        default_value = '0.075'
    )
    
    # Launch motors
    motors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros2_brickpi3'),
                'launch',
                'motors.launch.py'
            ])
        ]),
        launch_arguments = {
            'robot_ns': LaunchConfiguration('robot_ns'),
            'left_motor_port': LaunchConfiguration('left_motor_port'),
            'right_motor_port': LaunchConfiguration('right_motor_port')
        }.items()
    )

    # Differential drive node
    drive = Node(
        package = 'ros2_brickpi3',
        namespace = robot_ns,
        executable = 'drive',
        name = 'differential_drive_controller',
        parameters=[{
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'base_distance': LaunchConfiguration('base_distance')
        }]
    )
    
    return LaunchDescription([
        robot_ns_launch_arg,
        left_motor_port_launch_arg,
        right_motor_port_launch_arg,
        wheel_radius_launch_arg,
        base_distance_launch_arg,
        motors,
        drive
    ])
