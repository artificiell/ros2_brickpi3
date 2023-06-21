#!/usr/bin/env python3
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

def generate_launch_description():

    # Launch configuration
    robot_ns = LaunchConfiguration('robot_ns')
    left_motor_port = LaunchConfiguration('left_motor_port')
    right_motor_port = LaunchConfiguration('right_motor_port')

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
    
    # Left motor node
    left_motor = Node(
        package = 'ros2_brickpi3',
        namespace = robot_ns,
        executable = 'motor',
        name = 'left_motor_node',
        parameters=[
            {'port': LaunchConfiguration('left_motor_port')}
        ],      
        remappings=[
            ('speed', 'left/speed'),
            ('encoder', 'left/encoder'),
        ]
    )

    # Right motor node
    right_motor = Node(
        package = 'ros2_brickpi3',
        namespace = robot_ns,
        executable = 'motor',
        name = 'right_motor_node',
        parameters=[
            {'port': LaunchConfiguration('right_motor_port')}
        ],      
        remappings=[
            ('speed', 'right/speed'),
            ('encoder', 'right/encoder'),
        ]
    )
    
    return LaunchDescription([
        robot_ns_launch_arg,
        left_motor_port_launch_arg,
        right_motor_port_launch_arg,
        left_motor,
        right_motor
    ])
