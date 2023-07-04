#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np

#  Class for handle Differential drive
class DifferentialDrive(Node):

    def __init__(self):
        super().__init__('differential_drive_controller')

        # Declare robot parameters
        self.declare_parameter('wheel_radius', 0.0275)
        self.declare_parameter('base_distance', 0.075)
        self.r = self.get_parameter('wheel_radius').value
        self.b = self.get_parameter('base_distance').value
        self.ticks_per_meter = 1.0 / (2 * np.pi * self.r) * 360
        
        # Setup ROS publishers
        self.left_motor_publisher_ = self.create_publisher(Int32, 'left/speed', 10)
        self.right_motor_publisher_ = self.create_publisher(Int32, 'right/speed', 10)
        self.odom_publisher_ = self.create_publisher(Odometry, 'odom', 10)

        # Setup ROS subscribers
        self.cmd_subscription = self.create_subscription(
            Twist,
            'cmd',
            self.cmd_callback,
            10
        )
        self.left_encoder_subscription = self.create_subscription(
            Int32,
            'left/encoder',
            self.left_encoder_callback,
            10
        )
        self.right_encoder_subscription = self.create_subscription(
            Int32,
            'right/encoder',
            self.right_encoder_callback,
            10
        )

        # Encoder and position variables
        self.left_encoder_value, self.right_encoder_value = None, None 
        self.left_encoder_previous, self.right_encoder_previous = None, None
        self.x, self.y, self.th  = 0.0, 0.0, 0.0  
        self.dx, self.dr = 0.0, 0.0
        self.then = self.get_clock().now()
        
        # Setup timed subscriber
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.odom_callback)

        
    # Receive and process movement commands, send speed messages to motors
    def cmd_callback(self, msg):

        # Get linear and rotational speed
        dx = msg.linear.x
        dr = msg.angular.z

        # Approximate the speed for each motor
        left_speed  = (dx - dr * self.b / 2 ) / self.r  # 1.0 * self.dx - self.dr * self.w / 2.0
        right_speed = (dx + dr * self.b / 2 ) / self.r   # 1.0 * dx + dr * self.w / 2.0

        # Convert to degrees per second (DPS)
        left_speed = left_speed * 180 / np.pi 
        right_speed = right_speed * 180 / np.pi 
        
        # Publish speed messsages to motors
        left_msg, right_msg = Int32(), Int32()
        left_msg.data = int(left_speed)
        right_msg.data = int(right_speed)        
        self.left_motor_publisher_.publish(left_msg)
        self.right_motor_publisher_.publish(right_msg)

        
    # Periodically send odometry readings
    def odom_callback(self):
        if self.left_encoder_value is not None and self.right_encoder_value is not None:
            now = self.get_clock().now()
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.nanoseconds / 1e9 

            # Calculate odometry
            if self.left_encoder_previous is None or  self.left_encoder_previous is None:
                d_left, d_right = 0.0, 0.0
            else:
                d_left = (self.left_encoder_value - self.left_encoder_previous) / self.ticks_per_meter
                d_right = (self.right_encoder_value - self.right_encoder_previous) / self.ticks_per_meter
            self.left_encoder_previous = self.left_encoder_value
            self.right_encoder_previous = self.right_encoder_value
            
            # Distance traveled (approximated by average travel distance of both wheels)
            d  = (d_left + d_right) / 2
            th = (d_right - d_left) / self.b
            x =  np.cos(th) * d
            y = -np.sin(th) * d
            self.th = self.th + th
            self.x = self.x + (np.cos(self.th) * x - np.sin(self.th) * y)
            self.y = self.y + (np.sin(self.th) * x + np.cos(self.th) * y)            
            
            # Publish odometry messsage
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = 'odom'
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = np.sin(self.th / 2)
            odom.pose.pose.orientation.w = np.cos(self.th / 2)
            odom.child_frame_id = 'base_link'
            odom.twist.twist.linear.x = d / elapsed # Linear velocity
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = th / elapsed # Angular velocity
            self.odom_publisher_.publish(odom)

    
    # Receive and store left motor encoder value
    def left_encoder_callback(self, msg):
        self.left_encoder_value = msg.data

    # Receive and store right motor encoder value
    def right_encoder_callback(self, msg):
        self.right_encoder_value = msg.data

        
# Main function
def main(args = None):
    rclpy.init(args = args)

    diff_drive = DifferentialDrive()

    try:
        rclpy.spin(diff_drive)
    except KeyboardInterrupt:
        pass
        
    # Destroy the node (explicitly)
    diff_drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
