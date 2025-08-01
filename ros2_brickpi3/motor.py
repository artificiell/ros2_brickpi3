#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from std_msgs.msg import Int32

import atexit
import brickpi3

# Class for handle EV3 Motor commands
class MotorController(Node):

    def __init__(self):
        super().__init__('lego_motor_controller')

        # Declare port parameter
        self.declare_parameter('port', 'A')
        self.declare_parameter('profile', 'reliable') # QoS profile: best_effort or reliable

        # Init BrickPi3 and set up motor port
        self.brick = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class.
        port = self.get_parameter('port').value
        if port == 'A':
            self.port = self.brick.PORT_A
        if port == 'B':
            self.port = self.brick.PORT_B
        elif port == 'C':
            self.port = self.brick.PORT_C
        elif port == 'D':
            self.port = self.brick.PORT_D
        self.get_logger().info(f"Motor ouput port: {port}")
        self.brick.reset_motor_encoder(self.port) 
        self.limit = 1000

        # Setup ROS publisher
        if self.get_parameter('profile').get_parameter_value().string_value.lower() == 'reliable':
            qos_profile = qos_profile_default
        else:
            qos_profile = qos_profile_sensor_data
            qos_profile.depth = 1
        self.publisher_ = self.create_publisher(Int32, 'encoder', qos_profile)

        # Setup ROS subscriber
        self.subscription = self.create_subscription(
            Int32,
            'speed',
            self.speed_callback,
            qos_profile_default
        )
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.encoder_callback)

        # Register reset method 
        atexit.register(self.reset)

    # Write and set motor speed
    def speed_callback(self, msg):
        try:
            speed = msg.data
            speed = self.limit if speed > self.limit else speed
            speed = -self.limit if speed < -self.limit else speed
            self.brick.set_motor_dps(self.port, speed) 
        except IOError as e:
            self.get_logger().error(f"Motor speed: {e}", throttle_duration_sec = 1)

    # Read and publish motor encoder value
    def encoder_callback(self):
        try:   
            msg = Int32()
            msg.data = self.brick.get_motor_encoder(self.port)
            self.publisher_.publish(msg)
        except IOError as e:
            self.get_logger().error(f"Motor encoder: {e}", throttle_duration_sec = 1)

    # Reset all motor ports
    def reset(self):
        try:
            self.brick.set_motor_dps(self.port, 0)
            self.brick.reset_motor_encoder(self.port)
        except IOError as e:
            self.get_logger().error(f"Motor reset: {e}", throttle_duration_sec = 1)
        finally:
            self.brick.set_motor_power(self.port, self.brick.MOTOR_FLOAT)
        

# Main function
def main(args = None):
    rclpy.init(args = args)
    motor_controller = MotorController()
    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
        
    # Destroy the node (explicitly)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
