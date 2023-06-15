#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

import brickpi3

# Controller for handle EV3 Motor commands
class MotorController(Node):

    def __init__(self):
        super().__init__('lego_motor_controller')

        # Setup ROS subscriber
        self.subscription = self.create_subscription(
            Int32,
            'speed',
            self.callback,
            10
        )

        # Declare port parameter
        self.declare_parameter('port', 'A')

        # Setup BrickPi3
        self.brick = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class.
        port = self.get_parameter('port').value
        if port == 'A':
            self.port = self.brick.PORT_B
        if port == 'B':
            self.port = self.brick.PORT_B
        elif port == 'C':
            self.port = self.brick.PORT_C
        elif port == 'D':
            self.port = self.brick.PORT_D
        self.get_logger().info(f"Motor ouput port: {port}")
        #self.brick.set_motor_power(self.port, self.brick.MOTOR_FLOAT)  # float motor 
 
    def callback(self, msg):
        speed = msg.data
        if speed > 100:
            speed = 100
        elif speed < -100:
            speed = -100
        self.brick.set_motor_power(self.port, speed)

    def stop(self):
        self.brick.set_motor_power(self.port, 0)
        self.brick.reset_all()
        

# Main function
def main(args = None):
    rclpy.init(args = args)

    motor_controller = MotorController()

    try:
        rclpy.spin(motor_controller)
    except KeyboardInterrupt:
        pass
        
    # Stop the motor and destroy the node (explicitly)
    motor_controller.stop()
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
