#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from std_msgs.msg import Int16, String

import atexit
import brickpi3

# Class for handle EV3 Color sensor inputs
class ColorSensor(Node):

    def __init__(self):
        super().__init__('lego_color_sensor')

        # Declare mode and port parameters
        self.declare_parameter('mode', 'color')
        self.declare_parameter('port', 1)
        self.declare_parameter('profile', 'best_effort') # QoS profile: best_effort or reliable
                
        # Initialize BrickPi3 instance and set up sensor port
        self.brick = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class.
        self.mode = self.get_parameter('mode').value
        port = self.get_parameter('port').value
        if port == 1:
            self.port = self.brick.PORT_1
        if port == 2:
            self.port = self.brick.PORT_2
        elif port == 3:
            self.port = self.brick.PORT_3
        elif port == 4:
            self.port = self.brick.PORT_4
        self.get_logger().info(f"Color sensor input port: {port}")
        self.get_logger().info(f"Color sensor mode: {self.mode}")
        self.init() # Initialize sensor
          
        # Setup ROS publisher
        if self.get_parameter('profile').get_parameter_value().string_value.lower() == 'reliable':
            qos_profile = qos_profile_default
        else:
            qos_profile = qos_profile_sensor_data
            qos_profile.depth = 1
        if self.mode == 'ambient' or self.mode == 'reflected':
            self.publisher_ = self.create_publisher(Int16, 'color', qos_profile)
        else:
            self.publisher_ = self.create_publisher(String, 'color', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.callback)
        self.colors = ["unknown", "black", "blue", "green", "yellow", "red", "white", "brown"]

        # Register reset method 
        atexit.register(self.reset)
        
    # Initialize sensor according to sensor mode
    def init(self):
        if self.mode == 'ambient':
            self.brick.set_sensor_type(self.port, self.brick.SENSOR_TYPE.EV3_COLOR_AMBIENT)
        elif self.mode == 'reflected':
            self.brick.set_sensor_type(self.port, self.brick.SENSOR_TYPE.EV3_COLOR_REFLECTED)
        else:
            self.brick.set_sensor_type(self.port, self.brick.SENSOR_TYPE.EV3_COLOR_COLOR)
            
    # Read and publish sensor value
    def callback(self):
        try:
            val = self.brick.get_sensor(self.port)
            if self.mode == 'color' and val >= 0 and val < len(self.colors):
                msg = String()
                msg.data = self.colors[val]
                self.publisher_.publish(msg)
            else:
                msg = Int16()
                msg.data = val
                self.publisher_.publish(msg)
        except brickpi3.SensorError as e:
            self.get_logger().error(f"Color sensor: {e}", throttle_duration_sec = 1)
            self.init() # Re-initialize sensor
            
    # Reset sensor port
    def reset(self):
        self.brick.set_sensor_type(self.port, self.brick.SENSOR_TYPE.NONE)

        
# Main function
def main(args = None):
    rclpy.init(args = args)
    color_sensor = ColorSensor()
    try:
        rclpy.spin(color_sensor)
    except KeyboardInterrupt:
        pass
        
    # Destroy the node (explicitly)
    color_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
