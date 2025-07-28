#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from std_msgs.msg import String

import atexit
import brickpi3
import json

# Class for handle  Mindsensors SumoEyes sensor inputs
class EyesSensor(Node):

    # Declare constants
    EYES_SHORT_RANGE = 0x0101
    EYES_LONG_RANGE  = 0x0110

    def __init__(self):
        super().__init__('lego_eyes_sensor')

        # Declare port parameter
        self.declare_parameter('port', 1)
        self.declare_parameter('mode', 'short')
        self.declare_parameter('profile', 'reliable') # QoS profile: best_effort or reliable
        
        # Init BrickPi3 instance and set up sensor port
        self.brick = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class.
        port = self.get_parameter('port').value
        if port == 1:
            self.port = self.brick.PORT_1
        if port == 2:
            self.port = self.brick.PORT_2
        elif port == 3:
            self.port = self.brick.PORT_3
        elif port == 4:
            self.port = self.brick.PORT_4
        self.get_logger().info(f"Eyes sensor input port: {port}")

        mode = self.get_parameter('mode').value
        if mode == 'short':
            self.set_mode(self.EYES_SHORT_RANGE)
        else:
            self.set_mode(self.EYES_LONG_RANGE)
        self.get_logger().info(f"Eyes sensor range mode: {mode}")
        
        # Setup ROS publisher
        if self.get_parameter('profile').get_parameter_value().string_value.lower() == 'reliable':
            qos_profile = qos_profile_default
        else:
            qos_profile = qos_profile_sensor_data
            qos_profile.depth = 1
        self.publisher_ = self.create_publisher(String, 'obstacles', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

        # Register reset method 
        atexit.register(self.reset)
        
    # Set SumoEyes sensor input mode
    def set_mode(self, val):
        mode = [(self.brick.SENSOR_CUSTOM.PIN1_ADC | val)]
        self.brick.set_sensor_type(self.port, self.brick.SENSOR_TYPE.CUSTOM, mode)
                
    # Read and publish sensor value
    def callback(self):
        try:
            obstacles = { 'left': 0, 'front': 0, 'right': 0 }

            # Get voltage reference
            ref = (4095.0 / self.brick.get_voltage_5v())
            
            # Get range reading
            val = self.brick.get_sensor(self.port)[0]
            val = val / (4095.0 / ref)
            if val < 800.0:
                if val < 300.0:
                    obstacles['front'] = 1
                elif val < 400.0:
                    obstacles['right'] = 1
                elif val < 600.0:
                    obstacles['left'] = 1

            # Publish obstacle dict (as string)
            msg = String()
            msg.data = json.dumps(obstacles)
            self.publisher_.publish(msg)
            
        except brickpi3.SensorError as e:
            self.get_logger().error(f"Eyes sensor: {e}", throttle_duration_sec = 1)
            
    # Reset sensor port
    def reset(self):
        self.brick.set_sensor_type(self.port, self.brick.SENSOR_TYPE.NONE)
 
                                                                                                                                    
# Main function
def main(args = None):
    rclpy.init(args = args)
    eyes_sensor = EyesSensor()
    try:
        rclpy.spin(eyes_sensor)
    except KeyboardInterrupt:
        pass
        
    # Destroy the node (explicitly)
    eyes_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
