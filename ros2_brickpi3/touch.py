#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_default, qos_profile_sensor_data
from std_msgs.msg import Bool

import atexit
import brickpi3

# Class for handle EV3 Touch sensor inputs
class TouchSensor(Node):

    def __init__(self):
        super().__init__('lego_touch_sensor')

        # Declare port parameter
        self.declare_parameter('port', 1)
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
        self.get_logger().info(f"Touch sensor input port: {port}")
        self.brick.set_sensor_type(self.port, self.brick.SENSOR_TYPE.TOUCH)
        
        # Setup ROS publisher
        if self.get_parameter('profile').get_parameter_value().string_value.lower() == 'reliable':
            qos_profile = qos_profile_default
        else:
            qos_profile = qos_profile_sensor_data
            qos_profile.depth = 1
        self.publisher_ = self.create_publisher(Bool, 'pressed', qos_profile)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

        # Register reset method 
        atexit.register(self.reset)

    # Read and publish sensor value
    def callback(self):
        try:
            msg = Bool()
            msg.data = True if self.brick.get_sensor(self.port) else False 
            self.publisher_.publish(msg)
        except brickpi3.SensorError as e:
            self.get_logger().error(f"Touch sensor: {e}", throttle_duration_sec = 1)
            self.brick.set_sensor_type(self.port, self.brick.SENSOR_TYPE.TOUCH)

    # Reset sensor port
    def reset(self):
        self.brick.set_sensor_type(self.port, self.brick.SENSOR_TYPE.NONE)

                                                                                                                                    
# Main function
def main(args = None):
    rclpy.init(args = args)

    touch_sensor = TouchSensor()

    try:
        rclpy.spin(touch_sensor)
    except KeyboardInterrupt:
        pass
        
    # Destroy the node (explicitly)
    touch_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
