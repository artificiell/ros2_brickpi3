#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import brickpi3

# Class for handle EV3 Touch sensor inputs
class TouchSensor(Node):

    def __init__(self):
        super().__init__('lego_touch_sensor')

        # Declare port parameter
        self.declare_parameter('port', 1)
        
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
        self.publisher_ = self.create_publisher(Bool, 'pressed', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

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
        
    # Stop the sensor and destroy the node (explicitly)
    touch_sensor.stop()
    touch_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
