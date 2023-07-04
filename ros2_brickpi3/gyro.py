#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import brickpi3
import numpy as np

# Class for handle EV3 Gyro sensor inputs
class GyroSensor(Node):

    def __init__(self):
        super().__init__('lego_gyro_sensor')

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
        self.get_logger().info(f"Gyro sensor input port: {port}")
        self.brick.set_sensor_type(self.port, self.brick.SENSOR_TYPE.EV3_GYRO_ABS)
        
        # Setup ROS publisher
        self.publisher_ = self.create_publisher(Float32, 'rotation', 1)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.callback)
        self.last_val = None
        
    # Read and publish sensor value
    def callback(self):
        try:
            this_val = self.brick.get_sensor(self.port)
            if self.last_val is None:
                self.last_val = this_val
            else:
                if this_val == 0 and abs(this_val - self.last_val) > 1:
                    this_val = self.last_val
                
            msg = Float32()
            msg.data = -this_val * np.pi / 180.0            
            while msg.data < -np.pi * 2.:
                msg.data += np.pi * 2.
            while msg.data > np.pi * 2.:
                msg.data -= np.pi * 2.
            if msg.data > np.pi:
                msg.data =  (msg.data - np.pi) - np.pi
            if msg.data < -np.pi:
                msg.data =  np.pi + (msg.data + np.pi)
            self.publisher_.publish(msg)
            self.last_val = this_val
            
        except brickpi3.SensorError as e:
            self.get_logger().error(f"Gyro sensor: {e}", throttle_duration_sec = 1)

    # Reset sensor port
    def reset(self):
        self.brick.set_sensor_type(self.port, self.brick.SENSOR_TYPE.NONE)
 
                                                                                                                                    
# Main function
def main(args = None):
    rclpy.init(args = args)

    gyro_sensor = GyroSensor()

    try:
        rclpy.spin(gyro_sensor)
    except KeyboardInterrupt:
        pass
        
    # Stop the sensor and destroy the node (explicitly)
    gyro_sensor.reset()
    gyro_sensor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
