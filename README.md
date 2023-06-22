# ros2_brickpi3
Repository for ROS2-BrickPi3 wrapper.

## Prerequisites

- ROS 2 (Foxy or later)
- Python 3
- BrickPi3 library (`brickpi3`)

## Installation

1. Clone the repository to your ROS 2 workspace, e.g.:
```
cd ~/ros2_ws/src/
git clone https://github.com/artificiell/ros2_brickpi3.git
```

2. Build the ROS 2 workspace:
```
cd ~/ros2_ws/
colcon build 
```

3. Source the ROS 2 workspace:
```
source install/setup.bash
```


## Usage

To use any of the LEGO Sensor ROS 2 nodes, follow these steps:

1. Connect the sensor to the desired **sensor** port on the BrickPi3.

2. Launch the node, e.g., for the  LEGO Color Sensor:
```
ros2 run ros2_brickpi3 color --ros-args -p port:=1
```
3. Subscribe to the publishing topic to receive sensor values, e.g., topic "color" to receive color sensor values:
```
ros2 topic echo /color
```

To use the LEGO Motor Controller ROS 2 node, follow these steps:

1. Connect an EV3 motor to the desired **motor** port on the BrickPi3.

2. Launch the node:
```
ros2 run ros2_brickpi3 motor --ros-args -p port:=A
```

3. Publish the desired speed to the "speed" topic to control the motor. For example, to set the motor speed to 10 (forward direction):
```
ros2 topic pub speed std_msgs/Int32 "{data: 10}"
```

4. To stop the motor, publish a speed of 0:
```
ros2 topic pub speed std_msgs/Int32 "{data: 0}"
```

## Sensor Nodes

The following parameters apply to all sensor nodes:

- **`port`** (integer, default: 1): The port number on the BrickPi3 where the sensor is connected. Valid options are 1, 2, 3, and 4.


### Color Sensor (`color.py`)

This node reads the values from the LEGO Color Sensor and publishes them to the "color" topic. The color sensor values are published as strings representing the detected color. The possible color values are: "unknown", "black", "blue", "green", "yellow", "red", "white", and "brown".

#### Published Topics

- **`color`** (`std_msgs/String`): The color sensor value is published to this topic as a string.


### Gyro Sensor (`gyro.py`)

This ROS 2 node interfaces with a LEGO Gyro Sensor using the BrickPi3 library. It reads the gyro sensor values and publishes the rotation angle in radians to the "rotation" topic. The angle represents the rotation around the sensor's axis.

#### Published Topics

- **`rotation`** (`std_msgs/Float32`): The rotation angle measured by the gyro sensor is published to this topic as a float value in radians.


### Touch Sensor (`touch.py`) 

This ROS 2 node is a handler for the EV3 Touch sensor inputs using the BrickPi3 library. It reads the touch sensor state and publishes a boolean value indicating whether the touch sensor is pressed or released.

#### Published Topics

- **`pressed`** (`std_msgs/Bool`): This topic publishes a boolean value indicating the state of the touch sensor. When the touch sensor is pressed, the published value is `True`, and when the touch sensor is released, the published value is `False`.


### Ultrasonic Sensor (`ultrasonic.py`)

This ROS 2 node is a handler for the EV3 Ultrasonic sensor inputs using the BrickPi3 library. The node uses the BrickPi3 library to communicate with the ultrasonic sensor and read the distance measurement. It reads the distance measure by the ultrasonic sensor and publishes it as a floating-point value.

#### Published Topics

- **`distance`** (`std_msgs/Float32`): This topic publishes the distance measured by the ultrasonic sensor as a floating-point value in centimeters.


## Motor Nodes

### Motor Controller (`motor.py`)

This ROS 2 node serves as a controller for handling EV3 Motor commands using the BrickPi3 library. It receives speed commands on the `speed` topic and controls the motor accordingly. Positive values indicate forward motion, negative values indicate backward motion, and zero stops the motor. This node further reads the motor encoder value and publishes it on the `encoder` topic.

#### Subscribed Topics

- **`speed`** (`std_msgs/Int32`): This topic receives speed commands for controlling the motor. The speed command should be an integer value representing the desired speed of the motor in _Degrees Per Second_ (DPS). Positive values indicate forward motion, negative values indicate backward motion, and zero stops the motor.

#### Published Topics

- **`encoder`** (`std_msgs/Int32`): This topic publishes the current encoder value of the motor. The encoder value is an integer that represents the position or rotation of the motor.

#### Parameters

- **`port`** (string, default: 'A'): The output port on the BrickPi3 where the motor is connected. Valid options are 'A', 'B', 'C', and 'D'.


### Differential Drive Controller(`drive.py`)

This ROS 2 node implements a differential drive controller for a mobile robot. It receives movement commands and publishes speed messages to control the left and right motors. Additionally, it calculates and publishes odometry readings based on the motor encoder values.

#### Subscribed Topics

- **`cmd`** (`geometry_msgs/Twist`): The movement command for the robot. It contains linear and angular velocities.
- **`left/encoder`** (`std_msgs/Int32`): The encoder value of the left motor. It provides information about the position of the left motor.
- **`right/encoder`** (`std_msgs/Int32`): The encoder value of the right motor. It provides information about the position of the right motor.

#### Published Topics

- **`left/speed`** (`std_msgs/Int32`): The speed message to control the left motor. The value represents the speed in degrees per second (DPS).
- **`right/speed`** (`std_msgs/Int32`): The speed message to control the right motor. The value represents the speed in degrees per second (DPS).
- **`odom`** (`nav_msgs/Odometry`): The odometry readings of the robot. It provides information about the position and orientation of the robot, as well as linear and angular velocities.

#### Parameters

- **`wheel_radius`** (float, default: 0.0275): The radius of the wheels in meters.
- **`base_distance`** (float, default: 0.075): The distance between the two wheels (base) in meters.

#### Node Behavior
- Upon receiving a movement command (`cmd`), the node calculates the required left and right motor speeds to achieve the desired linear and angular velocities.
- The calculated speeds are published as speed messages to the respective motor topics (`left/speed` and `right/speed`).
- The node continuously listens to the motor encoder topics (`left/encoder` and `right/encoder`) to receive the updated encoder values.
- Periodically, the node calculates the odometry readings based on the motor encoder values, including the distance traveled, orientation, and velocity.
- The odometry readings are published as `nav_msgs/Odometry` messages on the `odom` topic

#### Troubleshooting

- If the robot is not moving as expected, verify that the wheel radius and base distance parameters are set correctly for your robot.
- Ensure that the motor encoder topics (`left/encoder` and `right/encoder`) publish the correct encoder values.


## License

This software is released under the MIT License. See the [LICENSE](LICENSE) file for more details.

