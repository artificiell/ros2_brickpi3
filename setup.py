import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ros2_brickpi3'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='andreas.persson@oru.se',
    description='ROS 2 - BrickPi3 wrapper',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "color = ros2_brickpi3.color:main",
            "drive = ros2_brickpi3.drive:main",
            "eyes = ros2_brickpi3.eyes:main",
            "gyro = ros2_brickpi3.gyro:main",
            "motor = ros2_brickpi3.motor:main",
            "touch = ros2_brickpi3.touch:main",
            "ultrasonic = ros2_brickpi3.ultrasonic:main"
        ],
    },
)
