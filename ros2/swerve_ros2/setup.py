from setuptools import setup
import os
from glob import glob

package_name = 'swerve_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abdu7rahman',
    maintainer_email='mohammedabdulr.1@northeastern.edu',
    description='Swerve drive inverse kinematics and gamepad teleop for ROS 2.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'swerve_controller = swerve_ros2.swerve_controller:main',
            'joy_teleop = swerve_ros2.joy_teleop:main',
        ],
    },
)
