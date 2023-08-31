import os
from glob import glob
from setuptools import setup

package_name = 'lightrover_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'configuration_files'), glob('configuration_files/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vstone',
    maintainer_email='infodesk@vstone.co.jp',
    description='ROS2 samples for LightRover',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'i2c_controller = lightrover_ros.i2c_controller:wrc201_i2c_server',
            'odom_manager = lightrover_ros.odometry:lightrover_odometry',
            'pos_controller = lightrover_ros.pos_controller:pos_cntrl',
            'rover_gamepad = lightrover_ros.rover_gamepad:rover_gamepad',
        ],
    },
)
