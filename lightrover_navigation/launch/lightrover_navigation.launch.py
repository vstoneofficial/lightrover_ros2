# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    launch_base = LaunchConfiguration('launch_base', default='true')
    launch_rviz = LaunchConfiguration('launch_rviz', default='true')
    use_composition = LaunchConfiguration('use_composition', default='False')
    set_initial_pose = LaunchConfiguration('set_initial_pose', default='false')
    initial_pose_x = LaunchConfiguration('initial_pose_x', default='0.0')
    initial_pose_y = LaunchConfiguration('initial_pose_y', default='0.0')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw', default='0.0')
    initial_pose_wait_sec = LaunchConfiguration('initial_pose_wait_sec', default='5.0')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('lightrover_navigation'),
            'maps',
            'test.yaml'))

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('lightrover_navigation'),
            'config',
            'nav2_params.yaml'))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('lightrover_navigation'), 'launch')
    base_launch_file = os.path.join(
        get_package_share_directory('lightrover_ros'),
        'launch',
        'nav_base.launch.py')

    rviz_config_dir = os.path.join(
        get_package_share_directory('lightrover_navigation'),
        'rviz',
        'nav2.rviz')

    return LaunchDescription([  
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'launch_base',
            default_value='true',
            description='Launch robot base nodes needed for odometry, lidar, and motor control'),

        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz if true'),

        DeclareLaunchArgument(
            'use_composition',
            default_value='False',
            description='Launch Nav2 without composition by default for easier debugging'),

        DeclareLaunchArgument(
            'set_initial_pose',
            default_value='false',
            description='Set AMCL initial pose automatically after startup'),

        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Initial pose x in map frame'),

        DeclareLaunchArgument(
            'initial_pose_y',
            default_value='0.0',
            description='Initial pose y in map frame'),

        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Initial pose yaw in radians'),

        DeclareLaunchArgument(
            'initial_pose_wait_sec',
            default_value='5.0',
            description='Seconds to wait before sending initial pose'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch_file),
            condition=IfCondition(launch_base),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir,
                'use_composition': use_composition}.items(),
        ),
        Node(
            package='lightrover_ros',
            executable='set_initial_pose',
            name='set_initial_pose',
            output='screen',
            parameters=[{
                'frame_id': 'map',
                'x': initial_pose_x,
                'y': initial_pose_y,
                'yaw': initial_pose_yaw,
                'wait_sec': initial_pose_wait_sec,
                'use_sim_time': use_sim_time,
            }],
            condition=IfCondition(set_initial_pose)),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
            additional_env={
                'LIBGL_ALWAYS_SOFTWARE': '1',
                'QT_X11_NO_MITSHM': '1',
                'MESA_GL_VERSION_OVERRIDE': '3.3',
            },
            condition=IfCondition(launch_rviz)),
    ])
