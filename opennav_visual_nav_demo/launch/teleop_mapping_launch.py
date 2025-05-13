# Copyright (c) 2025 Open Navigation LLC
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Nova Carter hardware to be teleoperated.
        # Launches the robot hardware, URDF, joystick node, and muxer node.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nova_carter_bringup'),
                    'launch',
                    'teleop.launch.py'
                ])
            ]),
            launch_arguments={
                'mode': 'real_world',
                'enable_3d_lidar': 'false',
                'enabled_2d_lidars': 'none',
                'enabled_fisheye_cameras': 'none', 
                'enabled_stereo_cameras': 'front_stereo_camera,left_stereo_camera,right_stereo_camera,back_stereo_camera',
            }.items()
        ),

        # Launches the data recorder node to record the robot data.
        # This records only the IMU, stereo cameras, and odometry data (no lidar, no owls).
        TimerAction(
            period=10.0,  # delay launch for the robot data to be ready
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('isaac_ros_nova_recorder'),
                            'launch',
                            'nova_recorder.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'headless': 'true',
                        'config': 'nova-carter_hawk-4_no-3d-lidar',
                        'recording_directory': '/mnt/nova_ssd/recordings',
                    }.items()
                )
            ]
        )
    ])
