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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                get_package_share_directory('opennav_visual_nav_demo'),
                'demo_params.yaml'
            ]),
        ),

        DeclareLaunchArgument(
            'maps_directory',
            default_value='/mnt/nova_ssd/maps',
            description='Path to the occupancy map yaml file'
        ),

        DeclareLaunchArgument(
            'map_yaml_file',
            default_value='occupancy_map.yaml',
            description='Name of the occupancy map yaml file'
        ),

        # Nova Carter hardware, Nav2 stack, and Perceptor
        # cuVSLAM first localizes within the existing map using the pose hint provided by cuVGL.
        # Once localization is successful, the pose output from cuVSLAM can be used for navigation,
        # which is also launched here.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    get_package_share_directory('nova_carter_bringup'),
                    'launch',
                    'navigation.launch.py'
                ])
            ]),
            launch_arguments={
                'mode': 'real_world',
                'navigation_parameters_path': os.path.join(LaunchConfiguration('params_file')),
                'map_yaml_path': os.path.join(LaunchConfiguration('maps_directory'), LaunchConfiguration('map_yaml_file')),
                'occupancy_map_yaml_file': os.path.join(LaunchConfiguration('maps_directory'), LaunchConfiguration('map_yaml_file')),
                'vslam_load_map_folder_path': os.path.join(LaunchConfiguration('maps_directory'), 'cuvslam_map'),
                'vgl_map_dir':  os.path.join(LaunchConfiguration('maps_directory'), 'cuvgl_map'),
                'enable_docking': 'true',
                'enable_3d_lidar_localization': 'false',
                'enable_visual_localization': 'true',
                'stereo_camera_configuration': 'front_left_right_configuration',  # front_configuration
                'enable_nvblox_costmap': 'true',
                'enable_2d_lidar_costmap': 'false',
                'enable_3d_lidar_costmap': 'false',
                'enable_navigation': 'true',
                'vslam_enable_slam': 'true',
                'disable_vgl': 'false',
            }.items()
        ),

        # Demonstration autonomy route navigation system
        Node(
            package='opennav_visual_nav_demo',
            executable='patrol_application',
            name='patrol_application',
            output='screen',
            parameters=[LaunchConfiguration('params_file')]
        )
    ])
