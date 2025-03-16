# Copyright 2019 Intelligent Robotics Lab
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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('jigless-planner')
    
    # bottom_controller = Node(
    #     package='jigless-planner',
    #     executable='bottom_controller_node',
    #     name='validate',
    #     namespace='bottom_planner',
    #     output='screen',
    #     parameters=[]
    # )
    top_controller = Node(
    package='jigless-planner',
    executable='top_controller_node',
    name='top_controller',
    namespace='top_planner',
    output='screen',
    parameters=[]
    )
 # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(top_controller)
    return ld
