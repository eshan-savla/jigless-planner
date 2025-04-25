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
    bottom_ns = LaunchConfiguration('bottom_ns')
    top_ns = LaunchConfiguration('top_ns')
    bottom_name = "bottom_controller"

    # Declare namespaces
    declare_bottom_ns_cmd = DeclareLaunchArgument(
        'bottom_ns',
        default_value='bottom_planner',
        description='Namespace')

    declare_top_ns_cmd = DeclareLaunchArgument(
        'top_ns',
        default_value='top_planner',
        description='Namespace')

    # Specify launch bringups
    
    transit_dummy = Node(
        package='jigless-planner',
        executable='transit_dummy_node',
        name='transit_dummy_server',
        namespace=bottom_ns,
        output='screen',
        parameters=[]
    )
    
    weld_dummy = Node(
        package='jigless-planner',
        executable='weld_dummy_node',
        name='weld_dummy_server',
        namespace=bottom_ns,
        output='screen',
        parameters=[]
    )

    validate_dummy = Node(
        package='jigless-planner',
        executable='validate_dummy_node',
        name='validate_dummy_server',
        namespace=bottom_ns,
        output='screen',
        parameters=[]
    )

 # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_bottom_ns_cmd)
    ld.add_action(declare_top_ns_cmd)

    ld.add_action(transit_dummy)
    ld.add_action(weld_dummy)
    ld.add_action(validate_dummy)
    return ld
