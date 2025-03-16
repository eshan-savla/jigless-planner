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
    plansys2_bottom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': 
                    example_dir + '/pddl/transit_domain.pddl:' + 
                    example_dir + '/pddl/weld_domain.pddl',
          'namespace': bottom_ns
          }.items())
    
    plansys2_top = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file':  
                    example_dir + '/pddl/top_domain.pddl',
          'namespace': top_ns
          }.items())

    # Specify the bottom actions
    transit_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='transit',
        namespace=bottom_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'transit',
                'bt_xml_file': example_dir + '/behavior_trees_xml/transit.xml'
            }
        ])

    weld_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='weld',
        namespace=bottom_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'weld',
                'bt_xml_file': example_dir + '/behavior_trees_xml/weld.xml'
            }
        ])
    
    validate_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='validate',
        namespace=bottom_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'validate',
                'bt_xml_file': example_dir + '/behavior_trees_xml/validate.xml'
            }
        ])
    
    bottom_controller = Node(
        package='jigless-planner',
        executable='bottom_controller_node',
        name='bottom_controller',
        namespace=bottom_ns,
        output='screen',
        parameters=[]
    )   
    
    # Specify the top actions

    command_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='command',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'command',
                'bt_xml_file': example_dir + '/behavior_trees_xml/command.xml'
            }
        ])
    
    moverobot_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='moverobot',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'moverobot',
                'bt_xml_file': example_dir + '/behavior_trees_xml/moverobot.xml'
            }
        ])
    
    setstatus_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='setstatus',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'setstatus',
                'bt_xml_file': example_dir + '/behavior_trees_xml/setstatus.xml'
            }
        ])
    
    execute_bottom_cmd = Node(
        package='jigless-planner',
        executable='execute_bottom_node',
        name='execute_bottom',
        namespace=top_ns,
        output='screen',
        parameters=[]
    )

    top_controller = Node(
        package='jigless-planner',
        executable='top_controller_node',
        name='top_controller',
        namespace=top_ns,
        output='screen',
        parameters=[]
    )
 # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_bottom_ns_cmd)
    ld.add_action(declare_top_ns_cmd)

    # Declare the launch options
    ld.add_action(plansys2_bottom)
    ld.add_action(plansys2_top)
    ld.add_action(bottom_controller)
    ld.add_action(top_controller)

    ld.add_action(transit_cmd)
    ld.add_action(weld_cmd)
    ld.add_action(validate_cmd)
    ld.add_action(command_cmd)
    ld.add_action(moverobot_cmd)
    ld.add_action(setstatus_cmd)
    ld.add_action(execute_bottom_cmd)
    return ld
