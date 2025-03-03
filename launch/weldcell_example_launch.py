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
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_distributed.py')),
        launch_arguments={
          'model_file': 
                    example_dir + '/pddl/transit_domain.pddl:' + 
                    example_dir + '/pddl/weld_domain.pddl',
          'namespace': namespace
          }.items())

    # Specify the actions
    transit_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='transit',
        namespace=namespace,
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
        namespace=namespace,
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
        namespace=namespace,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'validate',
                'bt_xml_file': example_dir + '/behavior_trees_xml/validate.xml'
            }
        ])

    # transit_cmd = Node(
    #     package='jigless-planner',
    #     executable='transit_action_node',
    #     name='transit',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[]
    # )    

    # weld_cmd = Node(
    #     package='jigless-planner',
    #     executable='weld_action_node',
    #     name='weld',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[]
    # )
    # validate_cmd = Node(
    #     package='jigless-planner',
    #     executable='validate_action_node',
    #     name='validate',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[]
    # )
    
 # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    ld.add_action(plansys2_cmd)

    ld.add_action(transit_cmd)
    ld.add_action(weld_cmd)
    ld.add_action(validate_cmd)
    return ld
