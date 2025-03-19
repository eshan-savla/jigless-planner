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
                'enable_groot_monitoring': True,
                'action_name': 'transit',
                'publisher_port': 1692,
                'server_port': 1693,
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
                'enable_groot_monitoring': True,
                'publisher_port': 1690,
                'server_port': 1691,
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
                'enable_groot_monitoring': True,
                'action_name': 'validate',
                'publisher_port': 1688,
                'server_port': 1689,
                'bt_xml_file': example_dir + '/behavior_trees_xml/validate.xml'
            }
        ])

    bottom_controller = Node(
        package='jigless-planner',
        executable='bottom_controller_node',
        name=bottom_name,
        namespace=bottom_ns,
        output='screen',
        parameters=[]
    )

    # Specify the top actions

    command_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='command_1',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'enable_groot_monitoring': True,
                'action_name': 'command',
                'publisher_port': 1668,
                'server_port': 1669,
                'bt_xml_file': example_dir + '/behavior_trees_xml/command.xml'
            }
        ])

    command_2_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='command_2',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'enable_groot_monitoring': True,
                'action_name': 'command',
                'publisher_port': 1670,
                'server_port': 1671,
                'bt_xml_file': example_dir + '/behavior_trees_xml/command.xml'
            }
        ])

    command_3_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='command_3',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'enable_groot_monitoring': True,
                'action_name': 'command',
                'publisher_port': 1672,
                'server_port': 1673,
                'bt_xml_file': example_dir + '/behavior_trees_xml/command.xml'
            }
        ])

    moverobot_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_robot',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'enable_groot_monitoring': True,
                'action_name': 'move_robot',
                'publisher_port': 1684,
                'server_port': 1685,
                'bt_xml_file': example_dir + '/behavior_trees_xml/moverobot.xml'
            }
        ])

    set_status_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='set_status_1',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'enable_groot_monitoring': True,
                'action_name': 'set_status',
                'publisher_port': 1674,
                'server_port': 1675,
                'bt_xml_file': example_dir + '/behavior_trees_xml/setstatus.xml'
            }
        ])

    set_status_2_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='set_status_2',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'enable_groot_monitoring': True,
                'action_name': 'set_status',
                'publisher_port': 1676,
                'server_port': 1677,
                'bt_xml_file': example_dir + '/behavior_trees_xml/setstatus.xml'
            }
        ])

    set_status_3_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='set_status_3',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'enable_groot_monitoring': True,
                'action_name': 'set_status',
                'publisher_port': 1678,
                'server_port': 1679,
                'bt_xml_file': example_dir + '/behavior_trees_xml/setstatus.xml'
            }
        ])

    set_status_4_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='set_status_4',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'enable_groot_monitoring': True,
                'action_name': 'set_status',
                'publisher_port': 1680,
                'server_port': 1681,
                'bt_xml_file': example_dir + '/behavior_trees_xml/setstatus.xml'
            }
        ])

    set_status_5_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='set_status_5',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/params.yaml',
            {
                'action_name': 'set_status',
                'enable_groot_monitoring': True,
                'publisher_port': 1682,
                'server_port': 1683,
                'bt_xml_file': example_dir + '/behavior_trees_xml/setstatus.xml'
            }
        ])
    execute_bottom_cmd = Node(
        package='jigless-planner',
        executable='execute_bottom_node',
        name='execute_1',
        namespace=top_ns,
        output='screen',
        parameters=[
            {
                'enable_groot_monitoring': True,
                'bottom_ns': bottom_ns,
                'publisher_port': 1686,
                'server_port': 1687,
            },
        ]
    )

    top_controller = Node(
        package='jigless-planner',
        executable='top_controller_node',
        name='top_controller',
        namespace=top_ns,
        output='screen',
        parameters=[
            {
                'bottom_ns': bottom_ns,
                'bottom_controller_name': bottom_name,
            }
        ]
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
    ld.add_action(command_1_cmd)
    ld.add_action(command_2_cmd)
    ld.add_action(command_3_cmd)
    ld.add_action(moverobot_cmd)
    ld.add_action(set_status_1_cmd)
    ld.add_action(set_status_2_cmd)
    ld.add_action(set_status_3_cmd)
    ld.add_action(set_status_4_cmd)
    ld.add_action(set_status_5_cmd)
    ld.add_action(execute_bottom_cmd)
    return ld
