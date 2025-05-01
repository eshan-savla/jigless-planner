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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('jigless-planner')
    bottom_ns = LaunchConfiguration('bottom_ns')
    top_ns = LaunchConfiguration('top_ns')
    problem_file_count = LaunchConfiguration('file_number')
    node_count = LaunchConfiguration('node_count')
    bottom_name = "bottom_controller"
    top_problem_file = example_dir + '/pddl/test1_complexity/top_welding_problem_'
    bottom_problem_file = example_dir + '/pddl/test1_complexity/weldcell_problem_no_workpiece_'
    
    # Declare namespaces
    declare_bottom_ns_cmd = DeclareLaunchArgument(
        'bottom_ns',
        default_value='bottom_planner',
        description='Namespace')

    declare_top_ns_cmd = DeclareLaunchArgument(
        'top_ns',
        default_value='top_planner',
        description='Namespace')
    
    declare_file_count = DeclareLaunchArgument(
        'file_number',
        default_value='10',
        description='Problem file number'
    )
    declare_node_count = DeclareLaunchArgument(
        'node_count',
        default_value='3',
        description='Number of nodes to create'
    )

    log_problem_file = LogInfo(
        msg=[
            'Bottom file: ', bottom_problem_file, problem_file_count, '.pddl', '\n'
            'Top file: ', top_problem_file, problem_file_count, '.pddl'
        ]
    )
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
          'namespace': bottom_ns,
          'params_file': example_dir + '/config/bringup_params.yaml',
          }.items())

    plansys2_top = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file':
                    example_dir + '/pddl/top_domain.pddl',
          'namespace': top_ns,
          'params_file': example_dir + '/config/bringup_params.yaml',
          }.items())

    # Specify the bottom actions
    transit_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='transit',
        namespace=bottom_ns,
        output='screen',
        parameters=[
            example_dir + '/config/bt_params.yaml',
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
            example_dir + '/config/bt_params.yaml',
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
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'validate',
                'bt_xml_file': example_dir + '/behavior_trees_xml/validate.xml'
            }
        ])

    bottom_controller = Node(
        package='jigless-planner',
        executable='bottom_controller_node',
        name=bottom_name,
        namespace=bottom_ns,
        output='screen',
        parameters=[
            {
                'bottom_problem_file_path': [
                    bottom_problem_file, problem_file_count, TextSubstitution(text='.pddl')
                    ],
            }   
        ]
    )

    # Specify the top actions

    def create_dynamic_nodes(context): 
        dynamic_nodes = []
        count = int(LaunchConfiguration('node_count').perform(context))
        for i in range(1, count + 1):  # Dynamically create nodes
            dynamic_nodes.append(Node(
                package='plansys2_bt_actions',
                executable='bt_action_node',
                name=f'command_{i}',
                namespace=top_ns,
                output='screen',
                parameters=[
                    example_dir + '/config/bt_params.yaml',
                    {
                        'action_name': 'command',
                        'bt_xml_file': example_dir + '/behavior_trees_xml/command.xml'
                    }
                ]
            ))

            dynamic_nodes.append(Node(
                package='plansys2_bt_actions',
                executable='bt_action_node',
                name=f'set_commandable_{i}',
                namespace=top_ns,
                output='screen',
                parameters=[
                    example_dir + '/config/bt_params.yaml',
                    {
                        'action_name': 'set_commandable',
                        'bt_xml_file': example_dir + '/behavior_trees_xml/setcommandable.xml'
                    }
                ]
            ))

            dynamic_nodes.append(Node(
                package='plansys2_bt_actions',
                executable='bt_action_node',
                name=f'set_status_{i}',
                namespace=top_ns,
                output='screen',
                parameters=[
                    example_dir + '/config/bt_params.yaml',
                    {
                        'action_name': 'set_status',
                        'bt_xml_file': example_dir + '/behavior_trees_xml/setstatus.xml'
                    }
                ]
            ))
        return dynamic_nodes
        
    moverobot_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='move_robot',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'move_robot',
                'bt_xml_file': example_dir + '/behavior_trees_xml/moverobot.xml'
            }
        ])

    execute_bottom_cmd = Node(
        package='jigless-planner',
        executable='execute_bottom_node',
        name='execute_1',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/bt_params.yaml',
            {
                'bottom_ns': bottom_ns,
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
                'top_problem_file_path': [top_problem_file, problem_file_count, TextSubstitution(text='.pddl')],
            }
        ]
    )
 # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(log_problem_file)
    ld.add_action(declare_bottom_ns_cmd)
    ld.add_action(declare_top_ns_cmd)
    ld.add_action(declare_file_count)
    ld.add_action(declare_node_count)

    # Declare the launch options
    ld.add_action(plansys2_bottom)
    ld.add_action(plansys2_top)
    ld.add_action(bottom_controller)
    ld.add_action(top_controller)

    ld.add_action(transit_cmd)
    ld.add_action(weld_cmd)
    ld.add_action(validate_cmd)
    ld.add_action(moverobot_cmd)
    ld.add_action(execute_bottom_cmd)
    ld.add_action(OpaqueFunction(function=create_dynamic_nodes))
    return ld
