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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('jigless-planner')
    bottom_ns = LaunchConfiguration('bottom_ns')
    top_ns = LaunchConfiguration('top_ns')
    problem_file_count = LaunchConfiguration('file_number')
    bottom_name = "bottom_controller"
    top_problem_file = example_dir + '/pddl/test1/top_welding_problem_'
    bottom_problem_file = example_dir + '/pddl/test1/weldcell_problem_no_workpiece_'
    
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
    
    transit_dummy = Node(
        package='jigless-planner',
        executable='transit_dummy_node',
        name='transit_dummy_server',
        namespace=bottom_ns,
        output='screen',
        parameters=[]
    )

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

    weld_dummy = Node(
        package='jigless-planner',
        executable='weld_dummy_node',
        name='weld_dummy_server',
        namespace=bottom_ns,
        output='screen',
        parameters=[]
    )

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

    validate_dummy = Node(
        package='jigless-planner',
        executable='validate_dummy_node',
        name='validate_dummy_server',
        namespace=bottom_ns,
        output='screen',
        parameters=[]
    )

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

    command_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='command_1',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'command',
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
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'command',
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
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'command',
                'bt_xml_file': example_dir + '/behavior_trees_xml/command.xml'
            }
        ])

    set_commandable_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='set_commandable_1',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'set_commandable',
                'bt_xml_file': example_dir + '/behavior_trees_xml/setcommandable.xml'
            }
        ])
    
    set_commandable_2_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='set_commandable_2',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'set_commandable',
                'bt_xml_file': example_dir + '/behavior_trees_xml/setcommandable.xml'
            }
        ])
    
    set_commandable_3_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='set_commandable_3',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'set_commandable',
                'bt_xml_file': example_dir + '/behavior_trees_xml/setcommandable.xml'
            }
        ])
    
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

    set_status_1_cmd = Node(
        package='plansys2_bt_actions',
        executable='bt_action_node',
        name='set_status_1',
        namespace=top_ns,
        output='screen',
        parameters=[
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'set_status',
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
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'set_status',
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
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'set_status',
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
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'set_status',
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
            example_dir + '/config/bt_params.yaml',
            {
                'action_name': 'set_status',
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

    # Declare the launch options
    ld.add_action(plansys2_bottom)
    ld.add_action(plansys2_top)
    ld.add_action(bottom_controller)
    ld.add_action(top_controller)

    ld.add_action(transit_cmd)
    ld.add_action(transit_dummy)
    ld.add_action(weld_cmd)
    ld.add_action(weld_dummy)
    ld.add_action(validate_cmd)
    ld.add_action(validate_dummy)
    ld.add_action(command_1_cmd)
    ld.add_action(command_2_cmd)
    ld.add_action(command_3_cmd)
    ld.add_action(set_commandable_1_cmd)
    ld.add_action(set_commandable_2_cmd)
    ld.add_action(set_commandable_3_cmd)
    ld.add_action(moverobot_cmd)
    ld.add_action(set_status_1_cmd)
    ld.add_action(set_status_2_cmd)
    ld.add_action(set_status_3_cmd)
    ld.add_action(set_status_4_cmd)
    ld.add_action(set_status_5_cmd)
    ld.add_action(execute_bottom_cmd)
    return ld
