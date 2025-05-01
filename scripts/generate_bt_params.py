#!/usr/bin/env python3

import yaml
import sys

def generate_bt_params(num_commands, num_commandables, num_set_statuses):
    bt_params = {}
    port_counter = 1668  # Start port counter from the first port

    # Generate single instances of other nodes
    bt_params['bottom_planner/transit'] = {
        'ros__parameters': {
            'plugins': ['jigless_planner_transit_bt_node'],
            'enable_groot_monitoring': True,
            'publisher_port': port_counter,
            'server_port': port_counter + 1,
        }
    }
    port_counter += 2

    bt_params['bottom_planner/weld'] = {
        'ros__parameters': {
            'plugins': ['jigless_planner_weld_bt_node'],
            'enable_groot_monitoring': True,
            'publisher_port': port_counter,
            'server_port': port_counter + 1,
        }
    }
    port_counter += 2

    bt_params['bottom_planner/validate'] = {
        'ros__parameters': {
            'plugins': ['jigless_planner_validate_bt_node'],
            'enable_groot_monitoring': True,
            'publisher_port': port_counter,
            'server_port': port_counter + 1,
        }
    }
    port_counter += 2

    bt_params['top_planner/move_robot'] = {
        'ros__parameters': {
            'plugins': ['jigless_planner_moverobot_bt_node'],
            'enable_groot_monitoring': True,
            'publisher_port': port_counter,
            'server_port': port_counter + 1,
        }
    }
    port_counter += 2

    bt_params['top_planner/execute_1'] = {
        'ros__parameters': {
            'enable_groot_monitoring': True,
            'publisher_port': port_counter,
            'server_port': port_counter + 1,
        }
    }
    port_counter += 2

    # Generate command nodes
    for i in range(1, num_commands + 1):
        bt_params[f'top_planner/command_{i}'] = {
            'ros__parameters': {
                'plugins': ['jigless_planner_command_bt_node'],
                'enable_groot_monitoring': True,
                'publisher_port': port_counter,
                'server_port': port_counter + 1,
            }
        }
        port_counter += 2

    # Generate set_commandable nodes
    for i in range(1, num_commandables + 1):
        bt_params[f'top_planner/set_commandable_{i}'] = {
            'ros__parameters': {
                'plugins': ['jigless_planner_setcommandable_bt_node'],
                'enable_groot_monitoring': True,
                'publisher_port': port_counter,
                'server_port': port_counter + 1,
            }
        }
        port_counter += 2

    # Generate set_status nodes
    for i in range(1, num_set_statuses + 1):
        bt_params[f'top_planner/set_status_{i}'] = {
            'ros__parameters': {
                'plugins': ['jigless_planner_setstatus_bt_node'],
                'enable_groot_monitoring': True,
                'publisher_port': port_counter,
                'server_port': port_counter + 1,
            }
        }
        port_counter += 2

    return bt_params

if __name__ == "__main__":
    # if len(sys.argv) != 4:
    #     print("Usage: python generate_bt_params.py <num_commands> <num_commandables> <num_set_statuses>")
    #     sys.exit(1)

    # num_commands = int(sys.argv[1])
    # num_commandables = int(sys.argv[2])
    # num_set_statuses = int(sys.argv[3])

    num_commands = 10
    num_commandables = 10
    num_set_statuses = 10

    bt_params = generate_bt_params(num_commands, num_commandables, num_set_statuses)

    with open('config/bt_params.yaml', 'w') as yaml_file:
        yaml.dump(bt_params, yaml_file, default_flow_style=False)