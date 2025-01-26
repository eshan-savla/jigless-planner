#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from plansys2_msgs.srv import AddProblem
import sys

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddProblem, '/problem_expert/add_problem')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddProblem.Request()
        self.declare_parameter('problem_file_path')

    def send_request(self):
        problem_file_path = self.get_parameter('problem_file_path').get_parameter_value().string_value
        with open(problem_file_path, 'r') as file:
            self.req.problem = file.read()
        return self.cli.call_async(self.req)


def main():
    rclpy.init()
    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request()
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    if response.success:
        minimal_client.get_logger().info('Problem solved')
    else:
        minimal_client.get_logger().info(f'Problem ran into error {response.error_info}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()