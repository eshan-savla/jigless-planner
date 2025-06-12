#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor
from jigless_planner_interfaces.srv import InteractTop

class Test2ClientNode(Node):
    def __init__(self):
        super().__init__('test2_client_node')
        self.cli = self.create_client(InteractTop, '/top_planner/interact_top')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        count_descriptor = ParameterDescriptor(description='Number of joints to weld (int)')
        self.declare_parameter('joint_count', 10, count_descriptor)
        self.req = InteractTop.Request()

    def send_request(self, joints_count: int):
        self.req.joints.joints = ["joint" + str(i) for i in range(1, joints_count + 1)]
        self.get_logger().info(f'Sending request with joints: {self.req.joints.joints}')
        self.req.operation = InteractTop.Request.START
        return self.cli.call_async(self.req)
    
def main(args=None):
    rclpy.init(args=args)
    test2_client_node = Test2ClientNode()
    
    def shutdown_callback():
        test2_client_node.get_logger().info('Shutting down...')
        test2_client_node.destroy_node()
        rclpy.shutdown()

    # Send initial request
    test2_client_node.get_logger().info('Sending initial request...')
    joints_count = test2_client_node.get_parameter('joint_count').get_parameter_value().integer_value
    future = test2_client_node.send_request(joints_count)
    rclpy.spin_until_future_complete(test2_client_node, future)
    response = future.result()
    if response.success:
        test2_client_node.get_logger().info('Initial request successful')
    else:
        test2_client_node.get_logger().info(f'Initial request failed: {response.error_info}')
    shutdown_callback()

if __name__ == '__main__':
    main()