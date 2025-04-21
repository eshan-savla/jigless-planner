#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from jigless_planner_interfaces.srv import InteractTop

class Test2ClientNode(Node):
    def __init__(self):
        super().__init__('test2_client_node')
        self.cli = self.create_client(InteractTop, '/top_planner/interact_top')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        from rcl_interfaces.msg import ParameterDescriptor
        freq_descriptor = ParameterDescriptor(description='Frequency at which plan changes will be sent')
        self.declare_parameter('frequency', 0.05, freq_descriptor)
        duration_descriptor = ParameterDescriptor(description='Duration of the test in seconds')
        self.declare_parameter('duration', 60, duration_descriptor)
        self.req = InteractTop.Request()

    def send_request(self):
        return self.cli.call_async(self.req)
    
    def send_initial_request(self):
        self.req.joints.joints = ["joint" + str(i) for i in range(1, 11)]
        self.req.operation = InteractTop.Request.START
        return self.send_request()
    
    def send_test_request(self, operation: int):
        self.req.joints.joints = ["joint6"]
        self.req.operation = operation
        return self.send_request()
    
    def send_final_request(self):
        self.req.joints.joints = ["joint" + str(i) for i in range(1, 11)]
        self.req.operation = InteractTop.Request.STOP
        return self.send_request()
    
def main(args=None):
    rclpy.init(args=args)
    test2_client_node = Test2ClientNode()
    
    # Send initial request
    test2_client_node.get_logger().info('Sending initial request...')
    future = test2_client_node.send_initial_request()
    rclpy.spin_until_future_complete(test2_client_node, future)
    response = future.result()
    if response.success:
        test2_client_node.get_logger().info('Initial request successful')
    else:
        test2_client_node.get_logger().info(f'Initial request failed: {response.error_info}') 
        return
    test2_client_node.get_logger().info('Initial request completed, waiting for 10 seconds...')
    test2_client_node.get_clock().sleep_for(rclpy.duration.Duration(seconds=10.0))

    # Start the test
    freq = test2_client_node.get_parameter('frequency').get_parameter_value().double_value
    start_time = test2_client_node.get_clock().now()
    duration = test2_client_node.get_parameter('duration').get_parameter_value().integer_value
    test2_client_node.get_logger().info('Starting test...')
    operation = InteractTop.Request.ADD
    delta = 1
    while True:
        elapsed_time = test2_client_node.get_clock().now() - start_time
        if elapsed_time.nanoseconds >= duration * 1e9:
            break
        test2_client_node.get_logger().info(f'Test running for {elapsed_time.nanoseconds / 1e9:.2f} seconds')
        test2_client_node.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))
        future = test2_client_node.send_test_request(operation)
        rclpy.spin_until_future_complete(test2_client_node, future)
        response = future.result()
        if response.success:
            test2_client_node.get_logger().info(f'request {operation} successful')
        else:
            test2_client_node.get_logger().info(f'request {operation} failed')
            return
        operation += delta
        delta *= -1
        test2_client_node.get_clock().sleep_for(rclpy.duration.Duration(seconds= 1.0 / freq))
    
    test2_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()