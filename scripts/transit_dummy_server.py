#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from transit_interfaces.action import GenerateTransit

class TransitDummyServer(Node):
    def __init__(self):
        super().__init__('transit_dummy_server')
        self._action_server = ActionServer(
            self,
            GenerateTransit,
            'generate_transit',
            self.execute_callback
        )
        self.get_logger().info('Transit Dummy Server is running.')
        self.start_time = None
        self.duration = 5
        self.tick = 0

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Transit ...')
        self.start_time = self.get_clock().now()
        feedback_msg = GenerateTransit.Feedback()
        feedback_msg.completion = 0.0
        stop = False
        while not stop:
            elapsed_time = self.get_clock().now() - self.start_time
            if elapsed_time.nanoseconds >= self.duration * 1e9:
                stop = True
                break
            if self.tick == 0:
                self.get_logger().info(f"Transiting from {goal_handle.request.from_joint} to {goal_handle.request.to_joint}")
                self.get_logger().info(f"Moving workpieces: {goal_handle.request.workpiece1}")
                if goal_handle.request.workpiece2:
                    self.get_logger().info(f"Moving workpieces: {goal_handle.request.workpiece2}")
            feedback_msg.completion = elapsed_time.nanoseconds / (self.duration * 1e9)
            goal_handle.publish_feedback(feedback_msg)
            self.tick += 1
            self.get_logger().info(f"Transit tick: {self.tick}")
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))
        goal_handle.succeed()
        self.tick = 0
        result = GenerateTransit.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    transit_dummy_server = TransitDummyServer()
    rclpy.spin(transit_dummy_server)

if __name__ == '__main__':
    main()


