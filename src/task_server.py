#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from custom_interfaces.action import Wait
import time

class WaitActionServer(Node):
    def __init__(self):
        super().__init__('wait_action_server')
        
        # Use ReentrantCallbackGroup to allow concurrent callbacks
        callback_group = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            Wait,
            'wait',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group
        )
        self.get_logger().info('Wait action server has started')
        
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(f'Received goal request: {goal_request.request}')
        return GoalResponse.ACCEPT
        
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
        
    def execute_callback(self, goal_handle):
        """Execute the goal with a 5-second wait period."""
        self.get_logger().info(f'Executing goal: {goal_handle.request.request}')
        
        # Initialize feedback and result messages
        feedback_msg = Wait.Feedback()
        result = Wait.Result()
        
        # Simulate a 5-second task with feedback
        for i in range(5):
            # Check if goal is cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.result = 'Canceled before completion'
                return result
                
            # Set progress feedback
            feedback_msg.progress = (i + 1) * 20.0  # 20% per second
            self.get_logger().info(f'Progress: {feedback_msg.progress}%')
            goal_handle.publish_feedback(feedback_msg)
            
            # Wait for 1 second
            time.sleep(1)
        
        # Set goal success and return result
        goal_handle.succeed()
        result.result = 'Task completed successfully after 5 seconds'
        self.get_logger().info('Goal succeeded')
        return result

def main(args=None):
    rclpy.init(args=args)
    wait_action_server = WaitActionServer()
    
    # Use MultiThreadedExecutor to allow concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(wait_action_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        wait_action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
