#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from custom_interfaces.action import Wait

class WaitActionClient(Node):
    def __init__(self):
        super().__init__('wait_action_client')
        
        # Create callback group for concurrent callbacks
        callback_group = ReentrantCallbackGroup()
        
        # Create action client
        self._action_client = ActionClient(
            self, 
            Wait, 
            'wait',
            callback_group=callback_group
        )
        
        # Create subscriber to receive action requests
        self._subscription = self.create_subscription(
            String,
            'action_requests',
            self.request_callback,
            10,
            callback_group=callback_group
        )
        
        # Keep track of current goal handle
        self._goal_handle = None
        self._is_canceling = False
        
        self.get_logger().info('Wait action client has started')
        
    def request_callback(self, msg):
        """Handle incoming requests from topic."""
        self.get_logger().info(f'Received request: {msg.data}')
        
        # If there's an active goal, cancel it
        if self._goal_handle is not None and self._goal_handle.accepted:
            if not self._is_canceling:
                self.get_logger().info('Canceling previous goal')
                self._is_canceling = True
                # Cancel the active goal
                cancel_future = self._goal_handle.cancel_goal_async()
                # Wait for cancellation to complete before sending a new goal
                cancel_future.add_done_callback(
                    lambda _: self.send_goal(msg.data)
                )
        else:
            # No active goal, send a new one
            self.send_goal(msg.data)


    def send_goal(self, request_data):
        """Send a goal to the action server."""
        self._is_canceling = False
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Create goal
        goal_msg = Wait.Goal()
        goal_msg.request = request_data
        
        self.get_logger().info('Sending goal...')
        
        # Send goal and get future
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """Handle goal response from the action server."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
            
        self.get_logger().info('Goal accepted')
        self._goal_handle = goal_handle
        
        # Get result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        """Handle result from the action server."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')
        
    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.progress}%')

def main(args=None):
    rclpy.init(args=args)
    action_client = WaitActionClient()
    
    # Use MultiThreadedExecutor to allow concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(action_client)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
