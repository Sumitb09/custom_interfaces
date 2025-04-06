#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RequestPublisher(Node):
    def __init__(self):
        super().__init__('request_publisher')
        
        # Create publisher
        self._publisher = self.create_publisher(
            String,
            'action_requests',
            10
        )
        
        # Create timer to publish requests at different rates
        self.timer = self.create_timer(2.0, self.timer_callback)  # Start with 2-second interval
        self.count = 0
        
        self.get_logger().info('Request publisher has started')
        
    def timer_callback(self):
        """Publish requests with varying intervals."""
        msg = String()
        msg.data = f'Request {self.count}'
        
        self._publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        
        # Increase count
        self.count += 1
        
        # Change timer interval to simulate higher rate after 3 requests
        if self.count == 3:
            self.timer.cancel()
            self.timer = self.create_timer(1.0, self.timer_callback)  # Switch to 1-second interval
            self.get_logger().info('Increased publishing rate')

def main(args=None):
    rclpy.init(args=args)
    publisher = RequestPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
