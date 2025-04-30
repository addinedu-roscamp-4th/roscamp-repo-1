#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from libro_mobile_srvs.srv import SetGoalPose
from libro_picker_srvs.srv import AngleControl
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

class ServiceTestNode(Node):
    def __init__(self):
        super().__init__('service_test_node')
        
        # Create callback group
        self.callback_group = ReentrantCallbackGroup()
        
        # Create service clients
        self.mobile_client = self.create_client(
            SetGoalPose, 
            'set_goal_pose',
            callback_group=self.callback_group
        )
        self.picker_client = self.create_client(
            AngleControl, 
            'angle_control',
            callback_group=self.callback_group
        )
        
        # State variables
        self.mobile_future = None
        self.picker_future = None
        self.test_completed = False
        
        # Wait for services to become available
        while not self.mobile_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Mobile service not available, waiting...')
        while not self.picker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Picker service not available, waiting...')
            
        self.get_logger().info('All services are available!')
        
        # Start testing after a short delay
        self.create_timer(2.0, self.start_mobile_test)
        
    def start_mobile_test(self):
        """Start the mobile service test"""
        self.get_logger().info('Starting mobile service test...')
        
        request = SetGoalPose.Request()
        request.x = 0.8
        request.y = -0.2
        request.w = -0.707
        request.z = 0.707
        
        self.mobile_future = self.mobile_client.call_async(request)
        self.mobile_future.add_done_callback(self.mobile_test_done)
    
    def mobile_test_done(self, future):
        """Handle mobile service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Mobile service succeeded: {response.message}')
            else:
                self.get_logger().error(f'Mobile service failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Mobile service call failed: {str(e)}')
        
        # Schedule picker test after 2 seconds
        self.create_timer(2.0, self.start_picker_test, callback_group=self.callback_group)
    
    def start_picker_test(self):
        """Start the picker service test"""
        self.get_logger().info('Starting picker service test...')
        
        request = AngleControl.Request()
        request.angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        request.speed = 50
        
        self.picker_future = self.picker_client.call_async(request)
        self.picker_future.add_done_callback(self.picker_test_done)
    
    def picker_test_done(self, future):
        """Handle picker service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Picker service succeeded: {response.message}')
            else:
                self.get_logger().error(f'Picker service failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Picker service call failed: {str(e)}')
        
        # All tests completed, schedule shutdown
        self.get_logger().info('All tests completed, shutting down...')
        self.create_timer(1.0, self.shutdown_node, callback_group=self.callback_group)
    
    def shutdown_node(self):
        """Shutdown the node"""
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ServiceTestNode()
    rclpy.spin(node)
    
if __name__ == '__main__':
    main() 