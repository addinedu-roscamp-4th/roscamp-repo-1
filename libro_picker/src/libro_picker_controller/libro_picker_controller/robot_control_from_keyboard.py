#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from libro_picker_srvs.srv import AngleControl, PosControl

class RobotControlFromKeyboard(Node):
    def __init__(self):
        super().__init__('robot_control_from_keyboard')
        
        # Create service clients
        self.angle_client = self.create_client(AngleControl, 'angle_control')
        self.pos_client = self.create_client(PosControl, 'pos_control')
        
        # Create subscribers for keyboard commands
        self.angle_cmd_sub = self.create_subscription(
            Float32MultiArray,
            'keyboard_angle_cmd',
            self.angle_cmd_callback,
            10
        )
        self.pos_cmd_sub = self.create_subscription(
            Float32MultiArray,
            'keyboard_pos_cmd',
            self.pos_cmd_callback,
            10
        )
        
        # Wait for services to be available
        while not self.angle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('angle_control service not available, waiting again...')
        while not self.pos_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pos_control service not available, waiting again...')
        
        self.get_logger().info('Robot control from keyboard node initialized')
    
    def angle_cmd_callback(self, msg):
        """Handle angle command from keyboard"""
        request = AngleControl.Request()
        request.angles = msg.data
        request.speed = 50  # Default speed
        
        future = self.angle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().debug(f'Successfully sent angle command: {msg.data}')
            else:
                self.get_logger().error(f'Failed to send angle command: {future.result().message}')
    
    def pos_cmd_callback(self, msg):
        """Handle position command from keyboard"""
        request = PosControl.Request()
        request.coords = msg.data
        request.speed = 50  # Default speed
        request.mode = 0    # Cartesian mode
        
        future = self.pos_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().debug(f'Successfully sent position command: {msg.data}')
            else:
                self.get_logger().error(f'Failed to send position command: {future.result().message}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlFromKeyboard()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 