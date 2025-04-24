#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from libro_picker_srvs.srv import AngleControl, PosControl
import sys
import termios
import tty
import select
import time

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Create service clients
        self.angle_client = self.create_client(AngleControl, 'angle_control')
        self.pos_client = self.create_client(PosControl, 'pos_control')
        
        # Initialize control parameters
        self.angle_step = 5.0  # degrees
        self.pos_step = 10.0   # mm
        self.speed = 50        # 1-100
        
        # Wait for services to be available
        while not self.angle_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('angle_control service not available, waiting again...')
        while not self.pos_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('pos_control service not available, waiting again...')
        
        # Get initial robot state from topics
        self.get_logger().info('Getting initial robot state...')
        
        # Create temporary subscribers
        angles_sub = self.create_subscription(Float32MultiArray, 'robot_angles', 
            lambda msg: setattr(self, 'current_angles', msg.data), 10)
        coords_sub = self.create_subscription(Float32MultiArray, 'robot_coords', 
            lambda msg: setattr(self, 'current_coords', msg.data), 10)
        
        # Wait for initial messages
        self.current_angles = None
        self.current_coords = None
        
        while self.current_angles is None or self.current_coords is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_angles is None:
                self.get_logger().info('Waiting for initial angles...')
            if self.current_coords is None:
                self.get_logger().info('Waiting for initial coordinates...')
        
        # Clean up temporary subscribers
        self.destroy_subscription(angles_sub)
        self.destroy_subscription(coords_sub)
        
        self.get_logger().info(f'Initial angles: {self.current_angles}')
        self.get_logger().info(f'Initial coordinates: {self.current_coords}')
        
        self.get_logger().info('Keyboard teleop node initialized')
        self.print_instructions()
    
    def print_instructions(self):
        print("\n=== Keyboard Teleop Instructions ===")
        print("Joint Control:")
        print("  q/w: Joint 1 +/-")
        print("  a/s: Joint 2 +/-")
        print("  z/x: Joint 3 +/-")
        print("  e/r: Joint 4 +/-")
        print("  d/f: Joint 5 +/-")
        print("  c/v: Joint 6 +/-")
        print("\nPosition Control:")
        print("  i/k: X +/-")
        print("  j/l: Y +/-")
        print("  o/p: Z +/-")
        print("  u/m: RX +/-")
        print("  h/n: RY +/-")
        print("  b/m: RZ +/-")
        print("\nOther Controls:")
        print("  SPACE: Stop robot")
        print("  ESC: Exit")
        print("===============================\n")
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))
        return key
    
    def send_angle_request(self, joint_idx, direction):
        request = AngleControl.Request()
        self.current_angles[joint_idx] += direction * self.angle_step
        request.angles = self.current_angles
        request.speed = self.speed
        future = self.angle_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def send_pos_request(self, coord_idx, direction):
        request = PosControl.Request()
        self.current_coords[coord_idx] += direction * self.pos_step
        request.coords = self.current_coords
        request.speed = self.speed
        request.mode = 0  # Cartesian mode
        future = self.pos_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
    
    def run(self):
        try:
            while True:
                key = self.get_key()
                
                if key == '\x1b':  # ESC
                    break
                elif key == ' ':  # SPACE
                    # Stop robot (implement if needed)
                    pass
                
                # Joint control
                elif key == 'q': self.send_angle_request(0, -1)
                elif key == 'w': self.send_angle_request(0, 1)
                elif key == 'a': self.send_angle_request(1, -1)
                elif key == 's': self.send_angle_request(1, 1)
                elif key == 'z': self.send_angle_request(2, -1)
                elif key == 'x': self.send_angle_request(2, 1)
                elif key == 'e': self.send_angle_request(3, -1)
                elif key == 'r': self.send_angle_request(3, 1)
                elif key == 'd': self.send_angle_request(4, -1)
                elif key == 'f': self.send_angle_request(4, 1)
                elif key == 'c': self.send_angle_request(5, -1)
                elif key == 'v': self.send_angle_request(5, 1)
                
                # Position control
                elif key == 'i': self.send_pos_request(0, 1)   # X+
                elif key == 'k': self.send_pos_request(0, -1)  # X-
                elif key == 'j': self.send_pos_request(1, -1)  # Y-
                elif key == 'l': self.send_pos_request(1, 1)   # Y+
                elif key == 'o': self.send_pos_request(2, 1)   # Z+
                elif key == 'p': self.send_pos_request(2, -1)  # Z-
                elif key == 'u': self.send_pos_request(3, 1)   # RX+
                elif key == 'm': self.send_pos_request(3, -1)  # RX-
                elif key == 'h': self.send_pos_request(4, -1)  # RY-
                elif key == 'n': self.send_pos_request(4, 1)   # RY+
                elif key == 'b': self.send_pos_request(5, -1)  # RZ-
                elif key == 'm': self.send_pos_request(5, 1)   # RZ+
                
                time.sleep(0.1)  # Small delay to prevent overwhelming the robot
        
        except KeyboardInterrupt:
            pass
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, termios.tcgetattr(sys.stdin))

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
