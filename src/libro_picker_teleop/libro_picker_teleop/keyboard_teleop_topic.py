#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from libro_picker_srvs.msg import AngleCommand, PosCommand
import sys
import termios
import tty
import select
import time

class KeyboardTeleopTopic(Node):
    def __init__(self):
        super().__init__('keyboard_teleop_topic')
        
        # Create publishers
        self.angle_pub = self.create_publisher(AngleCommand, 'jetcobot_angles_command', 10)
        self.pos_pub = self.create_publisher(PosCommand, 'jetcobot_coords_command', 10)
        
        # Initialize control parameters
        self.angle_step = 5.0  # degrees
        self.pos_step = 10.0   # mm
        self.speed = 50        # 1-100
        
        # Initialize current state
        self.current_angles = None
        self.current_coords = None
        
        # Initialize control mode
        self.current_mode = None  # 'angle' or 'position'
        
        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
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
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def update_robot_state(self):
        """Update current robot state by subscribing to topics temporarily"""
        self.get_logger().info('Updating robot state...')
        
        # Reset current state
        self.current_angles = None
        self.current_coords = None
        
        # Create temporary subscribers
        angles_sub = self.create_subscription(Float32MultiArray, 'robot_angles', 
            lambda msg: setattr(self, 'current_angles', msg.data), 10)
        coords_sub = self.create_subscription(Float32MultiArray, 'robot_coords', 
            lambda msg: setattr(self, 'current_coords', msg.data), 10)
        
        # Wait for messages
        start_time = time.time()
        while (self.current_angles is None or self.current_coords is None) and \
              (time.time() - start_time) < 5.0:  # 5 second timeout
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_angles is None:
                self.get_logger().info('Waiting for current angles...')
            if self.current_coords is None:
                self.get_logger().info('Waiting for current coordinates...')
        
        # Clean up temporary subscribers
        self.destroy_subscription(angles_sub)
        self.destroy_subscription(coords_sub)
        
        if self.current_angles is not None and self.current_coords is not None:
            self.get_logger().info(f'Current angles: {self.current_angles}')
            self.get_logger().info(f'Current coordinates: {self.current_coords}')
            return True
        else:
            self.get_logger().error('Failed to update robot state')
            return False
    
    def check_mode_switch(self, new_mode):
        """Check if control mode is switching and update state if needed"""
        if self.current_mode != new_mode:
            self.current_mode = new_mode
            return self.update_robot_state()
        return True
    
    def send_angle_command(self, joint_idx, direction):
        if not self.check_mode_switch('angle'):
            return
        cmd = AngleCommand()
        self.current_angles[joint_idx] += direction * self.angle_step
        cmd.angles = self.current_angles
        cmd.speed = self.speed
        self.angle_pub.publish(cmd)
        self.get_logger().info(f'Sent angle command: {cmd.angles}')
    
    def send_pos_command(self, coord_idx, direction):
        if not self.check_mode_switch('position'):
            return
        cmd = PosCommand()
        self.current_coords[coord_idx] += direction * self.pos_step
        cmd.coords = self.current_coords
        cmd.speed = self.speed
        cmd.mode = 0  # Cartesian mode
        self.pos_pub.publish(cmd)
        self.get_logger().info(f'Sent position command: {cmd.coords}')
    
    def run(self):
        try:
            # Get initial state
            if not self.update_robot_state():
                self.get_logger().error('Failed to get initial robot state. Exiting...')
                return
                
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x1b':  # ESC
                    break
                elif key == ' ':  # SPACE
                    # Update state when stopping
                    self.update_robot_state()
                
                # Joint control
                elif key in ['q', 'w', 'a', 's', 'z', 'x', 'e', 'r', 'd', 'f', 'c', 'v']:
                    if key == 'q': self.send_angle_command(0, -1)
                    elif key == 'w': self.send_angle_command(0, 1)
                    elif key == 'a': self.send_angle_command(1, -1)
                    elif key == 's': self.send_angle_command(1, 1)
                    elif key == 'z': self.send_angle_command(2, -1)
                    elif key == 'x': self.send_angle_command(2, 1)
                    elif key == 'e': self.send_angle_command(3, -1)
                    elif key == 'r': self.send_angle_command(3, 1)
                    elif key == 'd': self.send_angle_command(4, -1)
                    elif key == 'f': self.send_angle_command(4, 1)
                    elif key == 'c': self.send_angle_command(5, -1)
                    elif key == 'v': self.send_angle_command(5, 1)
                
                # Position control
                elif key in ['i', 'k', 'j', 'l', 'o', 'p', 'u', 'm', 'h', 'n', 'b']:
                    if key == 'i': self.send_pos_command(0, 1)   # X+
                    elif key == 'k': self.send_pos_command(0, -1)  # X-
                    elif key == 'j': self.send_pos_command(1, -1)  # Y-
                    elif key == 'l': self.send_pos_command(1, 1)   # Y+
                    elif key == 'o': self.send_pos_command(2, 1)   # Z+
                    elif key == 'p': self.send_pos_command(2, -1)  # Z-
                    elif key == 'u': self.send_pos_command(3, 1)   # RX+
                    elif key == 'm': self.send_pos_command(3, -1)  # RX-
                    elif key == 'h': self.send_pos_command(4, -1)  # RY-
                    elif key == 'n': self.send_pos_command(4, 1)   # RY+
                    elif key == 'b': self.send_pos_command(5, -1)  # RZ-
                    elif key == 'm': self.send_pos_command(5, 1)   # RZ+
                
                time.sleep(0.1)  # Small delay to prevent overwhelming the robot
        
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.get_logger().info('Keyboard teleop node stopped.')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopTopic()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 