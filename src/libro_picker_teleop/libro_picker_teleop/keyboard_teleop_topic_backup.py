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
    
    def send_angle_command(self, joint_idx, direction):
        cmd = AngleCommand()
        self.current_angles[joint_idx] += direction * self.angle_step
        cmd.angles = self.current_angles
        cmd.speed = self.speed
        self.angle_pub.publish(cmd)
        self.get_logger().info(f'Sent angle command: {cmd.angles}')
    
    def send_pos_command(self, coord_idx, direction):
        cmd = PosCommand()
        self.current_coords[coord_idx] += direction * self.pos_step
        cmd.coords = self.current_coords
        cmd.speed = self.speed
        cmd.mode = 0  # Cartesian mode
        self.pos_pub.publish(cmd)
        self.get_logger().info(f'Sent position command: {cmd.coords}')
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x1b':  # ESC
                    break
                elif key == ' ':  # SPACE
                    # Stop robot (implement if needed)
                    pass
                
                # Joint control
                elif key == 'q': self.send_angle_command(0, -1)
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
                elif key == 'i': self.send_pos_command(0, 1)   # X+
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