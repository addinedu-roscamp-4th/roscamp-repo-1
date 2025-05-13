#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from libro_picker_srvs.msg import AngleCommand, PosCommand, MotorCommand, GripperCommand
from pymycobot import MyCobot280 as MyCobot
import time

class RobotControllerTopic(Node):
    def __init__(self):
        super().__init__('robot_controller_topic')
        
        # Initialize robot connection
        self.mc = None
        self.connected = False
        self.connect_robot()
        
        # Create publishers for robot state
        self.angles_pub = self.create_publisher(Float32MultiArray, 'robot_angles', 10)
        self.coords_pub = self.create_publisher(Float32MultiArray, 'robot_coords', 10)
        
        # Create subscribers for commands
        self.angles_sub = self.create_subscription(
            AngleCommand,
            'jetcobot_angles_command',
            self.angles_callback,
            10
        )
        self.coords_sub = self.create_subscription(
            PosCommand,
            'jetcobot_coords_command',
            self.coords_callback,
            10
        )
        self.motor_sub = self.create_subscription(
            MotorCommand,
            'jetcobot_motor_command',
            self.motor_callback,
            10
        )
        self.gripper_sub = self.create_subscription(
            GripperCommand,
            'jetcobot_gripper_command',
            self.gripper_callback,
            10
        )
        
        # Create timer for publishing robot state
        self.create_timer(0.1, self.publish_state)  # 10Hz
        
        self.get_logger().info('Robot controller topic node has been initialized')

    def connect_robot(self):
        """Connect to the robot"""
        try:
            self.mc = MyCobot('/dev/ttyJETCOBOT', 1000000)
            self.mc.thread_lock = True
            self.connected = True
            self.get_logger().info('Successfully connected to the robot')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to connect to the robot: {str(e)}')
            return False

    def publish_state(self):
        """Publish robot state (angles and coordinates)"""
        if not self.connected:
            return

        try:
            # Publish angles
            angles = self.mc.get_angles()
            if angles:
                msg = Float32MultiArray()
                msg.data = angles
                self.angles_pub.publish(msg)
                self.get_logger().debug(f'Published angles: {angles}')

            # Publish coordinates
            coords = self.mc.get_coords()
            if coords:
                msg = Float32MultiArray()
                msg.data = coords
                self.coords_pub.publish(msg)
                self.get_logger().debug(f'Published coordinates: {coords}')
        except Exception as e:
            self.get_logger().error(f'Error publishing robot state: {str(e)}')

    def angles_callback(self, msg):
        """Handle angle command"""
        if not self.connected:
            self.get_logger().error('Robot is not connected')
            return

        try:
            angles_list = list(msg.angles)
            self.mc.send_angles(angles_list, msg.speed)
            self.get_logger().info(f'Set angles to {angles_list} with speed {msg.speed}')
        except Exception as e:
            self.get_logger().error(f'Error controlling angles: {str(e)}')

    def coords_callback(self, msg):
        """Handle position command"""
        if not self.connected:
            self.get_logger().error('Robot is not connected')
            return

        try:
            coords_list = list(msg.coords)
            self.mc.send_coords(coords_list, msg.speed, msg.mode)
            self.get_logger().info(f'Set coordinates to {coords_list} with speed {msg.speed} and mode {msg.mode}')
        except Exception as e:
            self.get_logger().error(f'Error controlling position: {str(e)}')

    def motor_callback(self, msg):
        """Handle motor command"""
        if not self.connected:
            self.get_logger().error('Robot is not connected')
            return

        try:
            if msg.on_off:
                for i in range(6):
                    self.mc.focus_servo(i+1)
                self.get_logger().info('Motors have been activated')
            else:
                self.mc.release_all_servos()
                self.get_logger().info('Motors have been deactivated')
        except Exception as e:
            self.get_logger().error(f'Error controlling motors: {str(e)}')

    def gripper_callback(self, msg):
        """Handle gripper command"""
        if not self.connected:
            self.get_logger().error('Robot is not connected')
            return

        try:
            self.mc.set_gripper_value(msg.value, msg.speed)
            self.get_logger().info(f'Set gripper to value {msg.value} with speed {msg.speed}')
        except Exception as e:
            self.get_logger().error(f'Error controlling gripper: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    controller = RobotControllerTopic()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Node stopped cleanly')
    except Exception as e:
        controller.get_logger().error(f'Error in main loop: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 