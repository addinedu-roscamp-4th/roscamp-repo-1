#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from libro_picker_srvs.srv import MotorControl, AngleControl, PosControl, GripperControl
from pymycobot.mycobot import MyCobot
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Initialize robot connection
        self.mc = None
        self.connected = False
        self.connect_robot()
        
        # Create publishers for robot state
        self.angles_pub = self.create_publisher(Float32MultiArray, 'robot_angles', 10)
        self.coords_pub = self.create_publisher(Float32MultiArray, 'robot_coords', 10)
        
        # Create services
        self.motor_srv = self.create_service(MotorControl, 'motor_control', self.motor_callback)
        self.angle_srv = self.create_service(AngleControl, 'angle_control', self.angle_callback)
        self.pos_srv = self.create_service(PosControl, 'pos_control', self.pos_callback)
        self.gripper_srv = self.create_service(GripperControl, 'gripper_control', self.gripper_callback)
        
        # Create timer for publishing robot state
        self.create_timer(0.1, self.publish_state)  # 10Hz
        
        self.get_logger().info('Robot controller node has been initialized')

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

    def motor_callback(self, request, response):
        """Handle motor control service requests"""
        if not self.connected:
            response.success = False
            response.message = 'Robot is not connected'
            return response

        try:
            if request.on_off:
                for i in range(6):
                    self.mc.focus_servo(i+1)
                    time.sleep(0.1)
                response.message = 'Motors activated'
                self.get_logger().info('Motors have been activated')
            else:
                self.mc.release_all_servos()
                response.message = 'Motors deactivated'
                self.get_logger().info('Motors have been deactivated')
            response.success = True
        except Exception as e:
            response.success = False
            response.message = f'Error controlling motors: {str(e)}'
            self.get_logger().error(f'Motor control error: {str(e)}')
        return response

    def angle_callback(self, request, response):
        """Handle angle control service requests"""
        if not self.connected:
            response.success = False
            response.message = 'Robot is not connected'
            return response

        try:
            if len(request.angles) != 6:
                response.success = False
                response.message = 'Must provide exactly 6 angles'
                self.get_logger().error('Invalid number of angles provided')
                return response
            
            speed = request.speed if request.speed > 0 else 50  # Default speed if not specified
            self.mc.send_angles(request.angles, speed)
            response.success = True
            response.message = f'Successfully sent angle command with speed {speed}'
            self.get_logger().info(f'Set angles to {request.angles} with speed {speed}')
        except Exception as e:
            response.success = False
            response.message = f'Error controlling angles: {str(e)}'
            self.get_logger().error(f'Angle control error: {str(e)}')
        return response

    def pos_callback(self, request, response):
        """Handle position control service requests"""
        if not self.connected:
            response.success = False
            response.message = 'Robot is not connected'
            return response

        try:
            if len(request.coords) != 6:
                response.success = False
                response.message = 'Must provide exactly 6 coordinates'
                self.get_logger().error('Invalid number of coordinates provided')
                return response
            
            speed = request.speed if request.speed > 0 else 50  # Default speed if not specified
            mode = request.mode if request.mode >= 0 else 0  # Default mode if not specified
            
            self.mc.send_coords(request.coords, speed, mode)
            response.success = True
            response.message = f'Successfully sent coordinate command with speed {speed} and mode {mode}'
            self.get_logger().info(f'Set coordinates to {request.coords} with speed {speed} and mode {mode}')
        except Exception as e:
            response.success = False
            response.message = f'Error controlling position: {str(e)}'
            self.get_logger().error(f'Position control error: {str(e)}')
        return response

    def gripper_callback(self, request, response):
        """Handle gripper control service requests"""
        if not self.connected:
            response.success = False
            response.message = 'Robot is not connected'
            return response

        try:
            # Handle both integer and list inputs for gripper value
            grip_value = request.value
            if isinstance(grip_value, list):
                if len(grip_value) > 0:
                    grip_value = grip_value[0]
                else:
                    response.success = False
                    response.message = 'Empty gripper value list provided'
                    self.get_logger().error('Empty gripper value list')
                    return response

            speed = request.speed if request.speed > 0 else 50  # Default speed if not specified
            self.mc.set_gripper_value(grip_value, speed)
            response.success = True
            response.message = f'Successfully sent gripper command with value {grip_value} and speed {speed}'
            self.get_logger().info(f'Set gripper to value {grip_value} with speed {speed}')
        except Exception as e:
            response.success = False
            response.message = f'Error controlling gripper: {str(e)}'
            self.get_logger().error(f'Gripper control error: {str(e)}')
        return response

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    
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