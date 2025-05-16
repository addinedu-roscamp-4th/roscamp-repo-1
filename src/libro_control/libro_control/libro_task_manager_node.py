import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from libro_control_msgs.msg import BookPickUp, Navigate, LibroRobotLog, PinkyRequest
import json
import time
from datetime import datetime
import os
from enum import Enum
import requests

class RobotState(Enum):
    CHARGING = 'charging'      # a: 충전중
    WAITING = 'waiting'        # b: 대기중
    PICKUP_MISSION = 'pickup'  # c: 픽업미션중
    NAVIGATION_MISSION = 'nav' # d: 길안내미션중

class LibroTaskManagerNode(Node):
    def __init__(self):
        super().__init__('libro_task_manager_node')
        
        # API endpoints
        self.service_log_url = 'http://192.168.0.138:8000/service-log'
        self.robot_log_url = 'http://192.168.0.138:8000/robot-log'
        
        # Publishers
        self.mission_pickup_pub = self.create_publisher(BookPickUp, 'request_book', 10)
        self.mission_nav_pub = self.create_publisher(Navigate, 'request_nav', 10)
        # Add state publisher
        self.robot_states_pub = self.create_publisher(String, 'libro_states', 10)
        
        # Subscribers
        self.robot_log_sub = self.create_subscription(
            LibroRobotLog, 'robot_log', self.robot_log_callback, 10)
        
        # Robot 1 subscribers
        self.robot1_battery_sub = self.create_subscription(
            Float32, 'libro_1/battery_present', self.robot1_battery_callback, 10)
        self.robot1_mission_complete_sub = self.create_subscription(
            Bool, 'libro_1/completed', self.robot1_mission_complete_callback, 10)
        
        # Robot 2 subscribers
        self.robot2_battery_sub = self.create_subscription(
            Float32, 'libro_2/battery_present', self.robot2_battery_callback, 10)
        self.robot2_mission_complete_sub = self.create_subscription(
            Bool, 'libro_2/completed', self.robot2_mission_complete_callback, 10)
        
        # Robot states
        self.robot_states = {
            'libro_1': RobotState.WAITING,
            'libro_2': RobotState.WAITING
        }
        
        # Battery thresholds
        self.LOW_BATTERY_THRESHOLD = 20.0  # 20%
        self.CHARGED_BATTERY_THRESHOLD = 80.0  # 80%
        
        # Current tasks for each robot
        self.current_tasks = {
            'libro_1': None,
            'libro_2': None
        }
        
        # Service requests
        self.service_requests = []
        
        # Load service requests
        self.load_service_requests()
        
        # Timer for task processing
        self.task_timer = self.create_timer(1.0, self.process_tasks)
        # Timer for state publishing
        self.state_timer = self.create_timer(0.1, self.publish_robot_states)  # 10Hz로 상태 발행
        
        self.get_logger().info('Libro Task Manager Node initialized')

    def publish_robot_states(self):
        """Publish current states of both robots"""
        # Create states dictionary
        states = {
            'libro_1': self.robot_states['libro_1'].value,
            'libro_2': self.robot_states['libro_2'].value
        }
        
        # Convert to JSON string
        msg = String()
        msg.data = json.dumps(states)
        
        # Publish combined states
        self.robot_states_pub.publish(msg)

    def update_robot_state(self, robot_id, new_state):
        """Update robot state with validation"""
        old_state = self.robot_states[robot_id]
        
        # Validate state transition
        if new_state == RobotState.CHARGING:
            if old_state != RobotState.WAITING:
                self.get_logger().warn(f'Invalid state transition for {robot_id}: {old_state} -> {new_state}')
                return False
        elif new_state == RobotState.WAITING:
            if old_state not in [RobotState.CHARGING, RobotState.PICKUP_MISSION, RobotState.NAVIGATION_MISSION]:
                self.get_logger().warn(f'Invalid state transition for {robot_id}: {old_state} -> {new_state}')
                return False
        elif new_state in [RobotState.PICKUP_MISSION, RobotState.NAVIGATION_MISSION]:
            if old_state != RobotState.WAITING:
                self.get_logger().warn(f'Invalid state transition for {robot_id}: {old_state} -> {new_state}')
                return False
        
        self.robot_states[robot_id] = new_state
        self.get_logger().info(f'{robot_id} state changed: {old_state} -> {new_state}')
        return True

    def robot1_battery_callback(self, msg):
        """Handle robot 1 battery updates"""
        self.handle_battery_update('libro_1', msg.data)

    def robot2_battery_callback(self, msg):
        """Handle robot 2 battery updates"""
        self.handle_battery_update('libro_2', msg.data)

    def handle_battery_update(self, robot_id, battery_level):
        """Handle battery level updates for robots"""
        current_state = self.robot_states[robot_id]
        
        if current_state == RobotState.WAITING and battery_level < self.LOW_BATTERY_THRESHOLD:
            self.update_robot_state(robot_id, RobotState.CHARGING)
        elif current_state == RobotState.CHARGING and battery_level >= self.CHARGED_BATTERY_THRESHOLD:
            self.update_robot_state(robot_id, RobotState.WAITING)

    def robot1_mission_complete_callback(self, msg):
        """Handle robot 1 mission completion"""
        if msg.data:
            self.handle_mission_completion('libro_1')

    def robot2_mission_complete_callback(self, msg):
        """Handle robot 2 mission completion"""
        if msg.data:
            self.handle_mission_completion('libro_2')

    def handle_mission_completion(self, robot_id):
        """Handle mission completion for robots"""
        current_state = self.robot_states[robot_id]
        if current_state in [RobotState.PICKUP_MISSION, RobotState.NAVIGATION_MISSION]:
            self.update_robot_state(robot_id, RobotState.WAITING)
            self.current_tasks[robot_id] = None

    def load_service_requests(self):
        """Load service requests from remote server"""
        try:
            response = requests.get(self.service_log_url)
            if response.status_code == 200:
                self.service_requests = response.json()
                self.get_logger().info(f'Loaded {len(self.service_requests)} service requests')
            else:
                self.get_logger().error(f'Failed to load service requests. Status code: {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'Error loading service requests: {str(e)}')

    def create_mission_place(self, place_data, robot_id):
        """Create a PinkyRequest message from place data"""
        place = PinkyRequest()
        place.x = place_data['x']
        place.y = place_data['y']
        place.theta = place_data['theta']
        return place

    def process_tasks(self):
        """Process tasks in the queue"""
        # Check each robot's state and assign tasks if possible
        for robot_id in ['libro_1', 'libro_2']:
            if (self.robot_states[robot_id] == RobotState.WAITING and 
                not self.current_tasks[robot_id] and 
                self.service_requests):
                
                # Find next request
                for i, request in enumerate(self.service_requests):
                    if request['type'] == 'book_request':
                        # Create and publish book pickup mission message
                        msg = BookPickUp()
                        msg.robot_id = robot_id
                        msg.book_name = request['book_name']
                        msg.book_place = self.create_mission_place(request['book_place'][0], robot_id)
                        msg.pickup_place = self.create_mission_place(request['pickup_place'][0], robot_id)
                        msg.order_number = request['order_number']
                        
                        self.mission_pickup_pub.publish(msg)
                        self.current_tasks[robot_id] = request
                        self.update_robot_state(robot_id, RobotState.PICKUP_MISSION)
                        
                        # Remove the assigned request from the list
                        self.service_requests.pop(i)
                        self.save_service_requests()
                        
                        self.get_logger().info(f'Assigned book pickup task to {robot_id}: {request["book_name"]}')
                        break
                        
                    elif request['type'] == 'navigate_request':
                        # Create and publish navigation mission message
                        msg = Navigate()
                        msg.robot_id = robot_id
                        msg.user_email = request['user_email']
                        msg.order_number = request['order_number']
                        msg.start_place = self.create_mission_place(request['start_place'][0], robot_id)
                        msg.end_place = self.create_mission_place(request['end_place'][0], robot_id)
                        
                        self.mission_nav_pub.publish(msg)
                        self.current_tasks[robot_id] = request
                        self.update_robot_state(robot_id, RobotState.NAVIGATION_MISSION)
                        
                        # Remove the assigned request from the list
                        self.service_requests.pop(i)
                        self.save_service_requests()
                        
                        self.get_logger().info(f'Assigned navigation task to {robot_id}: {request["start_place"][0]["name"]} -> {request["end_place"][0]["name"]}')
                        break

    def robot_log_callback(self, msg):
        """Handle robot log messages and save to remote server"""
        try:
            # Create log entry from received message
            log_entry = {
                'timestamp': msg.timestamp,
                'robot_id': msg.robot_id,
                'order_number': msg.order_number,
                'log': msg.robot_log,
                'success': msg.success
            }
            
            # Get existing logs from server
            try:
                response = requests.get(self.robot_log_url)
                if response.status_code == 200:
                    existing_logs = response.json()
                else:
                    self.get_logger().warn(f'Failed to get existing logs. Status code: {response.status_code}')
                    existing_logs = []
            except Exception as e:
                self.get_logger().warn(f'Error getting existing logs: {str(e)}')
                existing_logs = []
            
            # Append new log
            existing_logs.append(log_entry)
            
            # Use POST method to update the log list
            response = requests.post(self.robot_log_url, json=existing_logs)
            
            if response.status_code != 200:
                self.get_logger().error(f'Failed to save robot log. Status code: {response.status_code}')
            else:
                self.get_logger().info(f'Saved new log from {msg.robot_id}: {msg.robot_log}')
            
        except Exception as e:
            self.get_logger().error(f'Error handling robot log: {str(e)}')

    def save_service_requests(self):
        """Save service requests to remote server"""
        try:
            # Use POST method to update the service requests
            response = requests.post(self.service_log_url, json=self.service_requests)
            if response.status_code != 200:
                self.get_logger().error(f'Failed to save service requests. Status code: {response.status_code}')
        except Exception as e:
            self.get_logger().error(f'Error saving service requests: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = LibroTaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 