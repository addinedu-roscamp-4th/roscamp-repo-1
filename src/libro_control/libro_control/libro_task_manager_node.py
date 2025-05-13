import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool
from libro_control_msgs.msg import BookPickUpMission, NavigateMission, LibroRobotLog, PinkyMissionPlace
import json
import time
from datetime import datetime
import os
from enum import Enum

class RobotState(Enum):
    CHARGING = 'charging'      # a: 충전중
    WAITING = 'waiting'        # b: 대기중
    PICKUP_MISSION = 'pickup'  # c: 픽업미션중
    NAVIGATION_MISSION = 'nav' # d: 길안내미션중

class LibroTaskManagerNode(Node):
    def __init__(self):
        super().__init__('libro_task_manager_node')
        
        # Publishers
        self.mission_pickup_pub = self.create_publisher(BookPickUpMission, 'book_pick_up_mission', 10)
        self.mission_nav_pub = self.create_publisher(NavigateMission, 'pinky_mission_place', 10)
        # Add state publisher
        self.robot_states_pub = self.create_publisher(String, 'libro_robot_states', 10)
        
        # Subscribers
        self.robot_log_sub = self.create_subscription(
            LibroRobotLog, 'libro_robot_log', self.robot_log_callback, 10)
        
        # Robot 1 subscribers
        self.robot1_battery_sub = self.create_subscription(
            Float32, 'libro_robot_1_battery_present', self.robot1_battery_callback, 10)
        self.robot1_mission_complete_sub = self.create_subscription(
            Bool, 'libro_robot_1_mission_complete', self.robot1_mission_complete_callback, 10)
        
        # Robot 2 subscribers
        self.robot2_battery_sub = self.create_subscription(
            Float32, 'libro_robot_2_battery_present', self.robot2_battery_callback, 10)
        self.robot2_mission_complete_sub = self.create_subscription(
            Bool, 'libro_robot_2_mission_complete', self.robot2_mission_complete_callback, 10)
        
        # Robot states
        self.robot_states = {
            'libro_robot_1': RobotState.WAITING,
            'libro_robot_2': RobotState.WAITING
        }
        
        # Battery thresholds
        self.LOW_BATTERY_THRESHOLD = 20.0  # 20%
        self.CHARGED_BATTERY_THRESHOLD = 80.0  # 80%
        
        # Current tasks for each robot
        self.current_tasks = {
            'libro_robot_1': None,
            'libro_robot_2': None
        }
        
        # Service requests
        self.service_requests = []
        
        # File paths :: 절대경로 여야함
        self.service_request_log_path = '/home/addinedu/libro_total/src/libro_control/libro_control/libro_service_request_log.json'
        self.robot_log_path = '/home/addinedu/libro_total/src/libro_control/libro_control/libro_robot_log.json'
        
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
            'libro_robot_1': self.robot_states['libro_robot_1'].value,
            'libro_robot_2': self.robot_states['libro_robot_2'].value
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
        self.handle_battery_update('libro_robot_1', msg.data)

    def robot2_battery_callback(self, msg):
        """Handle robot 2 battery updates"""
        self.handle_battery_update('libro_robot_2', msg.data)

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
            self.handle_mission_completion('libro_robot_1')

    def robot2_mission_complete_callback(self, msg):
        """Handle robot 2 mission completion"""
        if msg.data:
            self.handle_mission_completion('libro_robot_2')

    def handle_mission_completion(self, robot_id):
        """Handle mission completion for robots"""
        current_state = self.robot_states[robot_id]
        if current_state in [RobotState.PICKUP_MISSION, RobotState.NAVIGATION_MISSION]:
            self.update_robot_state(robot_id, RobotState.WAITING)
            self.current_tasks[robot_id] = None

    def load_service_requests(self):
        """Load service requests from JSON file"""
        try:
            if os.path.exists(self.service_request_log_path):
                with open(self.service_request_log_path, 'r', encoding='utf-8') as f:
                    self.service_requests = json.load(f)
                self.get_logger().info(f'Loaded {len(self.service_requests)} service requests')
        except Exception as e:
            self.get_logger().error(f'Error loading service requests: {str(e)}')

    def create_mission_place(self, place_data, robot_id):
        """Create a PinkyMissionPlace message from place data"""
        place = PinkyMissionPlace()
        place.robot_name_id = robot_id  # Use the assigned robot's name
        place.x = place_data['x']
        place.y = place_data['y']
        place.theta = place_data['theta']
        return place

    def process_tasks(self):
        """Process tasks in the queue"""
        # Check each robot's state and assign tasks if possible
        for robot_id in ['libro_robot_1', 'libro_robot_2']:
            if (self.robot_states[robot_id] == RobotState.WAITING and 
                not self.current_tasks[robot_id] and 
                self.service_requests):
                
                # Find next request
                for i, request in enumerate(self.service_requests):
                    if request['type'] == 'book_request':
                        # Create and publish book pickup mission message
                        msg = BookPickUpMission()
                        msg.robot_name_id = robot_id
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
                        msg = NavigateMission()
                        msg.robot_name_id = robot_id
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
        """Handle robot log messages from libro_robot_log topic"""
        try:
            # Create log entry from received message
            log_entry = {
                'timestamp': msg.time,
                'robot_name_id': msg.robot_name_id,
                'order_number': msg.order_number,
                'log': msg.robot_log,
                'success': msg.success
            }
            
            # Load existing logs or create new list if file doesn't exist or is empty
            existing_logs = []
            if os.path.exists(self.robot_log_path):
                try:
                    with open(self.robot_log_path, 'r', encoding='utf-8') as f:
                        content = f.read().strip()
                        if content:  # Only try to parse if file is not empty
                            existing_logs = json.loads(content)
                except json.JSONDecodeError:
                    self.get_logger().warn(f'Invalid JSON in log file, creating new log file')
                    existing_logs = []
            
            # Append new log
            existing_logs.append(log_entry)
            
            # Save updated logs to file
            with open(self.robot_log_path, 'w', encoding='utf-8') as f:
                json.dump(existing_logs, f, indent=2, ensure_ascii=False)
            
            self.get_logger().info(f'Saved new log from {msg.robot_name_id}: {msg.robot_log}')
            
        except Exception as e:
            self.get_logger().error(f'Error handling robot log: {str(e)}')

    def save_service_requests(self):
        """Save service requests to JSON file"""
        try:
            with open(self.service_request_log_path, 'w', encoding='utf-8') as f:
                json.dump(self.service_requests, f, indent=2, ensure_ascii=False)
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