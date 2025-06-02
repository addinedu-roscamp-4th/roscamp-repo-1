import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Bool, Int32, Float32MultiArray
from libro_control_msgs.msg import BookPickUp, Navigate, LibroRobotLog, PinkyRequest
import json
import time
from datetime import datetime
import os
from enum import Enum
import requests

class RobotState(Enum):
    CHARGING = 'charging'      # a: 충전중
    IDLE = 'idle'              # b: 대기중
    PICKUP_MISSION = 'pickup'  # c: 픽업미션중
    NAVIGATION_MISSION = 'nav' # d: 길안내미션중

class LibroTaskManagerNode(Node):
    def __init__(self):
        super().__init__('libro_task_manager_node')
        
        # API endpoints
        self.book_request_url = "http://192.168.0.138:8000/book-request-list/"
        self.navigation_url = "http://192.168.0.138:8000/navigation-guidances-list"
        self.robot_log_url = "http://192.168.0.138:8000/robot-log"
        
        # Request lists
        self.book_requests = []
        self.nav_requests = []
        
        # Publishers
        self.mission_pickup_pub = self.create_publisher(BookPickUp, 'request_book', 10)
        self.mission_nav_pub = self.create_publisher(Navigate, 'request_nav', 10)
        # Add state publisher
        self.robot_states_pub = self.create_publisher(String, 'libro_robot_states', 10)
        
        # Subscribers
        self.robot_log_sub = self.create_subscription(
            LibroRobotLog, 'robot_log', self.robot_log_callback, 10)
        
        # 활성화된 로봇 ID를 구독
        self.active_robots_subscription = self.create_subscription(
            Float32MultiArray,
            'active_lilbro_ids',
            self.active_robots_callback,
            10
        )
        
        # Robot states - 동적으로 관리
        self.robot_states = {}
        
        # Battery thresholds
        self.LOW_BATTERY_THRESHOLD = 20.0  # 20%
        self.CHARGED_BATTERY_THRESHOLD = 80.0  # 80%
        
        # Current tasks for each robot - 동적으로 관리
        self.current_tasks = {}
        
        # Timer for task processing
        self.task_timer = self.create_timer(1.0, self.process_tasks)
        # Timer for state publishing
        self.state_timer = self.create_timer(0.1, self.publish_robot_states)  # 10Hz로 상태 발행
        # Timer for updating request lists
        self.update_requests_timer = self.create_timer(5.0, self.update_request_lists)  # 5초마다 요청 목록 업데이트
        
        self.get_logger().info('Libro Task Manager Node initialized')

    def active_robots_callback(self, msg):
        """활성화된 로봇 목록이 변경될 때 호출되는 콜백"""
        current_robots = set(int(robot_id) for robot_id in msg.data)
        
        # 새로운 로봇에 대한 구독자 생성
        for robot_id in current_robots:
            robot_name = f'libro{robot_id}'
            if robot_name not in self.robot_states:
                self.robot_states[robot_name] = RobotState.IDLE
                self.current_tasks[robot_name] = None
                self.get_logger().info(f'New robot detected: {robot_name}')
        
        # 비활성화된 로봇 제거
        for robot_name in list(self.robot_states.keys()):
            robot_id = int(robot_name.replace('libro', ''))
            if robot_id not in current_robots:
                del self.robot_states[robot_name]
                if robot_name in self.current_tasks:
                    del self.current_tasks[robot_name]
                self.get_logger().info(f'Robot removed: {robot_name}')

    def publish_robot_states(self):
        """Publish current states of all active robots"""
        # Create states dictionary with only active robots
        states = {robot_name: state.value for robot_name, state in self.robot_states.items()}
        
        # Convert to JSON string
        msg = String()
        msg.data = json.dumps(states)
        
        # Publish combined states
        self.robot_states_pub.publish(msg)

    def update_robot_state(self, robot_id, new_state):
        """Update robot state with validation"""
        if robot_id not in self.robot_states:
            self.get_logger().warn(f'Attempted to update state for non-existent robot: {robot_id}')
            return False
            
        old_state = self.robot_states[robot_id]
        
        # Validate state transition
        if new_state == RobotState.CHARGING:
            if old_state != RobotState.IDLE:
                self.get_logger().warn(f'Invalid state transition for {robot_id}: {old_state} -> {new_state}')
                return False
        elif new_state == RobotState.IDLE:
            if old_state not in [RobotState.CHARGING, RobotState.PICKUP_MISSION, RobotState.NAVIGATION_MISSION]:
                self.get_logger().warn(f'Invalid state transition for {robot_id}: {old_state} -> {new_state}')
                return False
        elif new_state in [RobotState.PICKUP_MISSION, RobotState.NAVIGATION_MISSION]:
            if old_state != RobotState.IDLE:
                self.get_logger().warn(f'Invalid state transition for {robot_id}: {old_state} -> {new_state}')
                return False
        
        self.robot_states[robot_id] = new_state
        self.get_logger().info(f'{robot_id} state changed: {old_state} -> {new_state}')
        return True

    def handle_battery_update(self, robot_id, battery_level):
        """Handle battery level updates for robots"""
        if robot_id not in self.robot_states:
            return
            
        current_state = self.robot_states[robot_id]
        
        if current_state == RobotState.IDLE and battery_level < self.LOW_BATTERY_THRESHOLD:
            self.update_robot_state(robot_id, RobotState.CHARGING)
        elif current_state == RobotState.CHARGING and battery_level >= self.CHARGED_BATTERY_THRESHOLD:
            self.update_robot_state(robot_id, RobotState.IDLE)

    def handle_mission_completion(self, robot_id):
        """Handle mission completion for robots"""
        if robot_id not in self.robot_states:
            return
            
        current_state = self.robot_states[robot_id]
        if current_state in [RobotState.PICKUP_MISSION, RobotState.NAVIGATION_MISSION]:
            self.update_robot_state(robot_id, RobotState.IDLE)
            self.current_tasks[robot_id] = None

    def update_request_lists(self):
        """Update the request lists from API"""
        try:
            # Get book requests
            book_response = requests.get(self.book_request_url)
            if book_response.status_code == 200:
                self.book_requests = book_response.json()
            
            # Get navigation requests
            nav_response = requests.get(self.navigation_url)
            if nav_response.status_code == 200:
                self.nav_requests = nav_response.json()
            
            self.get_logger().info(f'Updated request lists - Book requests: {len(self.book_requests)}, Navigation requests: {len(self.nav_requests)}, Total: {len(self.book_requests) + len(self.nav_requests)}')
            
        except Exception as e:
            self.get_logger().error(f'Error updating request lists: {str(e)}')

    def process_tasks(self):
        """Process tasks in the queue"""
        # Check each robot's state and assign tasks if possible
        for robot_id in list(self.robot_states.keys()):
            if (self.robot_states[robot_id] == RobotState.IDLE and 
                not self.current_tasks[robot_id]):
                
                try:
                    # Find the oldest request
                    oldest_request = None
                    oldest_timestamp = None
                    request_type = None
                    
                    # Check book requests
                    if self.book_requests:
                        request = self.book_requests[0]
                        timestamp = datetime.fromisoformat(request['timestamp'].replace('Z', '+00:00'))
                        oldest_request = request
                        oldest_timestamp = timestamp
                        request_type = 'book'
                    
                    # Check navigation requests
                    if self.nav_requests:
                        request = self.nav_requests[0]
                        timestamp = datetime.fromisoformat(request['timestamp'].replace('Z', '+00:00'))
                        if oldest_timestamp is None or timestamp < oldest_timestamp:
                            oldest_request = request
                            oldest_timestamp = timestamp
                            request_type = 'nav'
                    
                    # Process the oldest request
                    if oldest_request:
                        if request_type == 'book':
                            # Create and publish book pickup mission message
                            msg = BookPickUp()
                            msg.robot_id = robot_id
                            msg.order_number = oldest_request['order_number']
                            
                            # First book
                            msg.book_name_1 = oldest_request['book_1']['book_name']
                            msg.book_place_1 = self.create_mission_place({
                                'x': oldest_request['book_1']['x'],
                                'y': oldest_request['book_1']['y'],
                                'theta': oldest_request['book_1']['theta']
                            }, robot_id)
                            
                            # Second book (if exists)
                            if 'book_2' in oldest_request and oldest_request['book_2']:
                                msg.book_name_2 = oldest_request['book_2']['book_name']
                                msg.book_place_2 = self.create_mission_place({
                                    'x': oldest_request['book_2']['x'],
                                    'y': oldest_request['book_2']['y'],
                                    'theta': oldest_request['book_2']['theta']
                                }, robot_id)
                            
                            # Pickup place
                            msg.pickup_place_name = oldest_request['pickup_place'][0]['name']
                            msg.pickup_place = self.create_mission_place(oldest_request['pickup_place'][0], robot_id)
                            
                            self.mission_pickup_pub.publish(msg)
                            self.current_tasks[robot_id] = oldest_request
                            self.update_robot_state(robot_id, RobotState.PICKUP_MISSION)
                            
                            # Delete the assigned request
                            delete_url = f"{self.book_request_url}/{oldest_request['order_number']}/"
                            self.get_logger().info(f'Attempting to delete book request at: {delete_url}')
                            delete_response = requests.delete(delete_url)
                            if delete_response.status_code != 200:
                                self.get_logger().error(f"Failed to delete book request {oldest_request['order_number']}. Status code: {delete_response.status_code}, Response: {delete_response.text}")
                            else:
                                self.get_logger().info(f"Successfully deleted book request {oldest_request['order_number']}")
                                # Remove from local list
                                self.book_requests = [req for req in self.book_requests if req['order_number'] != oldest_request['order_number']]
                            
                            self.get_logger().info(f'Assigned book pickup task to {robot_id}: {oldest_request["book_1"]["book_name"]}' + 
                                                 (f' and {oldest_request["book_2"]["book_name"]}' if 'book_2' in oldest_request and oldest_request['book_2'] else '') +
                                                 f' Pickup Place : {oldest_request["pickup_place"][0]["name"]}')
                            
                        else:  # navigation request
                            # Create and publish navigation mission message
                            msg = Navigate()
                            msg.robot_id = robot_id
                            msg.user_id = oldest_request['user_id']
                            msg.order_number = oldest_request['order_number']
                            msg.start_place = self.create_mission_place(oldest_request['start_place'][0], robot_id)
                            msg.end_place = self.create_mission_place(oldest_request['end_place'][0], robot_id)
                            
                            self.mission_nav_pub.publish(msg)
                            self.current_tasks[robot_id] = oldest_request
                            self.update_robot_state(robot_id, RobotState.NAVIGATION_MISSION)
                            
                            # Delete the assigned request
                            delete_url = f"{self.navigation_url}/{oldest_request['order_number']}/"
                            self.get_logger().info(f'Attempting to delete navigation request at: {delete_url}')
                            delete_response = requests.delete(delete_url)
                            if delete_response.status_code != 200:
                                self.get_logger().error(f"Failed to delete navigation request {oldest_request['order_number']}. Status code: {delete_response.status_code}, Response: {delete_response.text}")
                            else:
                                self.get_logger().info(f"Successfully deleted navigation request {oldest_request['order_number']}")
                                # Remove from local list
                                self.nav_requests = [req for req in self.nav_requests if req['order_number'] != oldest_request['order_number']]
                            
                            self.get_logger().info(f'Assigned navigation task to {robot_id}: {oldest_request["start_place"][0]["name"]} -> {oldest_request["end_place"][0]["name"]}')
                    
                    # Log current request counts
                    self.get_logger().info(f'Current request counts - Book requests: {len(self.book_requests)}, Navigation requests: {len(self.nav_requests)}, Total: {len(self.book_requests) + len(self.nav_requests)}')
                
                except Exception as e:
                    self.get_logger().error(f'Error processing requests: {str(e)}')

    def robot_log_callback(self, msg):
        """Handle robot log messages from libro_robot_log topic"""
        try:
            # Extract robot number from robot_id (e.g., "libro1" -> 1)
            robot_number = int(msg.robot_id.replace('libro', ''))
            
            # Create log entry from received message
            log_entry = {
                'timestamp': msg.timestamp,
                'robot_id': robot_number,  # Send only the number
                'order_number': msg.order_number,
                'log': msg.robot_log,
                'success': msg.success
            }
            
            # Send log to API
            response = requests.post(self.robot_log_url, json=log_entry)
            if response.status_code != 200:
                self.get_logger().error(f'Failed to send robot log: {response.text}')
            else:
                self.get_logger().info(f'Sent new log from {msg.robot_id}: {msg.robot_log}')
            
        except Exception as e:
            self.get_logger().error(f'Error handling robot log: {str(e)}')

    def create_mission_place(self, place_data, robot_id):
        """Create a PinkyRequest message from place data"""
        place = PinkyRequest()
        place.x = place_data['x']
        place.y = place_data['y']
        place.theta = place_data['theta']
        return place

def main(args=None):
    rclpy.init(args=args)
    node = LibroTaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 