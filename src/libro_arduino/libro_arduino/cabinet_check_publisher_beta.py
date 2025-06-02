import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import serial
import time
import os
import json

class CabinetCheckPublisher(Node):
    def __init__(self):
        super().__init__('cabinet_check_publisher')
        
        # Get parameter values with defaults
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        
        # Set absolute path for cabinet status file
        self.cabinet_status_file = '/home/addinedu/libro_total/src/libro_arduino/libro_arduino/cabinet_status.json'
        self.get_logger().info(f'Cabinet status file path: {self.cabinet_status_file}')

        arduino_port = self.get_parameter('arduino_port').value
        baud_rate = self.get_parameter('baud_rate').value

        # Initialize cabinet status
        self.cabinet_status = self.load_cabinet_status()
        
        # Dictionary to store publishers and subscribers for each robot
        self.robot_publishers = {}
        self.robot_subscribers = {}
        
        # Subscribe to active robots topic
        self.active_robots_subscription = self.create_subscription(
            Float32MultiArray,
            'active_lilbro_ids',
            self.active_robots_callback,
            10
        )
        self.get_logger().info('Subscribed to active_lilbro_ids topic')

        try:
            self.ser = serial.Serial(arduino_port, baud_rate)
            time.sleep(3)  # Wait for Arduino to initialize
            self.get_logger().info(f'Connected to Arduino on port {arduino_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            raise

        timer_period = 0.05  # 50ms로 변경 (더 빠른 읽기)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def load_cabinet_status(self):
        try:
            with open(self.cabinet_status_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except FileNotFoundError:
            self.get_logger().info(f'Creating new cabinet status file at {self.cabinet_status_file}')
            # Default status if file doesn't exist
            default_status = [
                {"pickup_place": "픽업존 A", "P1": "empty", "P2": "empty"},
                {"pickup_place": "픽업존 B", "P1": "empty", "P2": "empty"}
            ]
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(self.cabinet_status_file), exist_ok=True)
            # Save default status
            with open(self.cabinet_status_file, 'w', encoding='utf-8') as f:
                json.dump(default_status, f, indent=4, ensure_ascii=False)
            return default_status

    def save_cabinet_status(self):
        try:
            with open(self.cabinet_status_file, 'w', encoding='utf-8') as f:
                json.dump(self.cabinet_status, f, indent=4, ensure_ascii=False)
            self.get_logger().debug('Cabinet status saved successfully')
        except Exception as e:
            self.get_logger().error(f'Error saving cabinet status: {str(e)}')

    def timer_callback(self):
        try:
            if self.ser.in_waiting:
                current_status = self.ser.readline().decode('utf-8', errors='ignore').strip()
                self.get_logger().debug(f'Received from Arduino: {current_status}')
                
                # 상태 변화를 감지하기 위한 플래그
                status_changed = False
                
                # Update cabinet status based on sensor readings
                if "AP1 : OFF" in current_status:
                    if self.cabinet_status[0]["P1"] != "empty":
                        self.cabinet_status[0]["P1"] = "empty"
                        status_changed = True
                else:
                    if self.cabinet_status[0]["P1"] != "filled":
                        self.cabinet_status[0]["P1"] = "filled"
                        status_changed = True

                if "AP2 : OFF" in current_status:
                    if self.cabinet_status[0]["P2"] != "empty":
                        self.cabinet_status[0]["P2"] = "empty"
                        status_changed = True
                else:
                    if self.cabinet_status[0]["P2"] != "filled":
                        self.cabinet_status[0]["P2"] = "filled"
                        status_changed = True

                if "BP1 : OFF" in current_status:
                    if self.cabinet_status[1]["P1"] != "empty":
                        self.cabinet_status[1]["P1"] = "empty"
                        status_changed = True
                else:
                    if self.cabinet_status[1]["P1"] != "filled":
                        self.cabinet_status[1]["P1"] = "filled"
                        status_changed = True

                if "BP2 : OFF" in current_status:
                    if self.cabinet_status[1]["P2"] != "empty":
                        self.cabinet_status[1]["P2"] = "empty"
                        status_changed = True
                else:
                    if self.cabinet_status[1]["P2"] != "filled":
                        self.cabinet_status[1]["P2"] = "filled"
                        status_changed = True
                
                # 상태가 변경된 경우에만 JSON 파일 갱신
                if status_changed:
                    self.save_cabinet_status()
                    self.get_logger().info('Cabinet status changed and saved')
            
        except serial.SerialException as e:
            self.get_logger().error(f'Error reading from Arduino: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Unexpected error in timer_callback: {str(e)}')

    def create_robot_subscriber(self, robot_id):
        if robot_id not in self.robot_subscribers:
            # 빈 캐비닛 확인을 위한 구독자
            empty_topic = f'{robot_id}/empty_cabinet'
            self.robot_subscribers[robot_id] = self.create_subscription(
                String, empty_topic, 
                lambda msg: self.check_empty_cabinet_callback(msg, robot_id), 10)
            
            # 캐비닛 더블 체크를 위한 구독자
            check_topic = f'{robot_id}/check_cabinet'
            self.robot_subscribers[f'{robot_id}_check'] = self.create_subscription(
                String, check_topic,
                lambda msg: self.double_check_cabinet_callback(msg, robot_id), 10)
            
            self.get_logger().info(f'Created subscribers for {robot_id}')

    def check_empty_cabinet_callback(self, msg, robot_id):
        try:
            # Parse the request message
            request = msg.data
            self.get_logger().info(f'Received request from {robot_id}: {request}')
            
            # Expected format: "Book Number : X, Pickup Place : Y"
            parts = request.split(',')
            book_number = int(parts[0].split(':')[1].strip())
            pickup_place = parts[1].split(':')[1].strip()
            
            # Read latest status from JSON file
            try:
                with open(self.cabinet_status_file, 'r', encoding='utf-8') as f:
                    current_status = json.load(f)
            except Exception as e:
                self.get_logger().error(f'Error reading cabinet status file: {str(e)}')
                current_status = self.cabinet_status  # Fallback to memory status
            
            # Find the corresponding pickup zone
            zone_index = 0 if pickup_place == 'A' else 1
            zone = current_status[zone_index]
            
            # Find empty cabinets
            empty_cabinets = []
            if zone["P1"] == "empty":
                empty_cabinets.append("P1")
            if zone["P2"] == "empty":
                empty_cabinets.append("P2")
            
            # Create response based on book number and empty cabinets
            response = String()
            if book_number == 1:
                # 한 권만 필요하면 첫 번째 빈 캐비닛 반환
                response.data = empty_cabinets[0] if empty_cabinets else "P1"
            elif book_number == 2:
                # 두 권이 필요하면 두 캐비닛이 모두 비어있을 때만 P1,P2 반환
                if len(empty_cabinets) == 2:
                    response.data = "P1,P2"
                else:
                    response.data = empty_cabinets[0] if empty_cabinets else "P1"
            else:
                response.data = empty_cabinets[0] if empty_cabinets else "P1"
            
            # Create publisher if it doesn't exist
            if robot_id not in self.robot_publishers:
                topic_name = f'{robot_id}/empty_cabinet_info'
                self.robot_publishers[robot_id] = self.create_publisher(String, topic_name, 10)
            
            # Publish response
            self.robot_publishers[robot_id].publish(response)
            self.get_logger().info(f'Response to {robot_id}: {response.data}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing request: {str(e)}')

    def double_check_cabinet_callback(self, msg, robot_id):
        try:
            # Parse the request message
            request = msg.data
            self.get_logger().info(f'Received check request from {robot_id}: {request}')
            
            # Expected format: "Pickup Place : X, Cabinet : Y"
            try:
                pickup_part = request.split('Pickup Place : ')[1].split(',')[0].strip()
                cabinet_part = request.split('Cabinet : ')[1].strip()
            except IndexError:
                self.get_logger().error(f'Invalid request format: {request}')
                return
                
            # Validate pickup place
            if pickup_part not in ['A', 'B']:
                self.get_logger().error(f'Invalid pickup place: {pickup_part}')
                return
                
            # Validate cabinet
            if cabinet_part not in ['P1', 'P2', 'P1,P2']:
                self.get_logger().error(f'Invalid cabinet: {cabinet_part}')
                return
            
            # Read latest status from JSON file
            try:
                with open(self.cabinet_status_file, 'r', encoding='utf-8') as f:
                    current_status = json.load(f)
            except Exception as e:
                self.get_logger().error(f'Error reading cabinet status file: {str(e)}')
                current_status = self.cabinet_status  # Fallback to memory status
            
            # Find the corresponding pickup zone
            zone_index = 0 if pickup_part == 'A' else 1
            zone = current_status[zone_index]
            
            # Create response
            response = String()
            if cabinet_part == 'P1,P2':
                # 둘 다 채워져 있어야 ON
                if zone["P1"] == "filled" and zone["P2"] == "filled":
                    response.data = "ON"
                else:
                    response.data = "OFF"
            else:
                # 단일 캐비닛 체크
                if zone[cabinet_part] == "filled":
                    response.data = "ON"
                else:
                    response.data = "OFF"
            
            # Create publisher if it doesn't exist
            if f'{robot_id}_check' not in self.robot_publishers:
                topic_name = f'{robot_id}/check_cabinet_info'
                self.robot_publishers[f'{robot_id}_check'] = self.create_publisher(String, topic_name, 10)
            
            # Publish response
            self.robot_publishers[f'{robot_id}_check'].publish(response)
            self.get_logger().info(f'Response to {robot_id} check: {response.data}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing check request: {str(e)}')

    def active_robots_callback(self, msg):
        """활성화된 로봇 목록이 변경될 때 호출되는 콜백"""
        current_robots = set(int(robot_id) for robot_id in msg.data)
        
        # 새로운 로봇에 대한 구독자 생성
        for robot_id in current_robots:
            robot_name = f'libro{robot_id}'
            if robot_name not in self.robot_subscribers:
                self.create_robot_subscriber(robot_name)
                self.get_logger().info(f'Created subscriber for new robot: {robot_name}')
        
        # 비활성화된 로봇의 구독자 제거
        for robot_name in list(self.robot_subscribers.keys()):
            if '_check' in robot_name:
                base_name = robot_name.replace('_check', '')
            else:
                base_name = robot_name
            robot_id = int(base_name.replace('libro', ''))
            if robot_id not in current_robots:
                self.robot_subscribers[robot_name].destroy()
                del self.robot_subscribers[robot_name]
                if robot_name in self.robot_publishers:
                    self.robot_publishers[robot_name].destroy()
                    del self.robot_publishers[robot_name]
                self.get_logger().info(f'Removed subscriber for inactive robot: {robot_name}')

def main(args=None):
    rclpy.init(args=args)
    try:
        cabinet_check_publisher = CabinetCheckPublisher()
        rclpy.spin(cabinet_check_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if 'cabinet_check_publisher' in locals():
            cabinet_check_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()