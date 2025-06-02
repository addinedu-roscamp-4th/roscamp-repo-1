import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool
from rcl_interfaces.msg import Log
import re
import time

class ActiveLilbroChecker(Node):
    def __init__(self):
        super().__init__('active_lilbro_checker')
        
        # 활성화된 로봇 ID를 저장할 집합
        self.active_robots = set()
        # 이전에 발행한 로봇 ID 리스트
        self.last_published_ids = []
        # 각 로봇의 배터리 상태를 저장하는 딕셔너리
        self.robot_battery_states = {}
        # 각 로봇의 마지막 메시지 시간을 저장하는 딕셔너리
        self.last_message_times = {}
        # 메시지 타임아웃 시간 (초)
        self.message_timeout = 5.0
        
        # 활성화된 로봇 ID를 발행하는 퍼블리셔
        self.active_robots_publisher = self.create_publisher(
            Float32MultiArray, 
            'active_lilbro_ids', 
            10
        )
        
        # ROS 로그를 모니터링하여 새로운 로봇 토픽을 감지
        self.create_subscription(
            Log,
            '/rosout',
            self.rosout_callback,
            10
        )
        
        # 주기적으로 토픽 리스트를 확인하는 타이머
        self.timer = self.create_timer(1.0, self.check_topics_callback)
        
        self.get_logger().info('Active Lilbro Checker Node started')

    def rosout_callback(self, msg):
        """ROS 로그를 모니터링하여 새로운 로봇 토픽을 감지"""
        if 'battery_present' in msg.msg:
            match = re.search(r'/(libro\d+)/battery_present', msg.msg)
            if match:
                robot_id = match.group(1)
                robot_num = int(robot_id.replace('libro', ''))
                if robot_num not in self.active_robots:
                    # 새로운 로봇이 감지되면 배터리 토픽 구독
                    self.subscribe_to_battery_topic(robot_id)
                    self.active_robots.add(robot_num)
                    self.publish_active_robots()
                    self.get_logger().info(f'New robot detected: {robot_id}')

    def subscribe_to_battery_topic(self, robot_id):
        """특정 로봇의 배터리 토픽을 구독"""
        topic_name = f'/{robot_id}/battery_present'
        self.create_subscription(
            Bool,
            topic_name,
            lambda msg, rid=robot_id: self.battery_callback(msg, rid),
            10
        )
        self.get_logger().info(f'Subscribed to battery topic: {topic_name}')

    def battery_callback(self, msg, robot_id):
        """배터리 상태 업데이트 콜백"""
        robot_num = int(robot_id.replace('libro', ''))
        self.robot_battery_states[robot_num] = msg.data
        self.last_message_times[robot_num] = time.time()
        self.update_active_robots()

    def update_active_robots(self):
        """배터리 상태를 기반으로 활성 로봇 목록 업데이트"""
        current_time = time.time()
        active_robots = set()
        
        for robot_num, is_active in self.robot_battery_states.items():
            # 배터리 상태가 True이고 마지막 메시지가 타임아웃 시간 내에 있는 경우만 활성으로 간주
            if is_active and (current_time - self.last_message_times.get(robot_num, 0)) < self.message_timeout:
                active_robots.add(robot_num)
        
        if active_robots != self.active_robots:
            self.active_robots = active_robots
            self.publish_active_robots()
            self.get_logger().info(f'Active robots updated based on battery state: {self.active_robots}')

    def check_topics_callback(self):
        """주기적으로 토픽 리스트를 확인하고 활성화된 로봇을 업데이트"""
        try:
            # 현재 발행 중인 토픽 목록 가져오기
            topic_list = self.get_topic_names_and_types()
            current_robots = set()
            
            # battery_present 토픽을 가진 로봇 찾기
            for topic_name, _ in topic_list:
                match = re.search(r'/(libro\d+)/battery_present', topic_name)
                if match:
                    robot_id = match.group(1)
                    robot_num = int(robot_id.replace('libro', ''))
                    
                    # 토픽의 발행자 수 확인
                    topic_info = self.get_publishers_info_by_topic(topic_name)
                    if topic_info:  # 발행자가 있는 경우만 처리
                        current_robots.add(robot_num)
                        # 새로운 로봇이 발견되면 배터리 토픽 구독
                        if robot_num not in self.robot_battery_states:
                            self.subscribe_to_battery_topic(robot_id)
            
            # 더 이상 존재하지 않는 로봇 제거
            for robot_num in list(self.robot_battery_states.keys()):
                if robot_num not in current_robots:
                    del self.robot_battery_states[robot_num]
                    if robot_num in self.last_message_times:
                        del self.last_message_times[robot_num]
            
            # 활성화된 로봇 목록이 변경되었는지 확인
            if current_robots != self.active_robots:
                self.active_robots = current_robots
                self.publish_active_robots()
                self.get_logger().info(f'Active robots changed: {self.active_robots}')
                
        except Exception as e:
            self.get_logger().error(f'Error checking topics: {str(e)}')

    def publish_active_robots(self):
        """활성화된 로봇 ID 리스트를 발행"""
        # 정렬된 리스트로 변환
        robot_ids = sorted(list(self.active_robots))
        
        # 이전에 발행한 것과 동일한지 확인
        if robot_ids == self.last_published_ids:
            return
            
        # Float32MultiArray 메시지 생성
        msg = Float32MultiArray()
        msg.data = [float(id) for id in robot_ids]
        
        # 메시지 발행
        self.active_robots_publisher.publish(msg)
        self.last_published_ids = robot_ids
        self.get_logger().info(f'Published active robot IDs: {robot_ids}')

def main(args=None):
    rclpy.init(args=args)
    try:
        active_lilbro_checker = ActiveLilbroChecker()
        rclpy.spin(active_lilbro_checker)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if 'active_lilbro_checker' in locals():
            active_lilbro_checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 