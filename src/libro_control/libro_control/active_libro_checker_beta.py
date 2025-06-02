import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
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
        if 'pinky_battery_present' in msg.msg:
            match = re.search(r'/(libro\d+)/pinky_battery_present', msg.msg)
            if match:
                robot_id = match.group(1)
                robot_num = int(robot_id.replace('libro', ''))
                if robot_num not in self.active_robots:
                    self.active_robots.add(robot_num)
                    self.publish_active_robots()
                    self.get_logger().info(f'New robot detected: {robot_id}')

    def check_topics_callback(self):
        """주기적으로 토픽 리스트를 확인하고 활성화된 로봇을 업데이트"""
        try:
            # 현재 발행 중인 토픽 목록 가져오기
            topic_list = self.get_topic_names_and_types()
            current_robots = set()
            
            # pinky_battery_present 토픽을 가진 로봇 찾기
            for topic_name, _ in topic_list:
                match = re.search(r'/(libro\d+)/pinky_battery_present', topic_name)
                if match:
                    robot_id = match.group(1)
                    robot_num = int(robot_id.replace('libro', ''))
                    current_robots.add(robot_num)
            
            # 활성화된 로봇 목록이 변경되었는지 확인
            if current_robots != self.active_robots:
                self.active_robots = current_robots
                self.publish_active_robots()
                self.get_logger().info(f'Active robots changed: {self.active_robots}')
                
        except Exception as e:
            self.get_logger().error(f'Error checking topics: {str(e)}')

    def publish_active_robots(self):
        """활성화된 로봇 ID 리스트를 발행"""
        if not self.active_robots:
            return
            
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