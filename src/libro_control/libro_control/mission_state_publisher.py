#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class MissionStatePublisher(Node):
    def __init__(self):
        super().__init__('mission_state_publisher')
        
        # 퍼블리셔 생성
        self.publisher_ = self.create_publisher(Int32, 'pinky_mission_state', 10)
        
        # 서브스크라이버 생성 (상태 업데이트를 위해)
        self.subscription = self.create_subscription(
            Int32,
            'pinky_mission_state_cmd',  # 명령을 받을 새로운 토픽
            self.state_callback,
            10)
            
        # 초기 상태 설정 (기본값 0)
        self.current_state = Int32()
        self.current_state.data = 0
        
        # 타이머 생성 (1Hz로 상태 발행)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Mission State Publisher has started')
        self.get_logger().info('Current mission state: 0')
        self.get_logger().info('You can update mission states (0, 1, 2) using:')
        self.get_logger().info('ros2 topic pub --once /pinky_mission_state_cmd std_msgs/msg/Int32 "{data: 1}"')
        
    def state_callback(self, msg):
        """새로운 상태 명령을 받았을 때 호출되는 콜백"""
        if msg.data in [0, 1, 2]:  # 유효한 상태 값만 허용
            if self.current_state.data != msg.data:
                self.current_state.data = msg.data
                self.get_logger().info(f'Mission state updated to: {msg.data}')
        else:
            self.get_logger().warning(f'Invalid mission state received: {msg.data}. Valid states are 0, 1, 2')
            
    def timer_callback(self):
        """저장된 상태를 주기적으로 발행"""
        self.publisher_.publish(self.current_state)

def main(args=None):
    rclpy.init(args=args)
    mission_state_publisher = MissionStatePublisher()
    rclpy.spin(mission_state_publisher)
    mission_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 