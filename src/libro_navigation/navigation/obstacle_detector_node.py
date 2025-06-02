"""

/scan을 구독하여 일정거리내에 장애물이 감지되면 True를 반환하는 노드

"""

import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import rclpy

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.namespace = self.get_namespace()
        self.publisher = self.create_publisher(Bool, f'{self.namespace}/obstacle_detected', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            f'{self.namespace}/scan',
            self.scan_callback,
            10
        )

        # 원하는 탐지 각도 범위 (예: -20도 ~ +20도)
        self.angle_range_deg = (-180.0, -160.0)
        self.min_range = 0.05
        self.max_range = 0.1

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # 라디안 단위로 각도 범위 설정
        angle_start = math.radians(self.angle_range_deg[0])
        angle_end = math.radians(self.angle_range_deg[1])

        # 인덱스 범위 계산
        start_idx = int((angle_start - angle_min) / angle_increment)
        end_idx = int((angle_end - angle_min) / angle_increment)

        # 인덱스 안전하게 클리핑
        start_idx = max(0, start_idx)
        end_idx = min(len(msg.ranges), end_idx)

        # 범위 내 값 검사
        has_obstacle = any(self.min_range < d < self.max_range for d in msg.ranges[start_idx:end_idx])
        self.publisher.publish(Bool(data=has_obstacle))

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
