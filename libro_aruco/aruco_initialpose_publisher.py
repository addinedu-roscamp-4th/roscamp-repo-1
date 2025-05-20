#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile


class ArucoInitialPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_initialpose_publisher')

        self.declare_parameter('marker_id', 0)  # 추적할 마커 ID
        self.declare_parameter('map_frame', 'map')  # initialpose의 기준 프레임

        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        self.initial_pose_published = False

        qos = QoSProfile(depth=1)  # 최신 데이터 하나만 처리
        self.map_pose_sub = self.create_subscription(
            PoseStamped,
            f"/aruco{self.marker_id}/map/pose",  # 맵 기준 특정 마커 포즈 구독
            self.map_pose_callback,
            qos
        )
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        self.get_logger().info(f"ArucoInitialPosePublisher 초기화. 마커 ID: {self.marker_id}, 맵 프레임: {self.map_frame}")

    def map_pose_callback(self, msg: PoseStamped):
        if not self.initial_pose_published:
            if msg.header.frame_id != self.map_frame:
                self.get_logger().warn(
                    f"수신된 맵 포즈의 frame_id ({msg.header.frame_id})가 설정된 map_frame ({self.map_frame})과 다릅니다."
                )
                # 필요시 여기서 변환 로직을 추가하거나, 발행 노드에서 frame_id를 정확히 설정하도록 보장해야 함.
                # 현재는 동일하다고 가정.

            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.stamp = self.get_clock().now().to_msg()
            initial_pose.header.frame_id = self.map_frame  # 또는 msg.header.frame_id
            initial_pose.pose.pose = msg.pose

            # 예시 공분산 (필요에 따라 조정)
            initial_pose.pose.covariance = [
                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,  # y
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # z (2D 네비게이션에서는 무시될 수 있음)
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # roll
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # pitch
                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942  # yaw
            ]

            self.initial_pose_pub.publish(initial_pose)
            self.initial_pose_published = True
            self.get_logger().info(f"Initial pose 발행 완료: {self.get_namespace()}/initialpose")

            # 한 번 발행 후 종료하거나, 구독을 중지할 수 있음
            # self.destroy_subscription(self.map_pose_sub)
            # self.get_logger().info("Initial pose 발행 후 구독 중지.")
            # 또는 rclpy.shutdown() 호출로 노드 자체 종료
            # raise SystemExit # 또는 rclpy.utilities.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoInitialPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info('ArucoInitialPosePublisher shutting down.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
