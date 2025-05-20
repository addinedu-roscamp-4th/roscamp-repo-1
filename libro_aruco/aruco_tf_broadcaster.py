#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformListener, TransformBroadcaster, Buffer
import tf2_geometry_msgs
import rclpy.logging

class ArucoMapTfBroadcaster(Node):
    def __init__(self):
        super().__init__('aruco_tf_broadcaster')
        self.logger = rclpy.logging.get_logger('aruco_tf_broadcaster')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('marker_id', 0)

        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.marker_frame = f"aruco{self.marker_id}/map"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        qos_profile = QoSProfile(depth=10)

        self.subscription = self.create_subscription(
            PoseStamped,
            f"/{self.marker_frame}/pose", # 맵 기준 마커 포즈 토픽
            self.pose_callback,
            qos_profile)

        self.get_logger().info(
            f"ArucoMapTfBroadcaster 초기화. Subscribing to: /aruco{self.marker_id}/map/pose, "
            f"Publishing TF: {self.map_frame} -> {self.marker_frame}"
        )

    def pose_callback(self, msg: PoseStamped):
        # msg.header.frame_id는 ArUco 마커의 포즈가 정의된 카메라 프레임 ID
        if msg.header.frame_id != self.map_frame:
            self.logger.warn_once(
                f"수신된 PoseStamped의 frame_id ({msg.header.frame_id})가 "
                f"예상된 map_frame ({self.map_frame})과 다릅니다. TF 발행이 부정확할 수 있습니다."
            )

        # 2. 맵 기준 TF 발행 (map -> aruco_map_ID)
        try:
            t_map_marker = TransformStamped()
            t_map_marker.header.stamp = self.get_clock().now().to_msg()  # 또는 msg.header.stamp 사용
            t_map_marker.header.frame_id = self.map_frame  # 부모 프레임
            t_map_marker.child_frame_id = self.marker_frame  # 자식 프레임 (예: aruco0/map)

            t_map_marker.transform.translation.x = msg.pose.position.x
            t_map_marker.transform.translation.y = msg.pose.position.y
            t_map_marker.transform.translation.z = msg.pose.position.z  # 이미 0.0
            t_map_marker.transform.rotation = msg.pose.orientation  # 이미 roll, pitch 0

            self.tf_broadcaster.sendTransform(t_map_marker)
            self.logger.debug(f"맵 기준 마커 TF 발행: {t_map_marker.header.frame_id} -> {t_map_marker.child_frame_id}")

        except Exception as e:
            self.logger.error(f"TF 발행 중 오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMapTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
