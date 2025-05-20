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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            PoseStamped,
            f"/aruco{self.marker_id}/cam/pose", # 카메라 기준 마커 포즈 토픽
            self.pose_callback,
            qos_profile)
        self.subscription

    def pose_callback(self, msg: PoseStamped):
        # msg.header.frame_id는 ArUco 마커의 포즈가 정의된 카메라 프레임 ID
        camera_frame_id = msg.header.frame_id

        # 1. 카메라 기준 TF 발행 (camera_frame -> aruco_cam_ID)
        t_cam_marker = TransformStamped()
        t_cam_marker.header.stamp = self.get_clock().now().to_msg()
        t_cam_marker.header.frame_id = camera_frame_id
        t_cam_marker.child_frame_id = f'aruco{self.marker_id}/cam'

        t_cam_marker.transform.translation.x = msg.pose.position.x
        t_cam_marker.transform.translation.y = msg.pose.position.y
        t_cam_marker.transform.translation.z = msg.pose.position.z
        t_cam_marker.transform.rotation = msg.pose.orientation # Pose의 orientation 직접 사용
        self.tf_broadcaster.sendTransform(t_cam_marker)
        self.logger.debug(f"카메라 기준 마커 TF 발행: {t_cam_marker.header.frame_id} -> {t_cam_marker.child_frame_id}")

        # 2. 맵 기준 TF 발행 (map -> aruco_map_ID)
        try:
            # map 프레임에서 camera_frame_id (마커 포즈가 정의된 프레임) 까지의 변환(TransformStamped)을 조회
            transform_map_to_camera = self.tf_buffer.lookup_transform(
                self.map_frame,    # Target frame
                camera_frame_id,   # Source frame
                rclpy.time.Time()  # 최신 변환 사용
            )

            # PoseStamped 메시지(카메라->마커)를 map 프레임 기준으로 변환
            # do_transform_pose(PoseStamped, TransformStamped) -> PoseStamped
            pose_map_to_marker = tf2_geometry_msgs.do_transform_pose(msg.pose, transform_map_to_camera)

            # 변환된 Pose를 사용하여 TF 발행
            t_map_marker = TransformStamped()
            t_map_marker.header.stamp = self.get_clock().now().to_msg() # t_cam_marker와 동일 시간 사용 가능
            t_map_marker.header.frame_id = self.map_frame
            t_map_marker.child_frame_id = f'aruco{self.marker_id}/map'

            t_map_marker.transform.translation.x = pose_map_to_marker.position.x
            t_map_marker.transform.translation.y = pose_map_to_marker.position.y
            t_map_marker.transform.translation.z = pose_map_to_marker.position.z
            t_map_marker.transform.rotation = pose_map_to_marker.orientation # 변환된 Pose의 orientation 사용

            self.tf_broadcaster.sendTransform(t_map_marker)
            self.logger.debug(f"맵 기준 마커 TF 발행: {t_map_marker.header.frame_id} -> {t_map_marker.child_frame_id}")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.logger.warn(f"TF 예외 발생 (map -> aruco{self.marker_id})_map: {e}", throttle_duration_sec=5.0)
        except Exception as e:
            self.logger.error(f"TF 발행 중 일반 오류 발생 (map -> aruco{self.marker_id}_map): {e}")

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
