#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from libro_aruco.aruco_processor import ArucoProcessor  # 사용자 정의 모듈


class ArucoPosePublisher(Node):
    def __init__(self):
        super().__init__('aruco_pose_publisher')

        # 파라미터 선언
        self.declare_parameter('camera_name', 'global_camera')
        self.declare_parameter('cameral_frame', 'global_camera_link')
        self.declare_parameter('marker_size', 0.07)  # ArUco 마커의 실제 크기 (미터)
        self.declare_parameter('map_frame', 'map')

        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.camera_frame = self.get_parameter('cameral_frame').get_parameter_value().string_value
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        # 변수 초기화
        self.bridge = CvBridge()
        self.aruco_processor = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False

        # TF 리스너 (맵 기준 Pose 계산용)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 구독자
        self.camera_info_sub = self.create_subscription(
            CameraInfo, f"/{self.camera_name}/camera_info", self.camera_info_callback, 10)
        self.image_sub = self.create_subscription(
            Image, f"/{self.camera_name}/image_raw", self.image_callback, 10)

        # 발행자
        self.cam_pose_publishers = {}  # 마커 ID별 카메라 기준 Pose 발행자
        self.map_pose_publishers = {}  # 마커 ID별 맵 기준 Pose 발행자
        self.image_pub = self.create_publisher(  # 시각화된 이미지 발행자
            Image, f"/{self.camera_name}/image_aruco", 10)

        self.get_logger().info(
            f"ArucoPosePublisher 초기화. 카메라: {self.camera_name}, 광학 프레임: {self.camera_frame}, 맵 프레임: {self.map_frame}")

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            # CameraInfo의 frame_id가 실제 카메라 광학 프레임과 다를 수 있으므로, 파라미터 사용 권장
            # self.camera_frame = msg.header.frame_id # 필요시 이렇게 설정할 수도 있음

            self.aruco_processor = ArucoProcessor(
                camera_matrix=self.camera_matrix,
                dist_coeffs=self.dist_coeffs,
                marker_length=self.marker_size
            )
            self.camera_info_received = True
            self.get_logger().info(f"카메라 정보 수신 및 ArucoProcessor 초기화 완료. 광학 프레임 ID: {self.camera_frame}")
            # CameraInfo는 한 번만 필요하므로 구독 해제 가능
            # self.destroy_subscription(self.camera_info_sub)

    def image_callback(self, msg: Image):
        if not self.camera_info_received or self.aruco_processor is None:
            self.get_logger().debug('카메라 정보 또는 ArUco 프로세서 미초기화.', throttle_duration_sec=5.0)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        corners, ids, _, rvecs, tvecs = self.aruco_processor.detect_markers(cv_image)
        marker_data_list = self.aruco_processor.get_pose_data(corners, ids, rvecs, tvecs)

        current_time = msg.header.stamp

        if ids is not None:
            for marker_data in marker_data_list:
                marker_id = marker_data['id']

                # 1. 카메라 기준 PoseStamped 발행
                pose_cam = PoseStamped()
                pose_cam.header.stamp = current_time
                pose_cam.header.frame_id = self.camera_frame  # ArUco 결과는 카메라 광학 프레임 기준

                pose_cam.pose.position.x = float(marker_data['tvec'][0])
                pose_cam.pose.position.y = float(marker_data['tvec'][1])
                pose_cam.pose.position.z = float(marker_data['tvec'][2])
                pose_cam.pose.orientation.x = float(marker_data['quaternion'][0])
                pose_cam.pose.orientation.y = float(marker_data['quaternion'][1])
                pose_cam.pose.orientation.z = float(marker_data['quaternion'][2])
                pose_cam.pose.orientation.w = float(marker_data['quaternion'][3])

                if marker_id not in self.cam_pose_publishers:
                    self.cam_pose_publishers[marker_id] = self.create_publisher(
                        PoseStamped, f"/aruco{marker_id}/cam/pose", 10)
                self.cam_pose_publishers[marker_id].publish(pose_cam)

                # 2. 맵 기준 PoseStamped 발행
                try:
                    transform_map_to_cam_optical = self.tf_buffer.lookup_transform(
                        self.map_frame, self.camera_frame, rclpy.time.Time())

                    pose_map = tf2_geometry_msgs.do_transform_pose_stamped(pose_cam, transform_map_to_cam_optical)
                    # pose_map.pose.position.z = 0.0

                    if marker_id not in self.map_pose_publishers:
                        self.map_pose_publishers[marker_id] = self.create_publisher(
                            PoseStamped, f"/aruco{marker_id}/map/pose", 10)
                    self.map_pose_publishers[marker_id].publish(pose_map)

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                    self.get_logger().warn(
                        f"TF transform error from {self.camera_frame} to {self.map_frame}: {e}",
                        throttle_duration_sec=5.0)
                except Exception as e:
                    self.get_logger().error(f"Error transforming pose for marker {marker_id}: {e}")

            # 시각화 이미지 발행
            if len(marker_data_list) > 0:
                rvecs_viz = np.array([md['rvec'] for md in marker_data_list])
                tvecs_viz = np.array([md['tvec'] for md in marker_data_list])
                # corners와 ids는 detect_markers에서 나온 모든 감지된 마커 기준
                # draw_markers가 내부적으로 id와 매칭하여 rvecs_viz, tvecs_viz를 사용해야 함
                cv_image_drawn = self.aruco_processor.draw_markers(cv_image, corners, ids, rvecs_viz, tvecs_viz)

                img_msg_out = self.bridge.cv2_to_imgmsg(cv_image_drawn, "bgr8")
                img_msg_out.header = msg.header  # 원본 이미지 헤더 사용 (스탬프, 프레임 ID)
                img_msg_out.header.frame_id = self.camera_frame  # 또는 msg.header.frame_id
                self.image_pub.publish(img_msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ArucoPosePublisher shutting down.')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
