#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from scipy.spatial.transform import Rotation as R
from libro_aruco.aruco_processor import ArucoProcessor
import tf_transformations


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

        # TF 리스너 및 정적 변환 저장을 위한 변수
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_transform_map_to_cam_optical = None  # 조회된 정적 변환 저장
        self.static_transform_lookup_timer = None

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
            f"ArucoPosePublisher 초기화 완료. 카메라: {self.camera_name}, 카메라 광학 프레임: {self.camera_frame}, 맵 프레임: {self.map_frame}")

        # 초기 정적 TF 조회 시도 (CameraInfo 수신 후에도 다시 시도)
        self.attempt_static_tf_lookup()

    def attempt_static_tf_lookup(self):
        if self.static_transform_map_to_cam_optical is not None:
            if self.static_transform_lookup_timer is not None:
                self.static_transform_lookup_timer.cancel()
                self.static_transform_lookup_timer = None
            return True

        try:
            self.get_logger().info(
                f"정적 TF 조회 시도: Target Frame: '{self.map_frame}', Source Frame: '{self.camera_frame}'")
            self.static_transform_map_to_cam_optical = self.tf_buffer.lookup_transform(
                self.map_frame,  # Target frame
                self.camera_frame,  # Source frame
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.get_logger().info(f"정적 TF ('{self.map_frame}' -> '{self.camera_frame}') 조회 성공 및 저장 완료.")
            if self.static_transform_lookup_timer is not None:
                self.static_transform_lookup_timer.cancel()
                self.static_transform_lookup_timer = None
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f"정적 TF ('{self.map_frame}' -> '{self.camera_frame}') 조회 실패: {e}. 5초 후 재시도합니다.")
            if self.static_transform_lookup_timer is None or self.static_transform_lookup_timer.canceled:
                self.static_transform_lookup_timer = self.create_timer(5.0, self.attempt_static_tf_lookup)
            return False
        except Exception as e:
            self.get_logger().error(f"정적 TF 조회 중 예기치 않은 오류: {e}")
            if self.static_transform_lookup_timer is None or self.static_transform_lookup_timer.canceled:
                self.static_transform_lookup_timer = self.create_timer(5.0, self.attempt_static_tf_lookup)
            return False

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)

            # CameraInfo의 frame_id가 파라미터로 받은 self.camera_frame과 일치하는지 확인
            if msg.header.frame_id != self.camera_frame:
                self.get_logger().warn(
                    f"CameraInfo의 frame_id ('{msg.header.frame_id}')가 파라미터 camera_frame ('{self.camera_frame}')과 다릅니다. "
                    f"파라미터 값을 카메라 광학 프레임 ID로 사용합니다.")

            self.aruco_processor = ArucoProcessor(
                camera_matrix=self.camera_matrix,
                dist_coeffs=self.dist_coeffs,
                marker_length=self.marker_size
            )
            self.camera_info_received = True
            self.get_logger().info(f"카메라 정보 수신 및 ArucoProcessor 초기화 완료. 사용된 광학 프레임 ID: {self.camera_frame}")

            # 카메라 정보 수신 후, 아직 정적 TF 조회가 성공하지 못했다면 다시 시도
            if self.static_transform_map_to_cam_optical is None:
                self.attempt_static_tf_lookup()

            # CameraInfo는 한 번만 필요하므로 구독 해제 가능
            # self.destroy_subscription(self.camera_info_sub)
            # self.camera_info_sub = None # 명시적으로 None 처리하여 중복 해제 방지

    def image_callback(self, msg: Image):
        if not self.camera_info_received or self.aruco_processor is None:
            self.get_logger().debug('카메라 정보 또는 ArUco 프로세서가 아직 초기화되지 않았습니다.', throttle_duration_sec=5.0)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        corners, ids, _, rvecs, tvecs = self.aruco_processor.detect_markers(cv_image)
        marker_data_list = self.aruco_processor.get_pose_data(corners, ids, rvecs, tvecs)

        current_time = msg.header.stamp  # 이미지 메시지의 타임스탬프 사용

        if ids is not None and len(marker_data_list) > 0:
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

                # 2. 맵 기준 PoseStamped 발행 (최적화된 TF 조회 및 변환 사용)
                if self.static_transform_map_to_cam_optical:  # 정적 TF가 성공적으로 조회되었을 때만 실행
                    try:
                        # 저장된 정적 변환을 사용하여 카메라 기준 포즈를 맵 기준으로 변환
                        pose_map = tf2_geometry_msgs.do_transform_pose_stamped(pose_cam,
                                                                               self.static_transform_map_to_cam_optical)

                        # --- 맵 기준 포즈 변환 (roll/pitch 제거) 최적화 (tf_transformations 사용) ---
                        # z 위치를 0으로 설정 (2D 평면 가정)
                        pose_map.pose.position.z = 0.0

                        # roll, pitch를 0으로 만들고 yaw만 유지
                        original_q_ros = pose_map.pose.orientation
                        original_q_tf_format = [original_q_ros.x, original_q_ros.y, original_q_ros.z, original_q_ros.w]

                        # 오일러 각으로 변환 (tf_transformations.euler_from_quaternion의 기본 axes는 'sxyz')
                        # 이 함수는 roll, pitch, yaw 순서로 값을 반환하는 것으로 일반적으로 사용됨
                        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(original_q_tf_format)

                        # roll과 pitch를 0으로 설정하고 yaw만 사용하여 새 쿼터니언 생성
                        # tf_transformations.quaternion_from_euler는 roll, pitch, yaw 순서의 인자를 받음
                        modified_q_tf_format = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

                        pose_map.pose.orientation.x = modified_q_tf_format[0]
                        pose_map.pose.orientation.y = modified_q_tf_format[1]
                        pose_map.pose.orientation.z = modified_q_tf_format[2]
                        pose_map.pose.orientation.w = modified_q_tf_format[3]
                        # --- 변환 끝 ---

                        if marker_id not in self.map_pose_publishers:
                            self.map_pose_publishers[marker_id] = self.create_publisher(
                                PoseStamped, f"/aruco{marker_id}/map/pose", 10)
                        self.map_pose_publishers[marker_id].publish(pose_map)

                    except Exception as e:
                        self.get_logger().error(f"맵 기준 포즈 변환 또는 발행 중 오류 발생 (marker {marker_id}): {e}")
                else:
                    if self.camera_info_received:  # 카메라 정보는 받았으나 TF가 아직 준비 안된 경우만 로그 출력
                        self.get_logger().debug(
                            f"정적 TF ({self.map_frame} -> {self.camera_frame})가 아직 준비되지 않아 마커 {marker_id}의 맵 기준 포즈 발행을 건너<0xE1><0x8A><0x9D>니다.",
                            throttle_duration_sec=5.0)

            # 시각화 이미지 발행 (marker_data_list가 비어있지 않을 때만)
            rvecs_viz = np.array([md['rvec'] for md in marker_data_list])
            tvecs_viz = np.array([md['tvec'] for md in marker_data_list])

            ids_viz = np.array([md['id'] for md in marker_data_list]).reshape(-1, 1)  # draw_markers가 요구하는 형태로
            corners_viz = [md['corners'] for md in marker_data_list]  # corners도 매칭

            if len(corners_viz) > 0:  # 실제 그릴 마커가 있을 때만
                cv_image_drawn = self.aruco_processor.draw_markers(cv_image, corners_viz, ids_viz, rvecs_viz, tvecs_viz)

                try:
                    img_msg_out = self.bridge.cv2_to_imgmsg(cv_image_drawn, "bgr8")
                    img_msg_out.header = msg.header
                    img_msg_out.header.frame_id = self.camera_frame
                    self.image_pub.publish(img_msg_out)
                except CvBridgeError as e:
                    self.get_logger().error(f"시각화 이미지 변환 오류: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ArucoPosePublisher 노드 종료 중...')
    finally:
        if rclpy.ok():  # 노드가 이미 종료되지 않았는지 확인
            node.destroy_node()
        if rclpy.ok():  # rclpy 컨텍스트가 유효한지 확인
            rclpy.try_shutdown()  # try_shutdown은 예외 발생 시에도 안전


if __name__ == '__main__':
    main()