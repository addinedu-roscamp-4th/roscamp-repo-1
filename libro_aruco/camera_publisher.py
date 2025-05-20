#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import yaml
import os


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.declare_parameter('camera_device', '/dev/video2')
        self.declare_parameter('camera_name', 'global_camera')
        self.declare_parameter('calibration_file', '../params/camera_param.yaml')
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)

        self.camera_device_path = self.get_parameter('camera_device').get_parameter_value().string_value
        self.camera_name_param = self.get_parameter('camera_name').get_parameter_value().string_value
        self.calibration_file_path = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.publish_rate_val = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.frame_width_val = self.get_parameter('frame_width').get_parameter_value().integer_value
        self.frame_height_val = self.get_parameter('frame_height').get_parameter_value().integer_value

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(self.camera_device_path)
        if not self.cap.isOpened():
            self.get_logger().error(f"카메라 장치 {self.camera_device_path}를 열 수 없습니다!")
            rclpy.shutdown()
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width_val)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height_val)
        self.cap.set(cv2.CAP_PROP_FPS, self.publish_rate_val)

        self.actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.get_logger().info(f"카메라 실제 해상도: {self.actual_width}x{self.actual_height}")

        self.camera_frame_id_str = f'{self.camera_name_param}_link'  # 예: camera_link

        # CameraInfo 메시지용 변수
        self.cam_info_msg = CameraInfo()
        self.K_matrix = None
        self.D_coeffs = None
        self.R_matrix = None
        self.P_matrix = None
        self.img_width_from_yaml = self.frame_width_val
        self.img_height_from_yaml = self.frame_height_val
        self.distortion_model_str = "plumb_bob"

        self.mapx = None
        self.mapy = None
        self.new_camera_matrix = None

        if os.path.exists(self.calibration_file_path):
            self.load_camera_calibration()
        else:
            self.get_logger().warn(f"캘리브레이션 파일 '{self.calibration_file_path}'을(를) 찾을 수 없습니다. 왜곡 보정 없이 진행합니다.")
            # 기본 CameraInfo 설정 (왜곡 없음)
            self.K_matrix = np.eye(3, dtype=float)  # 기본 단위행렬
            # 기본 K 행렬 (Pinhole 모델 가정, 초점거리=너비, 중심=너비/2, 높이/2)
            self.K_matrix[0, 0] = float(self.actual_width)
            self.K_matrix[1, 1] = float(self.actual_width)  # 보통 높이와 비슷하거나 같음
            self.K_matrix[0, 2] = float(self.actual_width / 2)
            self.K_matrix[1, 2] = float(self.actual_height / 2)

            self.D_coeffs = np.zeros(5, dtype=float)  # 왜곡 없음
            self.R_matrix = np.eye(3, dtype=float)  # 단위 행렬
            self.P_matrix = np.zeros((3, 4), dtype=float)  # P = [K | 0]
            self.P_matrix[:3, :3] = self.K_matrix
            self.img_width_from_yaml = self.actual_width
            self.img_height_from_yaml = self.actual_height

        self.image_publisher = self.create_publisher(Image, f"/{self.camera_name_param}/image_raw", 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, f"/{self.camera_name_param}/camera_info", 10)

        self.timer = self.create_timer(1.0 / self.publish_rate_val, self.timer_callback)
        self.get_logger().info(f"카메라 퍼블리셔 ({self.camera_name_param}) 초기화 완료. 발행 주기: {self.publish_rate_val} Hz")

    def load_camera_calibration(self):
        try:
            with open(self.calibration_file_path, 'r') as f:
                calib_data = yaml.safe_load(f)
                self.img_width_from_yaml = calib_data['image_width']
                self.img_height_from_yaml = calib_data['image_height']
                self.K_matrix = np.array(calib_data['camera_matrix']['data'], dtype=float).reshape((3, 3))
                self.D_coeffs = np.array(calib_data['distortion_coefficients']['data'], dtype=float)
                self.R_matrix = np.array(calib_data['rectification_matrix']['data'], dtype=float).reshape((3, 3))
                self.P_matrix = np.array(calib_data['projection_matrix']['data'], dtype=float).reshape((3, 4))
                self.distortion_model_str = calib_data.get('distortion_model', 'plumb_bob')  # 없을 경우 plumb_bob

                # 왜곡 보정을 위한 remap 매트릭스 계산
                # 실제 카메라 해상도(self.actual_width, self.actual_height) 기준으로 undistort map 생성
                self.new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
                    self.K_matrix, self.D_coeffs,
                    (self.img_width_from_yaml, self.img_height_from_yaml),
                    0,  # alpha=1: 모든 픽셀 유지, 0: 유효한 픽셀만 (잘릴 수 있음)
                    (self.actual_width, self.actual_height)
                )
                self.mapx, self.mapy = cv2.initUndistortRectifyMap(
                    self.K_matrix, self.D_coeffs, self.R_matrix, self.new_camera_matrix,
                    (self.actual_width, self.actual_height), cv2.CV_32FC1
                )
                self.get_logger().info(f"카메라 캘리브레이션 로드 완료: {self.calibration_file_path}")

        except Exception as e:
            self.get_logger().error(f"캘리브레이션 파일 로드 또는 처리 중 오류: {e}")
            # 오류 발생 시 기본값 사용 (위 생성자에서 이미 처리)
            self.K_matrix = np.eye(3, dtype=float)
            self.K_matrix[0, 0] = float(self.actual_width)
            self.K_matrix[1, 1] = float(self.actual_width)
            self.K_matrix[0, 2] = float(self.actual_width / 2)
            self.K_matrix[1, 2] = float(self.actual_height / 2)
            self.D_coeffs = np.zeros(5, dtype=float)
            self.R_matrix = np.eye(3, dtype=float)
            self.P_matrix = np.zeros((3, 4), dtype=float)
            self.P_matrix[:3, :3] = self.K_matrix
            self.img_width_from_yaml = self.actual_width
            self.img_height_from_yaml = self.actual_height
            self.mapx, self.mapy = None, None  # 왜곡 보정 안 함

    def timer_callback(self):
        if not self.cap.isOpened():
            self.get_logger().warn("카메라가 열려있지 않습니다. 재시도 중...", throttle_duration_sec=5)
            # 재연결 시도 로직
            self.cap.release()
            self.cap = cv2.VideoCapture(self.camera_device_path)
            if self.cap.isOpened():
                self.get_logger().info("카메라 재연결 성공.")
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width_val)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height_val)
                self.cap.set(cv2.CAP_PROP_FPS, self.publish_rate_val)
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("카메라에서 프레임을 읽을 수 없습니다.", throttle_duration_sec=5)
            return

        # 왜곡 보정 (mapx, mapy가 존재하면)
        if self.mapx is not None and self.mapy is not None:
            processed_frame = cv2.remap(frame, self.mapx, self.mapy, cv2.INTER_LINEAR)
        else:
            processed_frame = frame  # 왜곡 보정 정보 없으면 원본 사용

        current_time_msg = self.get_clock().now().to_msg()

        # Image 메시지 발행
        img_msg = self.bridge.cv2_to_imgmsg(processed_frame, "bgr8")
        img_msg.header.stamp = current_time_msg
        img_msg.header.frame_id = self.camera_frame_id_str
        self.image_publisher.publish(img_msg)

        # CameraInfo 메시지 발행 (캘리브레이션 정보가 유효할 때)
        if self.K_matrix is not None and self.D_coeffs is not None and self.R_matrix is not None and self.P_matrix is not None:
            self.cam_info_msg.header.stamp = current_time_msg
            self.cam_info_msg.header.frame_id = self.camera_frame_id_str
            self.cam_info_msg.height = self.img_height_from_yaml  # 캘리브레이션 기준 높이
            self.cam_info_msg.width = self.img_width_from_yaml  # 캘리브레이션 기준 너비
            self.cam_info_msg.distortion_model = self.distortion_model_str
            self.cam_info_msg.d = self.D_coeffs.flatten().tolist()
            # K는 CameraInfo에서 3x3이므로, new_camera_matrix (왜곡 보정 후) 또는 K_matrix (원본) 사용
            # 일반적으로 왜곡 보정 후의 파라미터를 CameraInfo에 넣음
            if self.new_camera_matrix is not None:
                self.cam_info_msg.k = self.new_camera_matrix.flatten().tolist()
                # P_matrix도 new_camera_matrix 기준으로 업데이트
                P_temp = np.zeros((3, 4), dtype=float)
                P_temp[:3, :3] = self.new_camera_matrix
                self.cam_info_msg.p = P_temp.flatten().tolist()

            else:  # new_camera_matrix가 없으면(왜곡보정 실패 등) 원본 K, P 사용
                self.cam_info_msg.k = self.K_matrix.flatten().tolist()
                self.cam_info_msg.p = self.P_matrix.flatten().tolist()

            self.cam_info_msg.r = self.R_matrix.flatten().tolist()
            # Binning x, y 는 0으로 설정
            self.cam_info_msg.binning_x = 0
            self.cam_info_msg.binning_y = 0
            # ROI는 전체 이미지로 설정
            self.cam_info_msg.roi.x_offset = 0
            self.cam_info_msg.roi.y_offset = 0
            self.cam_info_msg.roi.height = 0  # 0은 roi 사용 안 함을 의미
            self.cam_info_msg.roi.width = 0
            self.cam_info_msg.roi.do_rectify = False  # 이미지가 rectify된 경우 True

            self.camera_info_publisher.publish(self.cam_info_msg)

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.get_logger().info("카메라 장치를 해제합니다.")
            self.cap.release()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:  # 노드가 생성되었다면
            node.get_logger().fatal(f"CameraPublisher 노드 실행 중 치명적 오류: {e}")
    finally:
        if node is not None and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():  # rclpy.ok()는 shutdown 이후 False가 됨
            rclpy.shutdown()


if __name__ == '__main__':
    main()
