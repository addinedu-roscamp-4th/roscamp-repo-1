#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.qos import QoSProfile
import numpy as np
import tf_transformations

from libro_aruco.low_pass_filter import LowPassFilter


class ArucoOdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('aruco_odom_tf_broadcaster')

        self.declare_parameter('marker_id', 0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # LPF 파라미터 (차단 주파수와 샘플링 주기에 따라 결정, 0 < alpha <= 1)
        # alpha가 작을수록 강한 필터링 (느린 반응), 클수록 약한 필터링 (빠른 반응)
        self.declare_parameter('lpf_alpha_linear', 0.1) # 선속도 LPF alpha
        self.declare_parameter('lpf_alpha_angular', 0.1) # 각속도 LPF alpha

        # 공분산 파라미터 (작을수록 신뢰도 높음)
        self.declare_parameter('pose_cov_x', 0.01)       # m^2
        self.declare_parameter('pose_cov_y', 0.01)       # m^2
        self.declare_parameter('pose_cov_z', 1e9)        # m^2 (2D에서는 매우 크게)
        self.declare_parameter('pose_cov_roll', 1e9)     # rad^2 (2D에서는 매우 크게)
        self.declare_parameter('pose_cov_pitch', 1e9)    # rad^2 (2D에서는 매우 크게)
        self.declare_parameter('pose_cov_yaw', 0.005)    # rad^2

        self.declare_parameter('twist_cov_vx', 0.1)      # (m/s)^2
        self.declare_parameter('twist_cov_vy', 0.1)      # (m/s)^2
        self.declare_parameter('twist_cov_vz', 1e9)      # (m/s)^2
        self.declare_parameter('twist_cov_wx', 1e9)      # (rad/s)^2
        self.declare_parameter('twist_cov_wy', 1e9)      # (rad/s)^2
        self.declare_parameter('twist_cov_wz', 0.05)     # (rad/s)^2


        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.map_frame_id = self.get_parameter('map_frame').get_parameter_value().string_value
        odom_frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
        base_frame_id = self.get_parameter('base_frame').get_parameter_value().string_value

        # base_footprint에서 lidar_lidar_mount까지의 z축 오프셋
        z_offset_base_to_lidar_mount = 0.072  # 미터

        # lidar_lidar_mount에서 아르코 마커까지의 z축 오프셋
        z_offset_lidar_mount_to_marker = 0.02  # 미터

        # base_footprint에서 아르코 마커까지의 총 z축 오프셋
        total_z_offset_base_to_marker = z_offset_base_to_lidar_mount + z_offset_lidar_mount_to_marker

        # base_footprint에서 아르코 마커까지의 변환 행렬 (T_base_marker)
        # 마커는 base_footprint 기준으로 x, y, yaw 오프셋 없이 z축으로만 이동
        trans_base_to_marker = np.array([0.0, 0.0, total_z_offset_base_to_marker])
        # 마커의 방향은 base_footprint의 방향과 동일하다고 가정 (회전 없음)
        rot_base_to_marker_quat = tf_transformations.quaternion_from_euler(0, 0, 0)  # RPY

        T_base_marker_mat = tf_transformations.quaternion_matrix(rot_base_to_marker_quat)
        T_base_marker_mat[0:3, 3] = trans_base_to_marker

        # 아르코 마커에서 base_footprint까지의 변환 행렬 (T_marker_base)
        # T_marker_base = inv(T_base_marker)
        self.T_marker_base_footprint_mat = tf_transformations.inverse_matrix(T_base_marker_mat)

        # LPF 인스턴스 생성
        lpf_alpha_linear = self.get_parameter('lpf_alpha_linear').get_parameter_value().double_value
        lpf_alpha_angular = self.get_parameter('lpf_alpha_angular').get_parameter_value().double_value
        self.lpf_vx = LowPassFilter(lpf_alpha_linear)
        self.lpf_vy = LowPassFilter(lpf_alpha_linear)
        self.lpf_wz = LowPassFilter(lpf_alpha_angular)

        # 공분산 값 로드
        self.pose_covariance_values = [
            self.get_parameter('pose_cov_x').get_parameter_value().double_value,
            self.get_parameter('pose_cov_y').get_parameter_value().double_value,
            self.get_parameter('pose_cov_z').get_parameter_value().double_value,
            self.get_parameter('pose_cov_roll').get_parameter_value().double_value,
            self.get_parameter('pose_cov_pitch').get_parameter_value().double_value,
            self.get_parameter('pose_cov_yaw').get_parameter_value().double_value
        ]
        self.twist_covariance_values = [
            self.get_parameter('twist_cov_vx').get_parameter_value().double_value,
            self.get_parameter('twist_cov_vy').get_parameter_value().double_value,
            self.get_parameter('twist_cov_vz').get_parameter_value().double_value,
            self.get_parameter('twist_cov_wx').get_parameter_value().double_value,
            self.get_parameter('twist_cov_wy').get_parameter_value().double_value,
            self.get_parameter('twist_cov_wz').get_parameter_value().double_value
        ]


        current_namespace = self.get_namespace()
        if current_namespace == '/':
            current_namespace = ""

        if not odom_frame_id.startswith('/') and current_namespace:
            self.robot_odom_frame_id = f"{current_namespace}/{odom_frame_id}"
        else:
            self.robot_odom_frame_id = odom_frame_id.lstrip('/')

        if not base_frame_id.startswith('/') and current_namespace:
            self.robot_base_frame_id = f"{current_namespace}/{base_frame_id}"
        else:
            self.robot_base_frame_id = base_frame_id.lstrip('/')

        # StaticTransformBroadcaster 초기화 (map -> odom 용)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # 일반 TransformBroadcaster 초기화 (odom -> base_footprint 용)
        self.tf_broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(depth=1) # ArUco 마커 포즈는 최신 것 하나만 중요
        self.map_pose_sub = self.create_subscription(
            PoseStamped,
            f"/aruco{self.marker_id}/map/pose",
            self.map_pose_callback,
            qos
        )

        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.get_logger().info(f"Traditional odometry publisher on topic: {self.get_namespace()}/odom")
        self.get_logger().info(f"TF: {self.map_frame_id} -> {self.robot_odom_frame_id} -> {self.robot_base_frame_id}")

        self.initial_map_pose_to_odom_tf = None # map -> odom 발행할 static TF 메시지 저장 변수
        self.initial_marker_pose_in_map = None  # 마커의 초기 map 기준 포즈
        self.previous_marker_pose_in_map = None # 이전 마커의 map 기준 포즈 (속도 계산용)
        self.previous_time = None               # 이전 시간 (속도 계산용)
        self.static_tf_published = False  # static TF가 발행되었는지 확인하는 플래그


    def map_pose_callback(self, msg: PoseStamped):
        current_time_rclpy = self.get_clock().now()
        current_time_msg = current_time_rclpy.to_msg()

        if msg.header.frame_id != self.map_frame_id:
            self.get_logger().warn_once(
                f"수신된 마커 포즈의 frame_id ({msg.header.frame_id})가 설정된 map_frame ({self.map_frame_id})과 다릅니다."
            )
            # frame_id가 다르면 처리를 중단하거나, 변환을 시도해야 함

        # 현재 ArUco 마커의 map 기준 포즈
        current_marker_pose_in_map = msg.pose

        if self.initial_marker_pose_in_map is None:
            # 시스템 시작 시 첫 번째 마커 위치를 기록
            self.initial_marker_pose_in_map = current_marker_pose_in_map
            self.previous_marker_pose_in_map = current_marker_pose_in_map
            self.previous_time = current_time_rclpy

            # map -> odom TF를 초기 마커 위치에 고정하여 발행 준비
            # odom 프레임은 map 프레임 기준으로 초기 마커 위치에 생성되며, z축 회전(yaw)만 가짐
            self.initial_map_pose_to_odom_tf = TransformStamped()
            self.initial_map_pose_to_odom_tf.header.stamp = current_time_msg
            self.initial_map_pose_to_odom_tf.header.frame_id = self.map_frame_id
            self.initial_map_pose_to_odom_tf.child_frame_id = self.robot_odom_frame_id

            self.initial_map_pose_to_odom_tf.transform.translation.x = self.initial_marker_pose_in_map.position.x
            self.initial_map_pose_to_odom_tf.transform.translation.y = self.initial_marker_pose_in_map.position.y
            self.initial_map_pose_to_odom_tf.transform.translation.z = self.initial_marker_pose_in_map.position.z # 2D 환경이면 0으로 할 수도 있음

            # 초기 odom 프레임의 yaw는 마커의 초기 yaw를 따르도록 설정
            # roll, pitch는 0으로 만들어 odom 프레임이 수평을 유지하도록 함
            initial_q = [
                self.initial_marker_pose_in_map.orientation.x,
                self.initial_marker_pose_in_map.orientation.y,
                self.initial_marker_pose_in_map.orientation.z,
                self.initial_marker_pose_in_map.orientation.w
            ]
            _, _, initial_yaw = tf_transformations.euler_from_quaternion(initial_q)
            q_odom_in_map = tf_transformations.quaternion_from_euler(0.0, 0.0, initial_yaw)

            self.initial_map_pose_to_odom_tf.transform.rotation.x = q_odom_in_map[0]
            self.initial_map_pose_to_odom_tf.transform.rotation.y = q_odom_in_map[1]
            self.initial_map_pose_to_odom_tf.transform.rotation.z = q_odom_in_map[2]
            self.initial_map_pose_to_odom_tf.transform.rotation.w = q_odom_in_map[3]

            # 1. TF 발행: map -> odom
            self.static_tf_broadcaster.sendTransform(self.initial_map_pose_to_odom_tf)
            self.static_tf_published = True  # 발행됨
            self.get_logger().info(
                f"Static TF {self.map_frame_id} -> {self.robot_odom_frame_id} published: \n"
                f"Translation: [{self.initial_map_pose_to_odom_tf.transform.translation.x:.2f}, "
                f"{self.initial_map_pose_to_odom_tf.transform.translation.y:.2f}, "
                f"{self.initial_map_pose_to_odom_tf.transform.translation.z:.2f}]\n"
                f"Rotation (yaw): {initial_yaw:.2f} rad"
            )
            return  # 첫 유효한 콜백에서 static TF 발행 후, 다음 로직은 실행 안함.

        # static TF가 아직 발행되지 않았으면(초기 마커 정보 대기 중), 나머지 로직 수행 안 함
        if not self.static_tf_published:
            self.get_logger().info("Waiting for initial ArUco marker pose to publish static TF...", throttle_duration_sec=5)
            # 이전 상태도 업데이트 해주어야 첫 속도 계산 시 오류 방지
            if self.initial_marker_pose_in_map is None:  # 아직 첫 마커도 못 받았으면
                self.initial_marker_pose_in_map = current_marker_pose_in_map  # 임시로 현재 값을 넣어둠 (다음 콜백에서 static TF 생성)
                self.previous_marker_pose_in_map = current_marker_pose_in_map
                self.previous_time = current_time_rclpy
            return

        # 2. 로봇의 odom 기준 포즈 계산 (base_footprint in odom frame)
        # map -> initial_marker_pose (T_map_initial_marker)
        # map -> current_marker_pose (T_map_current_marker)
        # odom 프레임은 initial_marker_pose에 위치하고 map과 동일한 z축 방향을 가진다고 가정 (위에서 yaw만 사용)
        # T_odom_map = T_map_odom^-1
        # T_odom_current_marker = T_odom_map * T_map_current_marker
        # base_footprint는 current_marker와 동일하다고 가정 (마커가 로봇 중심에 부착)

        # T_map_odom의 역행렬 (T_odom_map)
        q_map_odom = [
            self.initial_map_pose_to_odom_tf.transform.rotation.x,
            self.initial_map_pose_to_odom_tf.transform.rotation.y,
            self.initial_map_pose_to_odom_tf.transform.rotation.z,
            self.initial_map_pose_to_odom_tf.transform.rotation.w
        ]
        trans_map_odom = [
            self.initial_map_pose_to_odom_tf.transform.translation.x,
            self.initial_map_pose_to_odom_tf.transform.translation.y,
            self.initial_map_pose_to_odom_tf.transform.translation.z
        ]
        T_map_odom_mat = tf_transformations.quaternion_matrix(q_map_odom)
        T_map_odom_mat[0:3, 3] = trans_map_odom
        T_odom_map_mat = tf_transformations.inverse_matrix(T_map_odom_mat)

        # T_map_current_marker
        q_map_current_marker = [
            current_marker_pose_in_map.orientation.x,
            current_marker_pose_in_map.orientation.y,
            current_marker_pose_in_map.orientation.z,
            current_marker_pose_in_map.orientation.w
        ]
        trans_map_current_marker = [
            current_marker_pose_in_map.position.x,
            current_marker_pose_in_map.position.y,
            current_marker_pose_in_map.position.z
        ]
        T_map_current_marker_mat = tf_transformations.quaternion_matrix(q_map_current_marker)
        T_map_current_marker_mat[0:3, 3] = trans_map_current_marker

        # odom 기준 아르코 마커의 포즈 계산: T_odom_marker = T_odom_map * T_map_marker
        T_odom_current_marker_mat = np.dot(T_odom_map_mat, T_map_current_marker_mat)

        # odom 기준 'base_footprint'의 포즈 계산: T_odom_base = T_odom_marker * T_marker_base
        # self.T_marker_base_footprint_mat는 __init__에서 계산된 오프셋 변환 행렬
        T_odom_base_footprint_mat = np.dot(T_odom_current_marker_mat, self.T_marker_base_footprint_mat)

        # T_odom_base_footprint_mat로부터 translation과 quaternion 추출
        trans_odom_base = tf_transformations.translation_from_matrix(T_odom_base_footprint_mat)
        q_odom_base = tf_transformations.quaternion_from_matrix(T_odom_base_footprint_mat)

        # 3. Odometry 메시지 발행
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time_msg
        odom_msg.header.frame_id = self.robot_odom_frame_id # Odometry의 기준 프레임은 odom
        odom_msg.child_frame_id = self.robot_base_frame_id # Odometry가 나타내는 프레임 (로봇의 base)

        # Pose 정보 (odom 기준 base_footprint의 포즈)
        odom_msg.pose.pose.position.x = trans_odom_base[0]
        odom_msg.pose.pose.position.y = trans_odom_base[1]
        odom_msg.pose.pose.position.z = trans_odom_base[2] # 2D 환경이면 이 값은 거의 0이어야 함
        odom_msg.pose.pose.orientation.x = q_odom_base[0]
        odom_msg.pose.pose.orientation.y = q_odom_base[1]
        odom_msg.pose.pose.orientation.z = q_odom_base[2]
        odom_msg.pose.pose.orientation.w = q_odom_base[3]

        # Pose 공분산 설정
        odom_msg.pose.covariance = [0.0] * 36
        odom_msg.pose.covariance[0]  = self.pose_covariance_values[0] # x
        odom_msg.pose.covariance[7]  = self.pose_covariance_values[1] # y
        odom_msg.pose.covariance[14] = self.pose_covariance_values[2] # z
        odom_msg.pose.covariance[21] = self.pose_covariance_values[3] # roll
        odom_msg.pose.covariance[28] = self.pose_covariance_values[4] # pitch
        odom_msg.pose.covariance[35] = self.pose_covariance_values[5] # yaw


        # 속도 추정 (base_footprint의 odom 기준 속도)
        dt = (current_time_rclpy - self.previous_time).nanoseconds / 1e9
        linear_velocity_x_raw = 0.0
        linear_velocity_y_raw = 0.0
        angular_velocity_z_raw = 0.0

        if dt > 1e-6 and self.previous_marker_pose_in_map is not None: # 매우 작은 dt는 나누기 오류 방지
            # map 기준 위치 변화량 계산
            dx_map = current_marker_pose_in_map.position.x - self.previous_marker_pose_in_map.position.x
            dy_map = current_marker_pose_in_map.position.y - self.previous_marker_pose_in_map.position.y
            # dz_map = current_marker_pose_in_map.position.z - self.previous_marker_pose_in_map.position.z # 2D에서는 불필요

            # map 기준 회전 변화량 계산
            q_prev_map = [
                self.previous_marker_pose_in_map.orientation.x,
                self.previous_marker_pose_in_map.orientation.y,
                self.previous_marker_pose_in_map.orientation.z,
                self.previous_marker_pose_in_map.orientation.w
            ]
            q_curr_map = [
                current_marker_pose_in_map.orientation.x,
                current_marker_pose_in_map.orientation.y,
                current_marker_pose_in_map.orientation.z,
                current_marker_pose_in_map.orientation.w
            ]
            delta_q_map = tf_transformations.quaternion_multiply(q_curr_map, tf_transformations.quaternion_inverse(q_prev_map))
            _, _, dyaw_map = tf_transformations.euler_from_quaternion(delta_q_map)

            # map 기준 속도를 로봇의 현재 base_footprint 프레임 기준으로 변환해야 함
            # 로봇의 현재 yaw (map 기준)
            _, _, current_robot_yaw_map = tf_transformations.euler_from_quaternion(q_curr_map)

            # map 기준 속도를 base_footprint 기준 선속도로 변환
            # (dx_map, dy_map)을 현재 로봇의 yaw로 회전시켜 로봇 좌표계의 (vx, vy)를 얻음
            linear_velocity_x_raw = (dx_map * np.cos(-current_robot_yaw_map) - dy_map * np.sin(-current_robot_yaw_map)) / dt
            linear_velocity_y_raw = (dx_map * np.sin(-current_robot_yaw_map) + dy_map * np.cos(-current_robot_yaw_map)) / dt
            angular_velocity_z_raw = dyaw_map / dt # 각속도는 프레임 변환에 불변 (z축 기준)

        # LPF 적용
        filtered_vx = self.lpf_vx.filter(linear_velocity_x_raw)
        filtered_vy = self.lpf_vy.filter(linear_velocity_y_raw)
        filtered_wz = self.lpf_wz.filter(angular_velocity_z_raw)

        odom_msg.twist.twist.linear.x = filtered_vx
        odom_msg.twist.twist.linear.y = filtered_vy # 2D 로봇은 보통 y방향 선속도는 0에 가까움
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = filtered_wz

        # Twist 공분산 설정
        odom_msg.twist.covariance = [0.0] * 36
        odom_msg.twist.covariance[0]  = self.twist_covariance_values[0]  # vx
        odom_msg.twist.covariance[7]  = self.twist_covariance_values[1]  # vy
        odom_msg.twist.covariance[14] = self.twist_covariance_values[2]  # vz
        odom_msg.twist.covariance[21] = self.twist_covariance_values[3]  # wx
        odom_msg.twist.covariance[28] = self.twist_covariance_values[4]  # wy
        odom_msg.twist.covariance[35] = self.twist_covariance_values[5]  # wz

        self.odom_publisher.publish(odom_msg)

        # 4. TF 발행: odom -> base_footprint (Odometry 메시지의 pose와 일치해야 함)
        t_odom_to_base = TransformStamped()
        t_odom_to_base.header.stamp = current_time_msg
        t_odom_to_base.header.frame_id = self.robot_odom_frame_id
        t_odom_to_base.child_frame_id = self.robot_base_frame_id

        t_odom_to_base.transform.translation.x = odom_msg.pose.pose.position.x
        t_odom_to_base.transform.translation.y = odom_msg.pose.pose.position.y
        t_odom_to_base.transform.translation.z = odom_msg.pose.pose.position.z
        t_odom_to_base.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t_odom_to_base)

        # 이전 상태 업데이트
        self.previous_marker_pose_in_map = current_marker_pose_in_map
        self.previous_time = current_time_rclpy


def main(args=None):
    rclpy.init(args=args)
    node = ArucoOdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ArucoOdomTfBroadcaster shutting down.')
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

