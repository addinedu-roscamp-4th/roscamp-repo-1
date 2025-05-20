#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile
import numpy as np

class ArucoOdomTfBroadcaster(Node):

    def __init__(self):
        super().__init__('aruco_odom_tf_broadcaster')

        self.declare_parameter('marker_id', 0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
        self.map_frame_id = self.get_parameter('map_frame').get_parameter_value().string_value
        odom_frame_id = self.get_parameter('odom_frame').get_parameter_value().string_value
        base_frame_id = self.get_parameter('base_frame').get_parameter_value().string_value

        current_namespace = self.get_namespace()
        if current_namespace == '/':  # 네임스페이스가 없는 경우 (루트 네임스페이스)
            current_namespace = ""  # 빈 문자열로 만들어 이어붙이기 용이하게

        if not odom_frame_id.startswith('/') and current_namespace:
            self.robot_odom_frame_id = f"{current_namespace}/{odom_frame_id}"
        else:
            self.robot_odom_frame_id = odom_frame_id.lstrip('/')

        if not base_frame_id.startswith('/') and current_namespace:
            self.robot_base_frame_id = f"{current_namespace}/{base_frame_id}"
        else:
            self.robot_base_frame_id = base_frame_id.lstrip('/')

        self.tf_broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(depth=1)

        self.map_pose_sub = self.create_subscription(
            PoseStamped,
            f"/aruco{self.marker_id}/map/pose", # 맵 기준 특정 마커 포즈 구독
            self.map_pose_callback,
            qos
        )

        # Odometry 메시지 발행을 위한 퍼블리셔 추가
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        self.get_logger().info(f"Odometry: {self.get_namespace()}/odom")

    def map_pose_callback(self, msg: PoseStamped):
        current_time = self.get_clock().now().to_msg() # 일관된 시간 사용

        # 1. TF 발행: map -> odom_frame_id (arucoX/odom)
        #    이 TF는 ArUco 마커의 odom 프레임이 맵에 대해 어떤 포즈를 갖는지를 나타냅니다.
        #    Odometry 메시지와 일관성을 위해 TF의 회전도 roll/pitch를 0으로 만들 수 있습니다.
        #    그러나 여기서는 원본 PoseStamped의 회전을 그대로 사용하여 TF를 발행하고,
        #    Odometry 메시지만 roll/pitch를 0으로 수정합니다.
        #    만약 TF 자체의 좌표축도 맵과 수평이 되길 원한다면, 아래 Odometry 회전 수정 로직을
        #    t_map_to_odom.transform.rotation 에도 동일하게 적용해야 합니다.

        if msg.header.frame_id != self.map_frame_id:
            self.get_logger().warn_once(
                f"수신된 맵 포즈의 frame_id ({msg.header.frame_id})가 설정된 map_frame ({self.map_frame_id})과 다릅니다. "
                f"map -> {self.robot_odom_frame_id} TF 및 Odometry 발행에 영향을 줄 수 있습니다."
            )
            # 경우에 따라 여기서 처리를 중단할 수 있습니다.
            # return

        t_map_to_odom = TransformStamped()
        t_map_to_odom.header.stamp = current_time
        t_map_to_odom.header.frame_id = self.map_frame_id    # 부모 프레임: map
        t_map_to_odom.child_frame_id = self.robot_odom_frame_id    # 자식 프레임: arucoX/odom

        # 위치는 입력된 PoseStamped의 위치를 사용
        t_map_to_odom.transform.translation.x = msg.pose.position.x
        t_map_to_odom.transform.translation.y = msg.pose.position.y
        t_map_to_odom.transform.translation.z = msg.pose.position.z # z 위치는 원본 유지

        # 회전은 일단 입력된 PoseStamped의 회전을 사용 (위 주석 참고)
        t_map_to_odom.transform.rotation = msg.pose.orientation
        self.tf_broadcaster.sendTransform(t_map_to_odom)

        # 2. Odometry 메시지 발행: map 기준 odom_frame_id (arucoX/odom)의 위치/자세
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = self.map_frame_id       # Odometry의 기준 프레임은 map
        odom_msg.child_frame_id = self.robot_odom_frame_id      # Odometry가 나타내는 프레임 (예: aruco0/odom)

        # Pose 정보 설정
        # 위치 정보는 그대로 사용
        odom_msg.pose.pose.position = msg.pose.position
        # 만약 z 위치를 강제로 0으로 만들고 싶다면:
        # odom_msg.pose.pose.position.z = 0.0

        # 회전 정보 수정 (roll, pitch를 0으로)
        original_quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])

        # 쿼터니언 -> 오일러 각 (ZYX 순서로 yaw, pitch, roll)
        try:
            rotation_obj = R.from_quat(original_quat)
            euler_angles_zyx = rotation_obj.as_euler('zyx', degrees=False) # [yaw, pitch, roll] in radians

            # roll과 pitch를 0으로 설정 (맵 좌표계와 수평 유지)
            modified_euler_angles_zyx = np.array([euler_angles_zyx[0], 0.0, 0.0]) # [yaw, 0 (pitch), 0 (roll)]

            # 수정된 오일러 각 -> 쿼터니언
            modified_rotation_obj = R.from_euler('zyx', modified_euler_angles_zyx, degrees=False)
            modified_quat_array = modified_rotation_obj.as_quat() # [x, y, z, w]

            odom_msg.pose.pose.orientation.x = float(modified_quat_array[0])
            odom_msg.pose.pose.orientation.y = float(modified_quat_array[1])
            odom_msg.pose.pose.orientation.z = float(modified_quat_array[2])
            odom_msg.pose.pose.orientation.w = float(modified_quat_array[3])
        except Exception as e:
            self.get_logger().error(f"회전 변환 중 오류 발생: {e}. 원본 회전을 사용합니다.")
            odom_msg.pose.pose.orientation = msg.pose.orientation


        # Pose 공분산 설정: x, y, yaw에 대해서는 작은 분산값 (높은 신뢰도)
        # z, roll, pitch에 대해서는 매우 큰 분산값 (낮은 신뢰도, 사실상 무시)
        # 대각 성분 순서: x, y, z, roll, pitch, yaw
        x_variance = 0.1  # x 위치 분산 (예시 값, 실제 시스템에 맞게 튜닝 필요)
        y_variance = 0.1  # y 위치 분산
        z_variance = 1e9   # z 위치 분산 (매우 크게 설정하여 무시)
        roll_variance = 1e9 # roll 회전 분산 (매우 크게 설정하여 무시)
        pitch_variance = 1e9# pitch 회전 분산 (매우 크게 설정하여 무시)
        yaw_variance = 0.05 # yaw 회전 분산

        odom_msg.pose.covariance = [
            x_variance, 0.0, 0.0, 0.0, 0.0, 0.0,        # x
            0.0, y_variance, 0.0, 0.0, 0.0, 0.0,        # y
            0.0, 0.0, z_variance, 0.0, 0.0, 0.0,        # z
            0.0, 0.0, 0.0, roll_variance, 0.0, 0.0,     # roll
            0.0, 0.0, 0.0, 0.0, pitch_variance, 0.0,    # pitch
            0.0, 0.0, 0.0, 0.0, 0.0, yaw_variance       # yaw
        ]

        # Twist 정보 (선속도, 각속도) 설정
        # 현재 Pose 정보만으로는 속도를 알 수 없으므로 0으로 설정하거나 이전 값과 비교하여 추정
        # 여기서는 간단히 0으로 설정하고, 불확실성이 매우 높음을 표시
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # Twist 공분산 설정 (속도 정보가 없으므로 매우 높은 불확실성을 나타냄)
        # 대각 성분에만 -1 값을 넣어 "알 수 없음"을 표현 (몇몇 시스템에서 사용되는 방식)
        # 또는 매우 큰 값을 사용
        odom_msg.twist.covariance = [-1.0 if i == j else 0.0 for i in range(6) for j in range(6)]

        self.odom_publisher.publish(odom_msg)
        # self.get_logger().debug(f"Odometry 발행: {odom_msg.header.frame_id} -> {odom_msg.child_frame_id}")

        # 3. 정적 TF 발행: odom_frame_id (arucoX/odom) -> base_footprint_frame_id
        # 이 TF는 odom_frame_id에 대해 base_footprint가 정적이라고 가정합니다.
        # 즉, ArUco 마커의 odom 프레임과 로봇의 base_footprint 프레임이 동일하다고 가정합니다.
        # 실제 시스템에서는 로봇의 base_link 등 다른 프레임일 수 있으며,
        # 마커와 로봇 중심 간의 오프셋이 있다면 해당 값을 적용해야 합니다.
        # 여기서는 간단히 두 프레임이 일치한다고 가정 (변환 없음: 위치 (0,0,0), 회전 (0,0,0,1))
        t_odom_to_base = TransformStamped()
        t_odom_to_base.header.stamp = current_time
        t_odom_to_base.header.frame_id = self.robot_odom_frame_id
        t_odom_to_base.child_frame_id = self.robot_base_frame_id

        t_odom_to_base.transform.translation.x = 0.0
        t_odom_to_base.transform.translation.y = 0.0
        t_odom_to_base.transform.translation.z = 0.0
        t_odom_to_base.transform.rotation.x = 0.0
        t_odom_to_base.transform.rotation.y = 0.0
        t_odom_to_base.transform.rotation.z = 0.0
        t_odom_to_base.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t_odom_to_base)


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

