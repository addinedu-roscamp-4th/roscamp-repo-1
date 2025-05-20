import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


class ArucoProcessor:
    def __init__(self, camera_matrix, dist_coeffs, marker_length, aruco_dict_type=cv2.aruco.DICT_6X6_250):
        """ArUco 검출기 초기화"""
        self.camera_matrix = np.array(camera_matrix, dtype=np.float32)
        self.dist_coeffs = np.array(dist_coeffs, dtype=np.float32)
        self.marker_length = float(marker_length)

        # 필터링을 위한 내부 상태 변수
        self.filtered_quaternions = {}  # 마커 ID별 필터링된 쿼터니언 저장 (scipy.spatial.transform.Rotation 객체)
        self.filtered_positions = {}  # 마커 ID별 필터링된 위치 저장 (numpy array [x,y,z])
        self.prev_raw_quaternions = {}  # SLERP shortest path 결정을 위한 이전 Raw 쿼터니언

        try:
            # OpenCV 4.7.0 이상
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        except AttributeError:
            # 이전 OpenCV 버전
            self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
            self.aruco_params = cv2.aruco.DetectorParameters_create()

    def detect_markers(self, image):
        """이미지에서 ArUco 마커 검출"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        try:
            # OpenCV 4.7.0 이상
            corners, ids, rejected_img_points = self.detector.detectMarkers(gray)
        except AttributeError:
            # 이전 OpenCV 버전
            corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)

        rvecs, tvecs = None, None

        if ids is not None and len(ids) > 0:
            rvecs_list = []
            tvecs_list = []
            for i in range(len(ids)):
                try:
                    current_corners = corners[i]
                    # estimatePoseSingleMarkers는 rvec, tvec을 3x1 또는 1x3 형태로 반환할 수 있음
                    _rvec, _tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        current_corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
                    rvecs_list.append(_rvec.flatten())  # 1D 배열로 만듦
                    tvecs_list.append(_tvec.flatten())  # 1D 배열로 만듦
                except cv2.error as e:
                    # self.get_logger().warn(f"estimatePoseSingleMarkers OpenCV 에러 (마커 ID {ids[i][0]}): {e}")
                    pass
                except Exception as e:
                    # self.get_logger().warn(f"estimatePoseSingleMarkers 일반 에러 (마커 ID {ids[i][0]}): {e}")
                    pass

            if rvecs_list:  # 성공적으로 처리된 마커가 하나라도 있다면
                rvecs = np.array(rvecs_list, dtype=np.float32)  # (N, 3) 형태
                tvecs = np.array(tvecs_list, dtype=np.float32)  # (N, 3) 형태

        return corners, ids, rejected_img_points, rvecs, tvecs

    @staticmethod
    def rvec_to_scipy_rotation(rvec):
        """회전 벡터(rvec)를 scipy.spatial.transform.Rotation 객체로 변환"""
        # rvec의 크기가 회전 각도(라디안), 방향이 회전 축
        return Rotation.from_rotvec(rvec.flatten())

    def get_pose_data(self, corners, ids, rvecs, tvecs):
        """
        ArUco 마커의 포즈 정보를 담은 리스트를 반환합니다.
        위치에 대한 LPF와 회전(쿼터니언)에 대한 SLERP 기반 LPF가 적용됩니다.
        """
        aruco_data_list = []
        if ids is None or corners is None or rvecs is None or tvecs is None:
            return aruco_data_list

        # 필터 파라미터 (1차 LPF와 유사하게 동작하도록 SLERP 가중치 조절)
        # alpha 값 (0~1 사이). alpha가 클수록 새 측정값에 가중. 작을수록 필터링 강해짐.
        # LPF의 tau, Ts 관계: alpha = Ts / (tau + Ts)
        # 예: tau = 0.1, Ts = 0.033 (30FPS) -> alpha = 0.033 / (0.1 + 0.033) approx 0.248
        slerp_alpha = 0.25  # SLERP 필터링 가중치 (0 < alpha <= 1)
        position_lpf_alpha = 0.25  # 위치 LPF 가중치

        for i, marker_id_arr in enumerate(ids):  # ids는 (N, 1) 형태일 수 있음
            marker_id = int(marker_id_arr[0])
            current_corners = corners[i]
            current_rvec = rvecs[i]  # (3,) 형태
            current_tvec = tvecs[i]  # (3,) 형태

            # 1. 현재 측정된 회전 (쿼터니언)
            current_rotation_obj = self.rvec_to_scipy_rotation(current_rvec)
            current_quat_scipy = current_rotation_obj  # Rotation 객체 자체를 사용

            # 2. SLERP 필터링 적용 (회전)
            if marker_id not in self.filtered_quaternions:
                # 첫 프레임: 필터링 없이 현재 값을 사용
                filtered_rotation_obj = current_quat_scipy
            else:
                prev_filtered_rotation_obj = self.filtered_quaternions[marker_id]
                prev_raw_rotation_obj = self.prev_raw_quaternions.get(marker_id, current_quat_scipy)

                # SLERP은 두 쿼터니언 사이의 "최단 경로"를 보간합니다.
                # 두 쿼터니언 q1, q2가 있을 때, q2와 -q2는 동일한 회전을 나타낼 수 있습니다.
                # prev_filtered_rotation_obj.dot(current_quat_scipy) < 0 이면 반대 경로이므로
                # current_quat_scipy 대신 -current_quat_scipy (또는 그 역)를 사용해야 부드러운 보간이 됩니다.
                # Scipy의 Slerp는 내부적으로 이를 처리할 수 있지만, 명시적으로 확인하는 것이 안전할 수 있습니다.
                # 여기서는 Scipy의 Slerp가 keyframe_times=[0,1]로 주어지면 적절히 처리한다고 가정합니다.

                # 두 Rotation 객체 사이의 Slerp
                # Slerp([t_prev, t_curr], Rotation.concatenate([prev_filtered, current_raw]))(t_filtered)
                # 더 간단히는, 이전 필터값과 현재 측정값 사이를 slerp_alpha 비율로 보간
                key_rots = Rotation.concatenate([prev_filtered_rotation_obj, current_quat_scipy])
                key_times = [0, 1]  # 이전 값 시간 0, 현재 값 시간 1

                # 현재 측정된 쿼터니언이 이전 필터된 쿼터니언과 너무 다르면 (반대 방향 등)
                # current_quat_scipy의 부호를 조정하여 최단 경로 보간을 유도
                if prev_filtered_rotation_obj.as_quat() @ current_quat_scipy.as_quat() < 0:
                    # Scipy Rotation 객체는 내부적으로 쿼터니언을 다루므로,
                    # quat * -1 로 새로운 Rotation 객체를 만들어야 함
                    negated_quat_array = -current_quat_scipy.as_quat()
                    current_quat_for_slerp = Rotation.from_quat(negated_quat_array)
                    key_rots = Rotation.concatenate([prev_filtered_rotation_obj, current_quat_for_slerp])
                else:
                    current_quat_for_slerp = current_quat_scipy  # 원래값 사용

                slerp_interpolator = Slerp(key_times, key_rots)
                filtered_rotation_obj = slerp_interpolator(slerp_alpha)  # 0과 1 사이의 slerp_alpha 지점에서 보간

            self.filtered_quaternions[marker_id] = filtered_rotation_obj
            self.prev_raw_quaternions[marker_id] = current_quat_scipy  # 다음 프레임의 shortest path 결정을 위해 저장

            # 3. 위치(tvec)에 대한 LPF 적용
            if marker_id not in self.filtered_positions:
                # 첫 프레임: 필터링 없이 현재 값을 사용
                filtered_tvec = current_tvec
            else:
                prev_filtered_tvec = self.filtered_positions[marker_id]
                # LPF 공식: y_k = (1 - alpha) * y_k-1 + alpha * x_k
                filtered_tvec = (1 - position_lpf_alpha) * prev_filtered_tvec + position_lpf_alpha * current_tvec

            self.filtered_positions[marker_id] = filtered_tvec

            # 결과 저장
            final_quat_array = filtered_rotation_obj.as_quat()  # [x, y, z, w]
            final_euler_angles_deg = filtered_rotation_obj.as_euler('zyx', degrees=True)

            aruco_data_list.append({
                'id': marker_id,
                'corners': current_corners,
                'rvec': filtered_rotation_obj.as_rotvec(),  # 필터링된 회전을 rvec으로 표현
                'tvec': filtered_tvec,  # 필터링된 위치
                'quaternion': final_quat_array,  # 필터링된 쿼터니언 [x,y,z,w]
                'euler_angles': final_euler_angles_deg,  # 필터링된 오일러 각 (참고용)
                'rotation_z': final_euler_angles_deg[0]  # 필터링된 Z축 회전각 (참고용)
            })

        return aruco_data_list

    def draw_markers(self, image, corners, ids, rvecs=None, tvecs=None):
        """이미지에 마커 및 축 그리기. rvecs, tvecs는 필터링 적용 전/후 값 모두 가능."""
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)

        if rvecs is not None and tvecs is not None and self.camera_matrix is not None and self.dist_coeffs is not None:
            for i in range(len(ids)):
                # rvecs[i]가 이미 (3,) 형태여야 함
                rvec_to_draw = rvecs[i].reshape(3, 1)  # drawFrameAxes는 (3,1) 또는 (1,3) 형태를 요구할 수 있음
                tvec_to_draw = tvecs[i].reshape(3, 1)
                try:
                    # OpenCV 4.7.0+
                    cv2.drawFrameAxes(image, self.camera_matrix, self.dist_coeffs,
                                      rvec_to_draw, tvec_to_draw, self.marker_length * 0.5)
                except AttributeError:  # 이전 OpenCV
                    # cv2.aruco.drawAxis는 rvec, tvec을 1D 배열로 받을 수 있음
                    cv2.aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs,
                                       rvecs[i], tvecs[i], self.marker_length * 0.5)
        return image