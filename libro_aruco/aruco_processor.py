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
                    pass
                except Exception as e:
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

    @staticmethod
    def normalize_quaternion(quat):
        """쿼터니언 정규화"""
        norm = np.linalg.norm(quat)
        if norm < 1e-8:  # 매우 작은 값인 경우 단위 쿼터니언 반환
            return np.array([0.0, 0.0, 0.0, 1.0])
        return quat / norm

    @staticmethod
    def ensure_quaternion_continuity(q_current, q_previous):
        """쿼터니언 부호 일관성 보장 - 연속성을 위해 가장 가까운 표현 선택"""
        if q_previous is None:
            return q_current

        # 두 쿼터니언의 내적 계산
        dot_product = np.dot(q_current, q_previous)

        # 내적이 음수이면 q_current의 부호를 바꿔서 더 가까운 경로 선택
        if dot_product < 0.0:
            return -q_current
        return q_current

    def get_pose_data(self, corners, ids, rvecs, tvecs):
        """
        ArUco 마커의 포즈 정보를 담은 리스트를 반환합니다.
        위치에 대한 LPF와 회전(쿼터니언)에 대한 SLERP 기반 LPF가 적용됩니다.
        쿼터니언 정규화와 부호 일관성이 보장됩니다.
        """
        aruco_data_list = []

        if ids is None or corners is None or rvecs is None or tvecs is None:
            return aruco_data_list

        # 필터 파라미터
        slerp_alpha = 0.1  # SLERP 필터링 가중치 (0 < alpha <= 1)
        position_lpf_alpha = 0.1  # 위치 LPF 가중치

        for i, marker_id_arr in enumerate(ids):
            marker_id = int(marker_id_arr[0])
            current_corners = corners[i]
            current_rvec = rvecs[i]  # (3,) 형태
            current_tvec = tvecs[i]  # (3,) 형태

            # 1. 현재 측정된 회전 (쿼터니언)
            current_rotation_obj = self.rvec_to_scipy_rotation(current_rvec)
            current_quat_array = current_rotation_obj.as_quat()  # [x, y, z, w]

            # 쿼터니언 정규화
            current_quat_array = self.normalize_quaternion(current_quat_array)
            current_quat_scipy = Rotation.from_quat(current_quat_array)

            # 2. 부호 일관성 보장
            prev_quat_array = None
            if marker_id in self.prev_raw_quaternions:
                prev_quat_array = self.prev_raw_quaternions[marker_id].as_quat()

            # 연속성을 위한 부호 조정
            current_quat_array = self.ensure_quaternion_continuity(current_quat_array, prev_quat_array)
            current_quat_scipy = Rotation.from_quat(current_quat_array)

            # 3. SLERP 필터링 적용 (회전)
            if marker_id not in self.filtered_quaternions:
                # 첫 프레임: 필터링 없이 현재 값을 사용
                filtered_rotation_obj = current_quat_scipy
            else:
                prev_filtered_rotation_obj = self.filtered_quaternions[marker_id]

                # SLERP을 위한 최적화된 보간
                try:
                    # 두 회전 사이의 각도 차이 확인
                    relative_rotation = prev_filtered_rotation_obj.inv() * current_quat_scipy
                    angle_diff = relative_rotation.magnitude()

                    # 각도 차이가 너무 크면 (180도 이상) 부호 조정
                    if angle_diff > np.pi:
                        current_quat_array = -current_quat_array
                        current_quat_scipy = Rotation.from_quat(current_quat_array)

                    # SLERP 보간 수행
                    key_rots = Rotation.concatenate([prev_filtered_rotation_obj, current_quat_scipy])
                    key_times = [0, 1]
                    slerp_interpolator = Slerp(key_times, key_rots)
                    filtered_rotation_obj = slerp_interpolator(slerp_alpha)

                    # 결과 쿼터니언 정규화
                    filtered_quat_array = filtered_rotation_obj.as_quat()
                    filtered_quat_array = self.normalize_quaternion(filtered_quat_array)
                    filtered_rotation_obj = Rotation.from_quat(filtered_quat_array)

                except Exception as e:
                    # SLERP 실패 시 단순 LPF 적용
                    prev_quat = self.filtered_quaternions[marker_id].as_quat()
                    filtered_quat_array = (1 - slerp_alpha) * prev_quat + slerp_alpha * current_quat_array
                    filtered_quat_array = self.normalize_quaternion(filtered_quat_array)
                    filtered_rotation_obj = Rotation.from_quat(filtered_quat_array)

            # 필터링된 회전 저장
            self.filtered_quaternions[marker_id] = filtered_rotation_obj
            self.prev_raw_quaternions[marker_id] = current_quat_scipy

            # 4. 위치(tvec)에 대한 LPF 적용
            if marker_id not in self.filtered_positions:
                # 첫 프레임: 필터링 없이 현재 값을 사용
                filtered_tvec = current_tvec
            else:
                prev_filtered_tvec = self.filtered_positions[marker_id]
                # LPF 공식: y_k = (1 - alpha) * y_k-1 + alpha * x_k
                filtered_tvec = (1 - position_lpf_alpha) * prev_filtered_tvec + position_lpf_alpha * current_tvec

            # 결과 저장
            self.filtered_positions[marker_id] = filtered_tvec

            # 5. 최종 결과 생성
            final_quat_array = filtered_rotation_obj.as_quat()  # [x, y, z, w]
            final_quat_array = self.normalize_quaternion(final_quat_array)  # 최종 정규화
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
