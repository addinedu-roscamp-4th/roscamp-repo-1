import numpy as np
import rclpy

class PredictiveFilter:
    def __init__(self, alpha=0.15, prediction_weight=0.1):  # 더 강한 필터링
        self.alpha = alpha
        self.prediction_weight = prediction_weight
        self.previous_value = None
        self.previous_velocity = None
        self.previous_time = None
        self.velocity_history = []
        self.max_history = 10  # 이력 증가
        self.noise_threshold = 0.002  # 노이즈 임계값 추가

    def filter(self, current_value, current_time):
        if self.previous_value is None:
            self.previous_value = current_value
            self.previous_time = current_time
            return current_value

        dt = (current_time - self.previous_time).nanoseconds / 1e9

        if dt <= 0:
            return self.previous_value

        if dt > 2.0:
            self.reset()
            self.previous_value = current_value
            self.previous_time = current_time
            return current_value

        # 노이즈 감지 및 제거
        movement = np.linalg.norm(current_value - self.previous_value)
        if movement < self.noise_threshold:
            # 매우 작은 움직임은 노이즈로 간주하고 이전 값 유지
            return self.previous_value

        velocity = (current_value - self.previous_value) / dt

        self.velocity_history.append(velocity)
        if len(self.velocity_history) > self.max_history:
            self.velocity_history.pop(0)

        if self.previous_velocity is not None and len(self.velocity_history) >= 3:
            # 중간값 필터링 추가
            velocities = np.array(self.velocity_history)
            median_velocity = np.median(velocities, axis=0)
            predicted_value = self.previous_value + median_velocity * dt

            mixed_value = (1 - self.prediction_weight) * current_value + \
                         self.prediction_weight * predicted_value
        else:
            mixed_value = current_value

        # 더 강한 LPF 적용
        filtered_value = self.alpha * mixed_value + \
                        (1 - self.alpha) * self.previous_value

        self.previous_value = filtered_value
        self.previous_velocity = velocity
        self.previous_time = current_time

        return filtered_value

    def reset(self):
        """필터 상태 초기화"""
        self.previous_value = None
        self.previous_velocity = None
        self.previous_time = None
        self.velocity_history.clear()
