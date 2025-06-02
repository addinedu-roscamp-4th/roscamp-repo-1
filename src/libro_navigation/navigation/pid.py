"""

state 값을 입력받아 PID 제어기를 통해 보정 출력을 계산하는 노드

"""
import time 

class PID:
    def __init__(self):
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.max_output = 3.14  # default = float('inf')
        self.min_output = -3.14 # default = float('inf')
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def update(self, error):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt == 0.0:
            dt = 1e-6  # 너무 짧은 시간 보호

        # PID 계산
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.P * error + self.I * self.integral + self.D * derivative

        # 출력 제한
        output = max(min(output, self.max_output), self.min_output)

        self.prev_error = error
        return output


