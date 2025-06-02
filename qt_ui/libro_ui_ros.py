import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Float32
from PyQt5.QtCore import QThread, pyqtSignal
from sensor_msgs.msg import BatteryState
import json
import requests


def parse_ros_json_data(data_str):
    """
    ROS String 메시지에서 'data: ...' 형식의 문자열을 받아
    JSON 부분만 추출해 파이썬 딕셔너리로 반환합니다.
    """
    prefix = "data: "
    if data_str.startswith(prefix):
        json_str = data_str[len(prefix):]
        if json_str.startswith("'") and json_str.endswith("'"):
            json_str = json_str[1:-1]
        return json.loads(json_str)
    else:
        # 이미 JSON이라면 바로 파싱
        return json.loads(data_str)

class LibroPoseSub(Node):
    def __init__(self,
                 libro1_pose,
                 libro2_pose,
                 libro3_pose,
                 libro_state,
                 libro1_erorr,
                 libro2_erorr,
                 libro3_erorr,
                 libro1_baterry,
                 libro2_baterry,
                 libro3_baterry,
                 libro1_state,
                 libro2_state,
                 libro3_state,
                 log_list_signal
):
        super().__init__('libro_pose_subscriber') # 노드 이름이 libro_pose_subscriber
        self.libro1_pose_signal = libro1_pose
        self.libro2_pose_signal = libro2_pose
        self.libro3_pose_signal = libro3_pose
        self.libro_state_signal = libro_state
        self.libro1_error_signal = libro1_erorr
        self.libro2_error_signal = libro2_erorr
        self.libro3_error_signal = libro3_erorr
        self.libro1_battery_signal = libro1_baterry
        self.libro2_battery_signal = libro2_baterry
        self.libro3_battery_signal = libro3_baterry
        self.libro1_state_signal = libro1_state
        self.libro2_state_signal = libro2_state
        self.libro3_state_signal = libro3_state
        self.log_list_signal = log_list_signal
        
        # 각각의 libro들의 pose 토픽을 받을 서브스크라이브 생성, 데이터를 받으면 콜백 함수 실행.
        self.libro1_pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco1/map/pose',
            self.libro1_pose_callback,
            10
        )

        self.libro2_pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco2/map/pose',
            self.libro2_pose_callback,
            10
        )

        self.libro3_pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco3/map/pose',
            self.libro3_pose_callback,
            10
        )
###########################################리브로 스테이트
        self.libro_state_sub = self.create_subscription(
            String,
            '/libro_robot_states',
            self.libro_state_callback,
            10
        )

##리브로 에러
        self.libro1_erorr_sub = self.create_subscription(
            String,
            '/libro1/error',
            self.libro1_erorr_callback,
            10
        )
        self.libro2_erorr_sub = self.create_subscription(
            String,
            '/libro2/error',
            self.libro2_erorr_callback,
            10
        )
        self.libro3_erorr_sub = self.create_subscription(
            String,
            '/libro3/error',
            self.libro3_erorr_callback,
            10
        )

##리브로 베터리
        self.libro1_battery_sub = self.create_subscription(
            Float32,
            '/libro1/pinky_battery_present',
            self.libro1_battery_callback,
            10
        )
        self.libro2_battery_sub = self.create_subscription(
            Float32,
            '/libro2/pinky_battery_present',
            self.libro2_battery_callback,
            10
        )

        self.libro3_battery_sub = self.create_subscription(
            Float32,
            '/libro3/pinky_battery_present',
            self.libro3_battery_callback,
            10
        )

##리브로 상세 스테이트
        self.libro1_state_sub = self.create_subscription(
            String,
            '/libro1/state',
            self.libro1_state_callback,
            10
        )
        self.libro2_state_sub = self.create_subscription(
            String,
            '/libro2/state',
            self.libro2_state_callback,
            10
        )
        self.libro3_state_sub = self.create_subscription(
            String,
            '/libro3/state',
            self.libro3_state_callback,
            10
        )

        self.get_and_emit_robot_logs()

    # 각각의 libro들의 pose 메시지를 수신했을 때 실행되는 콜백 함수.
    def libro1_pose_callback(self, msg):
        x = round(msg.pose.position.x, 2)
        y = round(msg.pose.position.y, 2)
        self.libro1_pose_signal.emit(1, x, y) # signal을 발생할 때 로봇 id도 같이 발생
        # libro_pose_signal이라는 시그널을 emit(발생시킨다.)
        # libro1_pose_signal은 방송 채널, 즉 libro1_pose_received 시그널을 발생.

    def libro2_pose_callback(self, msg):
        x = round(msg.pose.position.x, 2)
        y = round(msg.pose.position.y, 2)
        self.libro2_pose_signal.emit(2, x, y)


    def libro3_pose_callback(self, msg):
        x = round(msg.pose.position.x, 2)
        y = round(msg.pose.position.y, 2)
        self.libro3_pose_signal.emit(3, x, y)


###########################################
    def libro_state_callback(self, msg):
        #ex)msg=data: '{"libro1": "pickup", "libro2": "pickup"}'
        parsed_dict = json.loads(msg.data)
        required_keys = ['libro1', 'libro2', 'libro3'] #키 리스트

        for key in required_keys:
            if key not in parsed_dict:# 키 리스트에 없으면 none
                parsed_dict[key] = 'none'

        result_list = list(parsed_dict.items())
        self.libro_state_signal.emit(result_list)

    def libro1_erorr_callback(self, msg):
        #msg = data: '{"status": "0x42", "message": "책 위치 인식 실패: TCP 통신 오류 - [Errno 111] Connection refused"}'
        parsed_dict = json.loads(msg.data)
        message_1 = str(parsed_dict["message"])
        self.libro1_error_signal.emit(message_1)

    def libro2_erorr_callback(self, msg):
        parsed_dict = json.loads(msg.data)
        message_2 = str(parsed_dict["message"])
        self.libro2_error_signal.emit(message_2)

    def libro3_erorr_callback(self, msg):
        parsed_dict = json.loads(msg.data)
        message_3 = str(parsed_dict["message"])
        self.libro3_error_signal.emit(message_3)

    def libro1_battery_callback(self, msg):
        libro1_battery_info = int(msg.data)
        self.libro1_battery_signal.emit(libro1_battery_info)
        
    def libro2_battery_callback(self, msg):
        libro2_battery_info = int(msg.data)
        self.libro2_battery_signal.emit(libro2_battery_info)

    def libro3_battery_callback(self, msg):
        libro3_battery_info = int(msg.data)
        self.libro3_battery_signal.emit(libro3_battery_info)

    def libro1_state_callback(self, msg):
        try:
            parsed_dict = parse_ros_json_data(msg.data)
            status1 = str(parsed_dict["status"])
            self.libro1_state_signal.emit(status1)
        except Exception as e:
            print(f"[libro1_state_callback] 파싱 오류: {e}, 원본 메시지: {msg.data}")

    def libro2_state_callback(self, msg):
        try:
            parsed_dict = parse_ros_json_data(msg.data)
            status2 = str(parsed_dict["status"])
            self.libro2_state_signal.emit(status2)
        except Exception as e:
            print(f"[libro2_state_callback] 파싱 오류: {e}, 원본 메시지: {msg.data}")

    def libro3_state_callback(self, msg):
        try:
            parsed_dict = parse_ros_json_data(msg.data)
            status3 = str(parsed_dict["status"])
            self.libro3_state_signal.emit(status3)
        except Exception as e:
            print(f"[libro3_state_callback] 파싱 오류: {e}, 원본 메시지: {msg.data}")

    def get_and_emit_robot_logs(self):
        url = f"http://192.168.0.138:8000/robot-log"
        response = requests.get(url)
        if response.status_code == 200:
            data = response.json()
            headers = ["timestamp", "robot_id", "order_number", "robot_log", "success"]
            log_list = []
            for log in data:
                row = [log.get(h) for h in headers]
                log_list.append(row)
            # 시그널 emit
            self.log_list_signal.emit(log_list)

class RosTopicInfo(QThread):
    # 각 로봇의 pose 정보를 PyQT로 전달할 시그널 선언.
    # 이 시그널들은  정보를 PyQT 메인 스레드로 전달할 수 있음.
    # 콜백 함수에 libro_pose_signal과 같은 내용을 담은 변수라고 보면 됨.
    # 다만, 역할이 다르다. ROS 노드 클래스 안에서 signal은 방송을 하는 놈이고, QThread 안에 있는 놈은 방송을 받는 놈.
    # main_ui 파일에서 오브젝트들과 연결될 녀석은 QTread 안에 있는 received가 된다.
    libro1_pose_received = pyqtSignal(int, float, float) # robot_id를 int, x, y 좌표를 float 형태로 보냄.
    libro2_pose_received = pyqtSignal(int, float, float)
    libro3_pose_received = pyqtSignal(int, float, float)
###########################################
    libro_state_received = pyqtSignal(list)
    libro1_erorr_received = pyqtSignal(str)
    libro2_erorr_received = pyqtSignal(str)
    libro3_erorr_received = pyqtSignal(str)
    libro1_battery_received = pyqtSignal(int)
    libro2_battery_received = pyqtSignal(int)
    libro3_battery_received = pyqtSignal(int)
    libro1_state_received = pyqtSignal(str)
    libro2_state_received = pyqtSignal(str)
    libro3_state_received = pyqtSignal(str)
    log_list_received = pyqtSignal(list)
    def __init__(self):
        super().__init__()

    def run(self):
        # rclpy.init()은 여러 번 호출되면 안되니까, 예외처리 추가.
        try:
            rclpy.init()
            
        except RuntimeError:
    
            pass
        
        # 노드를 생성하여, 시그널 객체들을 전달
        self.node = LibroPoseSub(
            self.libro1_pose_received,
            self.libro2_pose_received,
            self.libro3_pose_received,
            self.libro_state_received,
            self.libro1_erorr_received,
            self.libro2_erorr_received,
            self.libro3_erorr_received,
            self.libro1_battery_received,
            self.libro2_battery_received,
            self.libro3_battery_received,
            self.libro1_state_received,
            self.libro2_state_received,
            self.libro3_state_received,
            self.log_list_received
        )
        
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()