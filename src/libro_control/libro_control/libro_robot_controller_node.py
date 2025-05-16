import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32MultiArray, Bool
from builtin_interfaces.msg import Time
from libro_control_msgs.msg import BookPickUp, LibroRobotLog, LibroRobotState #,Navigate        # 책 픽업 미션 , 길 안내 미션, (구독용) / 로봇 상태 로그 (발행용)
from libro_control_msgs.msg import PinkyRequest, PinkyResponse                  # 주행로봇 목적지 이동 요청 및 응답
from libro_control_msgs.msg import JetcobotPickRequest, JetcobotPickResponse    # 로봇팔 책장에서 책 꺼내기 요청 및 응답
from libro_control_msgs.msg import JetcobotPlaceRequest, JetcobotPlaceResponse  # 로봇팔 바구니에서 책 꺼내기 요청 및 응답
import socket
import threading
import yaml
import time
from datetime import datetime

class LibroControllerNode(Node):
    def __init__(self):
        super().__init__('libro_controller_node')
        self.declare_parameter('robot_id', 'libro_1')  # 기본값으로 libro_1 설정 / libro_2 , libro_3 .....
        self.robot_id = self.get_parameter('robot_id').value
        self.control_state = 0
        self.waiting_for_response = False
        self.order_number = None        # 픽업 주문에 대한 주문 data / libro_task_manager에서 topic으로 받아옴
        self.book_name = None           # 특정 책에 대한 책 이름 data / libro_task_manager에서 topic으로 받아옴
        self.book_place = None          # 특정 책에 대한 도서관 책장 위치 data / libro_task_manager에서 topic으로 받아옴
        self.pickup_place = None        # 특정 책에 대한 픽업 위치 data / libro_task_manager에서 topic으로 받아옴
        self.book_pose = None           # 책을 집기 위한 자세 data / Book Recognition AI Service에서 tcp 통신으로 받아옴 / jetcobot이 활용
        self.tcp_host = '192.168.0.158' # Book Recognition AI Service IP 
        self.tcp_port = 9999            # Book Recognition AI Service Port
        self.libro_robot_state = '대기중'

        # Publishers
        self.goal_place_pub = self.create_publisher(PinkyRequest, f'{self.robot_id}/goal_place', 10)            # 목적지 이동 요청 발행
        # self.pick_pose_pub = self.create_publisher(JetcobotPickRequest, f'{self.robot_id}/pick_book', 10)
        self.pick_pose_pub = self.create_publisher(Float32MultiArray, f'{self.robot_id}/pick_book', 10)         # 책장에서 책 집기 요청 발행   
        self.place_pose_pub = self.create_publisher(JetcobotPlaceRequest, f'{self.robot_id}/place_book', 10)    # 바구니에서 책 꺼내기 요청 발행
        self.check_basket_pub = self.create_publisher(String, f'{self.robot_id}/check_basket', 10)              # 바구니 확인 요청 발행
        self.check_cabinet_pub = self.create_publisher(String, f'{self.robot_id}/check_cabinet', 10)            # 픽업란 확인 요청 발행
        self.robot_state_pub = self.create_publisher(LibroRobotState, f'{self.robot_id}/state', 10)             # 로봇 상태 발행
        self.robot_mission_complete_pub = self.create_publisher(Bool, f'{self.robot_id}/completed', 10)         # 로봇 미션 완료 발행
        self.robot_log_pub = self.create_publisher(LibroRobotLog, 'robot_log', 10)                              # 로봇 미션 수행 로그 발행
        self.stop_arm_pub = self.create_publisher(Bool, f'{self.robot_id}/stop_arm', 10)                        # 로봇팔 작업 정지, 재시작 요청 발행
        self.stop_mobile_pub = self.create_publisher(Bool, f'{self.robot_id}/stop_mobile', 10)                  # 로봇 이동 정지, 재시작 요청 발행

        # Subscribers
        self.request_book_sub = self.create_subscription(
            BookPickUp, 'request_book', self.request_book_callback, 10)                         # 책 픽업 미션 구독
        # self.navigate_mission_sub = self.create_subscription(
        #     Navigate, 'request_nav', self.subscribe_navigate_mission_detail, 10)              # 길 안내 미션 구독
        self.goal_response_sub = self.create_subscription(
            PinkyResponse, f'{self.robot_id}/goal_info', self.goal_place_callback, 10)          # 목적지 이동 응답 구독
        self.pick_response_sub = self.create_subscription(
            Bool, f'{self.robot_id}/pick_info', self.pick_book_callback, 10)                    # 책장에서 책 집기 응답 구독
        # self.pick_response_sub = self.create_subscription(
        #     JetcobotPickResponse, f'{self.robot_id}/pick_info', self.pick_book_callback, 10)  # 책장에서 책 집기 응답 구독
        self.place_response_sub = self.create_subscription(
            JetcobotPlaceResponse, f'{self.robot_id}/place_info', self.place_book_callback, 10) # 바구니에서 책 꺼내기 응답 구독
        self.check_basket_response_sub = self.create_subscription(
            String, f'{self.robot_id}/check_basket_info', self.check_basket_callback, 10)       # 바구니 확인 응답 구독
        self.check_cabinet_response_sub = self.create_subscription(
            String, f'{self.robot_id}/check_cabinet_info', self.check_cabinet_callback, 10)     # 픽업란 확인 응답 구독
        self.stop_robot_sub = self.create_subscription(
            Bool, f'{self.robot_id}/stop', self.stop_robot_callback, 10)                        # 로봇 작업 정지/재시작 요청 구독


        self.timer = self.create_timer(1.0, self.state_machine)
        # 로봇 상태를 지속적으로 발행하기 위한 타이머 (0.5초마다)
        self.robot_state_timer = self.create_timer(0.5, self.publish_robot_state)

        self.get_logger().info(f'Robot : {self.robot_id} initialized')

    def state_machine(self):
        if self.waiting_for_response:
            return
        
        if self.control_state == 0:
            self.libro_robot_state = '대기중'
            pass
        elif self.control_state == 1:
            self.libro_robot_state = '도서관 책장 이동중'
            self.publish_goal_place()
            self.waiting_for_response = True
        elif self.control_state == 2:
            self.libro_robot_state = '책 집기 위한 자세 요청중'
            if not self.waiting_for_response:
                self.waiting_for_response = True
                self.request_book_pose_via_tcp()
        elif self.control_state == 3:
            self.libro_robot_state = '책장에서 책 꺼내는중'
            self.publish_pick_book()
            self.waiting_for_response = True
        elif self.control_state == 4:
            self.libro_robot_state = '바구니 확인중'
            self.publish_check_basket()
            self.waiting_for_response = True
        elif self.control_state == 5:
            self.libro_robot_state = '픽업 위치 이동중'
            self.publish_goal_place()
            self.waiting_for_response = True
        elif self.control_state == 6:
            self.libro_robot_state = '바구니에서 책 꺼내는중'
            self.publish_place_book()
            self.waiting_for_response = True
        elif self.control_state == 7:
            self.libro_robot_state = '픽업란 확인중'
            self.publish_check_cabinet()
            self.waiting_for_response = True
        elif self.control_state == 8:
            self.libro_robot_state = f'로봇 작업 정지됨 : {self.last_robot_state}'
            self.publish_stop_robot()
            self.waiting_for_response = True

    def request_book_callback(self, msg):
        if self.control_state == 0:
            if msg.robot_id == self.robot_id:
                self.get_logger().info(f'Robot : {self.robot_id} received book pick up mission: {msg}')
                self.order_number = msg.order_number
                self.book_name = msg.book_name
                self.book_place = msg.book_place
                self.pickup_place = msg.pickup_place
                self.get_logger().info(f'Updated book name: {self.book_name}')
                self.get_logger().info(f'Updated book place: {self.book_place}')
                self.get_logger().info(f'Updated pickup place: {self.pickup_place}')
                self.publish_robot_log(f"새로운 주문 수신: '{self.book_name}' 책 픽업 시작", True)
                self.control_state = 1  # 상태를 1로 변경
                self.publish_state()  # 상태 변경을 발행

    def publish_goal_place(self):
        msg = PinkyRequest()
        if self.control_state == 1: 
            msg.x = self.book_place.x
            msg.y = self.book_place.y
            msg.theta = self.book_place.theta
            self.goal_place_pub.publish(msg)
            self.get_logger().info(f'Robot : {self.robot_id} Published libro_mobile goal place : book place, {msg.x}, {msg.y}, {msg.theta}')
        if self.control_state == 5:
            msg.x = self.pickup_place.x
            msg.y = self.pickup_place.y
            msg.theta = self.pickup_place.theta
            self.goal_place_pub.publish(msg)
            self.get_logger().info(f'Robot : {self.robot_id} Published libro_mobile goal place : pickup place, {msg.x}, {msg.y}, {msg.theta}')

    # def publish_pick_book(self):
    #     msg = JetcobotPickRequest()
    #     msg.book_x = self.book_pose[0]
    #     msg.book_y = self.book_pose[1]
    #     msg.book_z = self.book_pose[2]
    #     msg.book_angle = self.book_pose[3]
    #     self.pick_pose_pub.publish(msg)
    #     self.get_logger().info('Published pick pose')

    def publish_pick_book(self):        # 통신테스트용
        msg = Float32MultiArray()
        # msg.data = self.book_pose  # TCP에서 받아온 값 사용
        msg.data = [-0.04, 0.01, 0.26, 110.23]
        self.pick_pose_pub.publish(msg)
        self.get_logger().info(f"Robot : {self.robot_id} Published pick pose: {msg.data}")

    def publish_place_book(self):
        msg = JetcobotPlaceRequest()
        msg.basket_num = 1
        self.place_pose_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_id} Published place book request : basket_num = {msg.basket_num}, book_name = {self.book_name}')
    
    def publish_check_basket(self):
        msg = String()
        msg.data = "CHECK"  # 내용은 아무거나, 신호만 주면 됨
        self.check_basket_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_id} Published basket check request')

    def publish_check_cabinet(self):
        msg = String()
        msg.data = "CHECK"  # 내용은 아무거나, 신호만 주면 됨
        self.check_cabinet_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_id} Published cabinet check request')

    def publish_stop_robot(self):
        if self.last_control_state == 1 or self.last_control_state == 5:
            msg = Bool()
            msg.data = True
            self.stop_mobile_pub.publish(msg)
            self.get_logger().info(f'Robot : {self.robot_id} Published stop mobile request')
        elif self.last_control_state == 3 or self.last_control_state == 6:
            msg = Bool()
            msg.data = True
            self.stop_arm_pub.publish(msg)
            self.get_logger().info(f'Robot : {self.robot_id} Published stop arm request')

    def request_book_pose_via_tcp(self):
        def tcp_thread():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.connect((self.tcp_host, self.tcp_port))
                    #sock.sendall((self.robot_id + " " + self.book_name).encode('utf-8'))
                    sock.sendall(self.book_name.encode('utf-8'))
                    data = sock.recv(1024)
                    if data:
                        response_data = yaml.unsafe_load(data.decode('utf-8'))
                        if response_data["success"]:
                            # 예시: x, y, z, angle(deg) 사용
                            self.book_pose = [
                                float(response_data["book_x"]),
                                float(response_data["book_y"]),
                                float(response_data["book_z"]),
                                float(response_data["book_angle"])
                            ]
                            self.get_logger().info(f"Robot : {self.robot_id} received book pose: {self.book_pose}")
                            self.publish_robot_log(f"책 위치 인식 완료: x={self.book_pose[0]:.2f}, y={self.book_pose[1]:.2f}, z={self.book_pose[2]:.2f}", True)
                            self.control_state = 3
                        else:
                            error_msg = response_data.get('error', 'Unknown error')
                            self.publish_robot_log(f"책 위치 인식 실패: {error_msg}", False)
                            self.get_logger().warn(f"Robot : {self.robot_id} Book not found: {error_msg}")
                    else:
                        self.publish_robot_log("책 위치 인식 실패: AI 서비스로부터 응답 없음", False)
                        self.get_logger().warn(f"Robot : {self.robot_id} No data received from Book Recognition AI Service")
            except Exception as e:
                self.publish_robot_log(f"책 위치 인식 실패: TCP 통신 오류 - {str(e)}", False)
                self.get_logger().error(f"Robot : {self.robot_id} TCP communication error: {e}")
            self.waiting_for_response = False
            self.publish_state()

        threading.Thread(target=tcp_thread, daemon=True).start()

    def goal_place_callback(self, msg):
        if self.control_state == 1 and msg.success:
            self.publish_robot_log(f"책장 위치({self.book_place.x}, {self.book_place.y})로 이동 완료", msg.success)
            self.get_logger().info(f'Robot : {self.robot_id} received mission response: {msg.message}, moving to state 2')
            self.control_state = 3
            self.waiting_for_response = False
            self.publish_state()
        if self.control_state == 5 and msg.success:
            self.publish_robot_log(f"픽업 위치({self.pickup_place.x}, {self.pickup_place.y})로 이동 완료", msg.success)
            self.get_logger().info(f'Robot : {self.robot_id} received mission response: {msg.message}, moving to state 6')
            self.control_state = 6
            self.waiting_for_response = False
            self.publish_state()

    def place_book_callback(self, msg):
        if self.control_state == 6 and msg.success:
            self.publish_robot_log("바구니에서 책 꺼내기 완료", msg.success)
            self.get_logger().info(f'Robot : {self.robot_id} received pose response: {msg.message}, moving to state 7')
            self.control_state = 7
            self.waiting_for_response = False
            self.publish_state()

    # def pick_book_callback(self, msg):
    #     if self.control_state == 3 and msg.success:
    #         self.publish_robot_log(f"책장에서 '{self.book_name}' 책 꺼내기 완료", msg.success)
    #         self.get_logger().info(f'Robot : {self.robot_id} received pick response: {msg.message}')
    #         self.control_state = 4
    #         self.waiting_for_response = False
    #         self.publish_state()

    def pick_book_callback(self, msg):
        if self.control_state == 3 and msg.data:
            self.publish_robot_log(f"책장에서 '{self.book_name}' 책 꺼내기 완료", msg.data)
            self.get_logger().info(f'Robot : {self.robot_id} received pick response: {msg.data}')
            self.control_state = 4
            self.waiting_for_response = False
            self.publish_state()

    def check_basket_callback(self, msg):
        if self.control_state == 4:
            if msg.data == "ON":
                self.publish_robot_log("바구니에 책 담기 완료", True)
                self.get_logger().info(f'Robot : {self.robot_id} received book check basket: {msg.data}')
                self.control_state = 5
                self.waiting_for_response = False
                self.publish_state()
            elif msg.data == "OFF":
                self.publish_robot_log("바구니에 책이 감지되지 않음 - 재시도 중", False)
                self.get_logger().warn(f'Robot : {self.robot_id} Book not detected in basket: {msg.data}')
                # 상태는 그대로 유지하고 다시 확인 요청
                self.publish_check_basket()

    def check_cabinet_callback(self, msg):
        if self.control_state == 7:
            if msg.data == "ON":
                self.publish_robot_log(f"픽업란에 '{self.book_name}' 책 배치 완료", True)
                complete_msg = Bool()
                complete_msg.data = True
                self.robot_mission_complete_pub.publish(complete_msg)
                self.get_logger().info(f'Robot : {self.robot_id} received cabinet check: {msg.data}')
                self.control_state = 0
                self.book_name = None
                self.book_place = None
                self.pickup_place = None
                self.book_pose = None
                self.waiting_for_response = False
                self.publish_state()
            elif msg.data == "OFF":
                self.publish_robot_log("픽업란에 책이 감지되지 않음 - 재시도 중", False)
                self.get_logger().warn(f'Robot : {self.robot_id} Book not detected in cabinet: {msg.data}')
                # 상태는 그대로 유지하고 다시 확인 요청
                self.publish_check_cabinet()

    def stop_robot_callback(self, msg):
        if self.control_state != 8:  # 현재 상태가 정지 상태가 아닐 때
            if msg.data:  # 정지 요청이 들어왔을 때
                self.last_control_state = self.control_state  # 현재 상태 저장
                self.last_robot_state = self.libro_robot_state
                self.control_state = 8     # 정지 상태로 변경
                self.libro_robot_state = f'로봇 작업 정지됨 : {self.last_robot_state}'
                self.publish_robot_log("로봇 작업 강제 중지", True)
                self.publish_stop_robot()  # 정지 명령 발행
                self.publish_state()
        else:  # 현재 상태가 정지 상태일 때
            if not msg.data:  # 재시작 요청이 들어왔을 때
                self.control_state = self.last_control_state  # 이전 상태로 복구
                self.libro_robot_state = self.last_robot_state
                self.publish_robot_log("로봇 작업 재시작", True)
                self.publish_state()
            else:  # 계속 정지 상태 유지
                self.libro_robot_state = f'로봇 작업 정지됨 : {self.last_robot_state}'
                self.control_state = 8
                self.publish_state()    

    def publish_state(self):
        msg = Int32()
        msg.data = self.control_state
        # self.controller_state_pub.publish(msg)
        self.get_logger().info(f'Published state: {self.control_state}')
    
    def publish_robot_state(self):
        msg = LibroRobotState()
        msg.code_id = f"0x{format(self.control_state, '02x')}"  # 16진수로 변환하여 0x 접두사 추가
        msg.message = str(self.libro_robot_state)  # 문자열로 명시적 변환
        self.robot_state_pub.publish(msg)
        # self.get_logger().info(f'Published robot state: {msg.state_id} - {self.libro_robot_state}')

    def publish_robot_log(self, robot_log, success):
        msg = LibroRobotLog()
        # Convert current time to formatted string
        current_time = self.get_clock().now()
        timestamp = datetime.fromtimestamp(current_time.seconds_nanoseconds()[0] + current_time.seconds_nanoseconds()[1] / 1e9)
        formatted_time = timestamp.strftime('%Y-%m-%d %H:%M:%S')

        msg.timestamp = formatted_time
        msg.robot_id = self.robot_id
        msg.order_number = self.order_number if self.order_number else 0
        msg.robot_log = robot_log
        msg.success = success
        self.robot_log_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_id} published robot log: {robot_log} (success: {success})')

def main(args=None):
    rclpy.init(args=args)
    node = LibroControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()