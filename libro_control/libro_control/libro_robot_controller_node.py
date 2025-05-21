import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32MultiArray, Bool
from libro_control_msgs.msg import BookPickUp, Navigate, LibroRobotLog # 책 픽업 미션 , 길 안내 미션, (구독 from task_manager) / 로봇 상태, 로그 (발행 for task_manager)
from libro_control_msgs.msg import PinkyRequest, PinkyResponse         # 로봇 작동 응답 (구독 from 로봇팔 패키지, 주행패키지)
import socket
import threading
import yaml
from datetime import datetime
import json

class LibroControllerNode(Node):
    def __init__(self):
        super().__init__('libro_controller_node')
        self.declare_parameter('robot_id', 'libro1')  # 기본값으로 libro1 설정 / libro2 , libro3 .....
        self.robot_id = self.get_parameter('robot_id').value
        self.control_state = 0
        self.waiting_for_response = False
        self.order_number = None        # 픽업 주문에 대한 주문 data / libro_task_manager에서 topic으로 받아옴
        self.book_name_1 = None         # 특정 책에 대한 책 이름 data / libro_task_manager에서 topic으로 받아옴
        self.book_place_1 = None        # 특정 책에 대한 도서관 책장 위치 data / libro_task_manager에서 topic으로 받아옴
        self.book_name_2 = None         # 특정 책에 대한 책 이름 data / libro_task_manager에서 topic으로 받아옴
        self.book_place_2 = None        # 특정 책에 대한 도서관 책장 위치 data / libro_task_manager에서 topic으로 받아옴
        self.pickup_place_name = None  # 특정 책에 대한 픽업 위치 이름 data / libro_task_manager에서 topic으로 받아옴
        self.pickup_place = None        # 특정 책에 대한 픽업 위치 data / libro_task_manager에서 topic으로 받아옴
        self.start_place = None         # 길 안내 시작 위치 data / libro_task_manager에서 topic으로 받아옴
        self.end_place = None           # 길 안내 끝 위치 data / libro_task_manager에서 topic으로 받아옴
        self.user_id = None             # 갈 안내 사용자 식별용 아이디 data / libro_task_manager에서 topic으로 받아옴
        self.book_pose = None           # 책을 집기 위한 자세 data / Book Recognition AI Service에서 tcp 통신으로 받아옴 / jetcobot이 활용
        self.target_basket = None       # 책을 담을 바구니 data / 아두이노 패키지에서 topic으로 받아옴
        self.target_cabinet = None      # 책을 넣을 픽업함 data / 아두이노 패키지에서 topic으로 받아옴
        self.tcp_host = '192.168.0.158' # Book Recognition AI Service IP 
        self.tcp_port = 9999            # Book Recognition AI Service Port
        self.libro_robot_state = '대기중'   
        self.current_book_index = 0  # 현재 처리 중인 책의 인덱스 (0 또는 1)
        self.total_books = 0        # 처리해야 할 총 책의 수

        # Publishers
        self.goal_place_pub = self.create_publisher(PinkyRequest, f'{self.robot_id}/goal_place', 10)        # 목적지 이동 요청 발행
        self.pick_pose_pub = self.create_publisher(Float32MultiArray, f'{self.robot_id}/pick_book', 10)     # 책장에서 책 집기 요청 발행   
        self.place_basket_pub = self.create_publisher(String, f'{self.robot_id}/place_basket', 10)          # 바구니에 책 담기 요청 발행
        self.place_cabinet_pub = self.create_publisher(String, f'{self.robot_id}/place_cabinet', 10)        # 픽업함에 책 담기 요청 발행
        self.empty_basket_pub = self.create_publisher(String, f'{self.robot_id}/empty_basket', 10)          # 빈 바구니 확인 요청 발행
        self.empty_cabinet_pub = self.create_publisher(String, f'{self.robot_id}/empty_cabinet', 10)        # 빈 픽업함 확인 요청 발행
        self.check_basket_pub = self.create_publisher(String, f'{self.robot_id}/check_basket', 10)          # 바구니 재확인 요청 발행
        self.check_cabinet_pub = self.create_publisher(String, f'{self.robot_id}/check_cabinet', 10)        # 픽업함 재확인 요청 발행
        self.check_user_pub = self.create_publisher(Int32, f'{self.robot_id}/check_user', 10)              # 사용자 확인 요청 발행
        self.robot_state_pub = self.create_publisher(String, f'{self.robot_id}/state', 10)                  # 로봇 상태 발행
        self.robot_log_pub = self.create_publisher(LibroRobotLog, 'robot_log', 10)                          # 로봇 미션 수행 로그 발행
        self.robot_error_pub = self.create_publisher(String, f'{self.robot_id}/error', 10)                  # 로봇 오류 발행
        self.robot_mission_complete_pub = self.create_publisher(Bool, f'{self.robot_id}/completed', 10)     # 로봇 미션 완료 발행
        self.stop_arm_pub = self.create_publisher(Bool, f'{self.robot_id}/stop_arm', 10)                    # 로봇팔 작업 정지, 재시작 요청 발행
        self.stop_mobile_pub = self.create_publisher(Bool, f'{self.robot_id}/stop_mobile', 10)              # 로봇 이동 정지, 재시작 요청 발행

        # Subscribers
        self.request_book_sub = self.create_subscription(
            BookPickUp, 'request_book', self.request_book_callback, 10)                         # 책 픽업 미션 구독
        self.navigate_mission_sub = self.create_subscription(
            Navigate, 'request_nav', self.request_nav_callback, 10)                             # 길 안내 미션 구독
        self.goal_response_sub = self.create_subscription(
            PinkyResponse, f'{self.robot_id}/goal_info', self.goal_place_callback, 10)          # 목적지 이동 응답 구독
        self.pick_response_sub = self.create_subscription(
            Bool, f'{self.robot_id}/pick_info', self.pick_book_callback, 10)                    # 책장에서 책 집기 응답 구독
        self.place_response_sub = self.create_subscription(
            Bool, f'{self.robot_id}/place_info', self.place_book_callback, 10)                  # 바구니 or 픽업함에 책 보관 응답 구독
        self.empty_basket_response_sub = self.create_subscription(
            String, f'{self.robot_id}/empty_basket_info', self.empty_basket_callback, 10)       # 빈 바구니 확인 응답 구독
        self.empty_cabinet_response_sub = self.create_subscription(
            String, f'{self.robot_id}/empty_cabinet_info', self.empty_cabinet_callback, 10)     # 빈 픽업함 확인 응답 구독
        self.check_basket_response_sub = self.create_subscription(
            String, f'{self.robot_id}/check_basket_info', self.check_basket_callback, 10)       # 바구니 재확인 응답 구독
        self.check_cabinet_response_sub = self.create_subscription(
            String, f'{self.robot_id}/check_cabinet_info', self.check_cabinet_callback, 10)     # 픽업함 재확인 응답 구독
        self.check_user_response_sub = self.create_subscription(
            Bool, f'{self.robot_id}/check_user_info', self.check_user_callback, 10)             # 호출 유저 확인 응답 구독
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
            self.libro_robot_state = '비어있는 바구니 확인중'
            self.publish_empty_basket()
            self.waiting_for_response = True
        elif self.control_state == 5:
            self.libro_robot_state = '바구니에 책 담기 중'
            self.publish_place_book()
            self.waiting_for_response = True
        elif self.control_state == 6:
            self.libro_robot_state = '바구니에 담겼는지 확인중'
            self.publish_check_basket()
            self.waiting_for_response = True
        elif self.control_state == 7:
            if self.current_book_index < self.total_books - 1:
                self.current_book_index += 1
                self.control_state = 1
                self.libro_robot_state = f'두 번째 책({self.book_name_2}) 픽업 시작'
            else:
                self.control_state = 8
        elif self.control_state == 8:
            self.libro_robot_state = '픽업 위치 이동중'
            self.publish_goal_place()
            self.waiting_for_response = True
        elif self.control_state == 9:
            self.libro_robot_state = '비어있는 픽업함 확인중'
            self.publish_empty_cabinet()
            self.waiting_for_response = True
        elif self.control_state == 10:
            self.libro_robot_state = '책 꺼내서 픽업함에 넣기 중'
            self.publish_place_book()
            self.waiting_for_response = True
        elif self.control_state == 11:
            self.libro_robot_state = '픽업함 확인중'
            self.publish_check_cabinet()
            self.waiting_for_response = True
        elif self.control_state == 12:
            self.libro_robot_state = '길 안내 시작 위치 이동중'
            self.publish_goal_place()
            self.waiting_for_response = True
        elif self.control_state == 13:
            self.libro_robot_state = '호출 유저 확인중'
            self.publish_check_user()
            self.waiting_for_response = True
        elif self.control_state == 14:
            self.libro_robot_state = '길 안내 끝 위치 이동중'
            self.publish_goal_place()
            self.waiting_for_response = True
        elif self.control_state == 15:
            self.libro_robot_state = f'로봇 작업 정지됨 : {self.last_robot_state}'
            self.publish_stop_robot()
            self.waiting_for_response = True

    def request_book_callback(self, msg):
        if self.control_state == 0:
            if msg.robot_id == self.robot_id:
                self.order_number = msg.order_number
                self.book_name_1 = msg.book_name_1
                self.book_place_1 = msg.book_place_1
                self.book_name_2 = msg.book_name_2
                self.book_place_2 = msg.book_place_2
                self.pickup_place = msg.pickup_place
                self.pickup_place_name = msg.pickup_place_name

                # 책 개수 설정
                self.total_books = 2 if self.book_name_2 else 1
                self.current_book_index = 0
                
                self.publish_robot_log(f"새로운 주문 수신: '{self.book_name_1}'" + 
                                     (f"와 '{self.book_name_2}'" if self.book_name_2 else "") + 
                                     " 책 픽업 시작", True)
                self.control_state = 1
                self.publish_state()

    def request_nav_callback(self, msg):
        if self.control_state == 0:
            if msg.robot_id == self.robot_id:
                self.get_logger().info(f'Robot : {self.robot_id} received navigate mission: {msg}')
                self.order_number = msg.order_number
                self.start_place = msg.start_place
                self.end_place = msg.end_place
                self.user_id = msg.user_id
                self.get_logger().info(f'Updated start place: {self.start_place}')
                self.get_logger().info(f'Updated end place: {self.end_place}')
                self.get_logger().info(f'Updated user id: {self.user_id}')
                self.publish_robot_log(f"새로운 주문 수신: 유저 정보 {self.user_id} 길 안내 시작", True)
                self.control_state = 12  # 상태를 1로 변경
                self.publish_state()  # 상태 변경을 발행

    def publish_goal_place(self):
        msg = PinkyRequest()
        if self.control_state == 1:
            # 현재 처리 중인 책의 위치로 이동
            current_place = self.book_place_1 if self.current_book_index == 0 else self.book_place_2
            msg.x = current_place.x
            msg.y = current_place.y
            msg.theta = current_place.theta
            self.goal_place_pub.publish(msg)
            self.get_logger().info(f'Robot : {self.robot_id} Published libro_mobile goal place : book place, {msg.x}, {msg.y}, {msg.theta}')
        elif self.control_state == 8:
            msg.x = self.pickup_place.x
            msg.y = self.pickup_place.y
            msg.theta = self.pickup_place.theta
            self.goal_place_pub.publish(msg)
            self.get_logger().info(f'Robot : {self.robot_id} Published libro_mobile goal place : pickup place, {msg.x}, {msg.y}, {msg.theta}')
        elif self.control_state == 12:
            msg.x = self.start_place.x
            msg.y = self.start_place.y
            msg.theta = self.start_place.theta
            self.goal_place_pub.publish(msg)
            self.get_logger().info(f'Robot : {self.robot_id} Published libro_mobile goal place : start place, {msg.x}, {msg.y}, {msg.theta}')
        elif self.control_state == 14:
            msg.x = self.end_place.x
            msg.y = self.end_place.y
            msg.theta = self.end_place.theta
            self.goal_place_pub.publish(msg)
            self.get_logger().info(f'Robot : {self.robot_id} Published libro_mobile goal place : end place, {msg.x}, {msg.y}, {msg.theta}')

    def publish_pick_book(self):        # 통신테스트용
        msg = Float32MultiArray()
        # msg.data = self.book_pose  # TCP에서 받아온 값 사용
        msg.data = [-0.04, 0.01, 0.26, 110.23]
        self.pick_pose_pub.publish(msg)
        self.get_logger().info(f"Robot : {self.robot_id} Published pick pose: {msg.data}")

    def publish_place_book(self):
        if self.control_state == 5:
            msg = String()
            msg.data = self.target_basket
            current_book = self.book_name_1 if self.current_book_index == 0 else self.book_name_2
            self.place_basket_pub.publish(msg)
            self.get_logger().info(f'Robot : {self.robot_id} Published place basket request : basket_id = {msg.data}, book_name = {current_book}')
        elif self.control_state == 10:
            msg = String()
            # 픽업함 ID와 두 권의 책 정보를 포함
            if self.total_books == 2:
                msg.data = f"{self.pickup_place_name},{self.target_cabinet},B1,B2"
            else:
                msg.data = f"{self.pickup_place_name},{self.target_cabinet},B1"
            self.place_cabinet_pub.publish(msg)
            self.get_logger().info(f'Robot : {self.robot_id} Published place cabinet request : cabinet_id = {self.target_cabinet}')
    
    def publish_empty_basket(self):
        msg = String()
        msg.data = "Find Empty Basket"  # 내용은 아무거나, 신호만 주면 됨
        self.empty_basket_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_id} Published check empty basket request')
    
    def publish_empty_cabinet(self):
        msg = String()
        msg.data = f'Book Number : {self.total_books}, Pickup Place : {self.pickup_place_name}'
        self.empty_cabinet_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_id} Published check empty cabinet request')

    def publish_check_basket(self):
        msg = String()
        msg.data = self.target_basket 
        self.check_basket_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_id} Published check basket {self.target_basket} request')

    def publish_check_cabinet(self):
        msg = String()
        msg.data = self.target_cabinet
        self.check_cabinet_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_id} Published cabinet check {self.target_cabinet} request')

    def publish_check_user(self):
        msg = Int32()
        msg.data = self.user_id
        self.check_user_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_id} Published user check request')

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
                    # 현재 처리 중인 책 이름 전송
                    current_book = self.book_name_1 if self.current_book_index == 0 else self.book_name_2
                    sock.sendall(current_book.encode('utf-8'))
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
                            self.waiting_for_response = False
                            self.publish_state()
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
            current_place = self.book_place_1 if self.current_book_index == 0 else self.book_place_2
            self.publish_robot_log(f"책장 위치({current_place.x}, {current_place.y})로 이동 완료", msg.success)
            self.get_logger().info(f'Robot : {self.robot_id} received mission response: {msg.message}, moving to state 2')
            self.control_state = 3
            self.waiting_for_response = False
            self.publish_state()
        if self.control_state == 8 and msg.success:
            self.publish_robot_log(f"픽업 위치({self.pickup_place.x}, {self.pickup_place.y})로 이동 완료", msg.success)
            self.get_logger().info(f'Robot : {self.robot_id} received mission response: {msg.message}, moving to state 6')
            self.control_state = 9
            self.waiting_for_response = False
            self.publish_state()
        if self.control_state == 12 and msg.success:
            self.publish_robot_log(f"길 안내 시작 위치({self.start_place.x}, {self.start_place.y})로 이동 완료", msg.success)
            self.get_logger().info(f'Robot : {self.robot_id} received mission response: {msg.message}, moving to state 10')
            self.control_state = 13
            self.waiting_for_response = False
            self.publish_state()
        if self.control_state == 14 and msg.success:
            self.publish_robot_log(f"길 안내 끝 위치({self.end_place.x}, {self.end_place.y})로 이동 완료", msg.success)
            self.get_logger().info(f'Robot : {self.robot_id} received mission response: {msg.message}, moving to state 0')
            complete_msg = Bool()
            complete_msg.data = True
            self.robot_mission_complete_pub.publish(complete_msg)
            self.start_place = None
            self.end_place = None
            self.user_id = None
            self.control_state = 0
            self.waiting_for_response = False
            self.publish_state()

    def pick_book_callback(self, msg):
        if self.control_state == 3 and msg.data:
            current_book = self.book_name_1 if self.current_book_index == 0 else self.book_name_2
            self.publish_robot_log(f"책장에서 '{current_book}' 책 꺼내기 완료", msg.data)
            self.get_logger().info(f'Robot : {self.robot_id} received pick response: {msg.data}')
            self.control_state = 4
            self.waiting_for_response = False
            self.publish_state()

    def place_book_callback(self, msg):
        if self.control_state == 5 and msg.data:
            self.publish_robot_log("로봇팔 바구니에 책 담기 동작 완료", msg.data)
            self.get_logger().info(f'Robot : {self.robot_id} received pose response: {msg.data}, moving to state 6')
            self.book_pose = None
            self.control_state = 6
            self.waiting_for_response = False
            self.publish_state()
        elif self.control_state == 10 and msg.data:
            self.publish_robot_log("로봇팔 픽업함에 책 담기 동작 완료", msg.data)
            self.get_logger().info(f'Robot : {self.robot_id} received pose response: {msg.data}, moving to state 7')
            self.control_state = 11
            self.waiting_for_response = False
            self.publish_state()

    def empty_basket_callback(self, msg):
        if self.control_state == 4:
            self.target_basket = msg.data
            self.publish_robot_log(f"빈 바구니 확인 완료: {self.target_basket}", True)
            self.get_logger().info(f'Robot : {self.robot_id} received empty basket_id : {self.target_basket}')
            self.control_state = 5
            self.waiting_for_response = False
            self.publish_state()
    
    def empty_cabinet_callback(self, msg):
        if self.control_state == 9:
            self.target_cabinet = msg.data
            self.publish_robot_log(f"빈 픽업함 확인 완료: {self.target_cabinet}", True)
            self.get_logger().info(f'Robot : {self.robot_id} received empty cabinet_id : {self.target_cabinet}')
            self.control_state = 10
            self.waiting_for_response = False
            self.publish_state()
            
    def check_basket_callback(self, msg):
        if self.control_state == 6:
            if msg.data == "ON":
                self.publish_robot_log("바구니에 책 배치 완료", True)
                self.get_logger().info(f'Robot : {self.robot_id} received book check basket: {msg.data}')
                self.control_state = 7
                self.waiting_for_response = False
                self.publish_state()
            elif msg.data == "OFF":
                self.publish_robot_log("바구니에 책이 감지되지 않음 - 재시도 중", False)
                self.get_logger().warn(f'Robot : {self.robot_id} Book not detected in basket: {msg.data}')
                # 상태는 그대로 유지하고 다시 확인 요청
                self.publish_check_basket()

    def check_cabinet_callback(self, msg):
        if self.control_state == 11:
            if msg.data == "ON":
                current_book = self.book_name_1 if self.current_book_index == 0 else self.book_name_2
                self.publish_robot_log(f"픽업란에 '{current_book}' 책 배치 완료", True)
                complete_msg = Bool()
                complete_msg.data = True
                self.robot_mission_complete_pub.publish(complete_msg)
                self.get_logger().info(f'Robot : {self.robot_id} received cabinet check: {msg.data}')
                self.control_state = 0
                self.book_name_1 = None
                self.book_place_1 = None
                self.book_name_2 = None
                self.book_place_2 = None
                self.pickup_place = None
                self.target_basket = None
                self.target_cabinet = None
                self.waiting_for_response = False
                self.publish_state()
            elif msg.data == "OFF":
                self.publish_robot_log("픽업란에 책이 감지되지 않음 - 재시도 중", False)
                self.get_logger().warn(f'Robot : {self.robot_id} Book not detected in cabinet: {msg.data}')
                # 상태는 그대로 유지하고 다시 확인 요청
                self.publish_check_cabinet()

    def check_user_callback(self, msg):
        if self.control_state == 13:
            if msg.data:
                self.publish_robot_log("호출 유저 확인 완료", True)
                self.get_logger().info(f'Robot : {self.robot_id} received user check: {msg.data}')
                self.control_state = 14
                self.waiting_for_response = False
                self.publish_state()
            else:
                self.publish_robot_log("호출 유저 확인 실패", False)
                self.get_logger().warn(f'Robot : {self.robot_id} User not detected: {msg.data}')
                # 상태는 그대로 유지하고 다시 확인 요청
                self.publish_check_user()

    def stop_robot_callback(self, msg):
        if self.control_state != 15:  # 현재 상태가 정지 상태가 아닐 때
            if msg.data:  # 정지 요청이 들어왔을 때
                self.last_control_state = self.control_state  # 현재 상태 저장
                self.last_robot_state = self.libro_robot_state
                self.control_state = 15     # 정지 상태로 변경
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
                self.control_state = 15
                self.publish_state()    

    def publish_state(self):
        msg = Int32()
        msg.data = self.control_state
        # self.controller_state_pub.publish(msg)
        self.get_logger().info(f'Published state: {self.control_state}')
    
    def publish_robot_state(self):
        msg = String()
        msg_data = {
            "status": f"0x{format(self.control_state, '02x')}",
            "message": str(self.libro_robot_state)
        }
        msg.data = str(json.dumps(msg_data, ensure_ascii=False))
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

        # 실패한 경우 오류 메시지를 별도 토픽으로 발행
        if not success:
            error_msg = String()
            error_data = {
                "status": f"0x{format(self.control_state + 64, '02x')}",
                "message": robot_log
            }
            error_msg.data = json.dumps(error_data, ensure_ascii=False)
            self.robot_error_pub.publish(error_msg)
            self.get_logger().warn(f'Robot : {self.robot_id} published error: {error_data}')

def main(args=None):
    rclpy.init(args=args)
    node = LibroControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

