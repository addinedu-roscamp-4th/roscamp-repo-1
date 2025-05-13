import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Float32MultiArray, Bool
from libro_control_msgs.msg import PinkyMissionResponse, JetcobotMissionPoseResponse, JetcobotMissionPickResponse, BookPickUpMission, PinkyMissionPlace, LibroRobotLog 
from libro_picker_srvs.msg import AngleCommand, PickBookCommand
import socket
import threading
import yaml
import time

class LibroControllerNode(Node):
    def __init__(self):
        super().__init__('libro_controller_node')
        self.control_mission_state = 0
        self.waiting_for_response = False
        self.order_number = None        # 픽업 주문에 대한 주문 data / dataservice에서 topic으로 받아옴
        self.book_name = None           # 특정 책에 대한 책 이름 data / dataservice에서 topic으로 받아옴
        self.book_place = None          # 특정 책에 대한 도서관 책장 위치 data / dataservice에서 topic으로 받아옴
        self.pickup_place = None        # 특정 책에 대한 픽업 위치 data / dataservice에서 topic으로 받아옴
        self.book_pose = None           # 책을 집기 위한 자세 data / Book Recognition AI Service에서 tcp 통신으로 받아옴 / jetcobot이 활용
        self.tcp_host = '192.168.0.158'  # Book Recognition AI Service IP 
        self.tcp_port = 9999         # Book Recognition AI Service Port
        self.libro_robot_state = '대기중'
        self.robot_name_id = 'libro_robot_2'  # 로봇 고유 이름/ID  'libro_robot_1', 'libro_robot_2' 까지만 지원

        # Publishers
        self.mission_place_pub = self.create_publisher(PinkyMissionPlace, 'libro_mobile_goal_place', 10)
        self.place_from_basket_pub = self.create_publisher(AngleCommand, 'jetcobot_angles_command', 10)
        # self.pick_pose_pub = self.create_publisher(PickBookCommand, 'pick_pose_topic', 10)
        self.pick_pose_pub = self.create_publisher(Float32MultiArray, 'pick_pose_topic', 10)
        self.basket_check_pub = self.create_publisher(String, 'basket_check_topic', 10)
        self.counter_check_pub = self.create_publisher(String, 'counter_check_topic', 10)
        self.state_pub = self.create_publisher(Int32, 'controller_2_mission_state', 10)
        self.libro_robot_state_pub = self.create_publisher(String, 'libro_robot_2_pickup_state', 10)
        self.libro_robot_log_pub = self.create_publisher(LibroRobotLog, 'libro_robot_log', 10)
        self.robot2_mission_complete_pub = self.create_publisher(Bool, 'libro_robot_2_mission_complete', 10)

        

        # Subscribers
        self.book_pick_up_mission_sub = self.create_subscription(
            BookPickUpMission, 'book_pick_up_mission', self.subscribe_book_pick_up_mission_detail, 10)
        self.mission_response_sub = self.create_subscription(
            PinkyMissionResponse, 'pinky_mission_response', self.mission_response_callback, 10)
        self.pose_response_sub = self.create_subscription(
            JetcobotMissionPoseResponse, 'jetcobot_mission_place_response', self.place_response_callback, 10)
        # self.pick_response_sub = self.create_subscription(
        #     JetcobotMissionPickResponse, 'jetcobot_mission_pick_response', self.pick_response_callback, 10)
        self.pick_response_sub = self.create_subscription(
            Bool, 'jetcobot_pick_response', self.pick_response_callback, 10)
        self.basket_check_response_sub = self.create_subscription(
            String, 'basket_check_response', self.basket_check_response_callback, 10)
        self.counter_check_response_sub = self.create_subscription(
            String, 'counter_check_response', self.counter_check_response_callback, 10)

        self.timer = self.create_timer(1.0, self.state_machine)
        # 로봇 상태를 지속적으로 발행하기 위한 타이머 (0.5초마다)
        self.robot_state_timer = self.create_timer(0.5, self.publish_robot_state)

        self.get_logger().info(f'Robot : {self.robot_name_id} initialized')

    def state_machine(self):
        if self.waiting_for_response:
            return
        
        if self.control_mission_state == 0:
            self.libro_robot_state = '대기중'
            pass
        elif self.control_mission_state == 1:
            self.libro_robot_state = '도서관 책장 이동중'
            self.publish_mission_place()
            self.waiting_for_response = True
        # elif self.control_mission_state == 2:
        #     self.publish_place_from_basket()
        #     self.waiting_for_response = True
        elif self.control_mission_state == 3:
            self.libro_robot_state = '책 집기 위한 자세 요청중'
            if not self.waiting_for_response:
                self.waiting_for_response = True
                self.request_book_pose_via_tcp()
        elif self.control_mission_state == 4:
            self.libro_robot_state = '책장에서 책 꺼내는중'
            self.publish_pick_pose()
            self.waiting_for_response = True
        elif self.control_mission_state == 5:
            self.libro_robot_state = '바구니 확인중'
            self.publish_basket_check()
            self.waiting_for_response = True
        elif self.control_mission_state == 6:
            self.libro_robot_state = '픽업 위치 이동중'
            self.publish_mission_place()
            self.waiting_for_response = True
        elif self.control_mission_state == 7:
            self.libro_robot_state = '바구니에서 책 꺼내는중'
            self.publish_place_from_basket()
            self.waiting_for_response = True
        elif self.control_mission_state == 8:
            self.libro_robot_state = '픽업란 확인중'
            self.publish_counter_check()
            self.waiting_for_response = True

    def subscribe_book_pick_up_mission_detail(self, msg):
        if self.control_mission_state == 0:
            if msg.robot_name_id == self.robot_name_id:
                self.get_logger().info(f'Robot : {self.robot_name_id} received book pick up mission: {msg}')
                self.order_number = msg.order_number
                self.book_name = msg.book_name
                self.book_place = msg.book_place
                self.pickup_place = msg.pickup_place
                self.get_logger().info(f'Updated book name: {self.book_name}')
                self.get_logger().info(f'Updated book place: {self.book_place}')
                self.get_logger().info(f'Updated pickup place: {self.pickup_place}')
                self.publish_robot_log(f"새로운 주문 수신: '{self.book_name}' 책 픽업 시작", True)
                self.control_mission_state = 1  # 상태를 1로 변경
                self.publish_state()  # 상태 변경을 발행

    def publish_mission_place(self):
        msg = PinkyMissionPlace()
        msg.robot_name_id = self.robot_name_id
        if self.control_mission_state == 1: 
            msg.x = self.book_place.x
            msg.y = self.book_place.y
            msg.theta = self.book_place.theta
            self.mission_place_pub.publish(msg)
            self.get_logger().info(f'Robot : {self.robot_name_id} Published libro_mobile goal place : book place, {msg.x}, {msg.y}, {msg.theta}')
        if self.control_mission_state == 6:
            msg.x = self.pickup_place.x
            msg.y = self.pickup_place.y
            msg.theta = self.pickup_place.theta
            self.mission_place_pub.publish(msg)
            self.get_logger().info(f'Robot : {self.robot_name_id} Published libro_mobile goal place : pickup place, {msg.x}, {msg.y}, {msg.theta}')

    def publish_place_from_basket(self):
        msg = AngleCommand()
        msg.angles = [0.0, 60.0, -110.0, 50.0, 0.0,  -45.0]  # 예시 각도 값들
        msg.speed = 50 # 예시 속도 값
        self.place_from_basket_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_name_id} Published picture pose')

    # def publish_pick_pose(self):
    #     msg = PickBookCommand()
    #     msg.x = 3.0
    #     msg.y = 4.0
    #     msg.z = 1.57
    #     msg.angle = 0.0
    #     self.pick_pose_pub.publish(msg)
    #     self.get_logger().info('Published pick pose')

    def publish_pick_pose(self):
        msg = Float32MultiArray()
        # msg.data = self.book_pose  # TCP에서 받아온 값 사용
        msg.data = [-0.04, 0.01, 0.26, 110.23]
        self.pick_pose_pub.publish(msg)
        self.get_logger().info(f"Robot : {self.robot_name_id} Published pick pose: {msg.data}")

    def publish_basket_check(self):
        msg = String()
        msg.data = "CHECK"  # 내용은 아무거나, 신호만 주면 됨
        self.basket_check_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_name_id} Published basket check request')

    def publish_counter_check(self):
        msg = String()
        msg.data = "CHECK"  # 내용은 아무거나, 신호만 주면 됨
        self.counter_check_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_name_id} Published counter check request')

    def request_book_pose_via_tcp(self):
        def tcp_thread():
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.connect((self.tcp_host, self.tcp_port))
                    #sock.sendall((self.robot_name_id + " " + self.book_name).encode('utf-8'))
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
                            self.get_logger().info(f"Robot : {self.robot_name_id} received book pose: {self.book_pose}")
                            self.publish_robot_log(f"책 위치 인식 완료: x={self.book_pose[0]:.2f}, y={self.book_pose[1]:.2f}, z={self.book_pose[2]:.2f}", True)
                            self.control_mission_state = 4
                        else:
                            error_msg = response_data.get('error', 'Unknown error')
                            self.publish_robot_log(f"책 위치 인식 실패: {error_msg}", False)
                            self.get_logger().warn(f"Robot : {self.robot_name_id} Book not found: {error_msg}")
                    else:
                        self.publish_robot_log("책 위치 인식 실패: AI 서비스로부터 응답 없음", False)
                        self.get_logger().warn(f"Robot : {self.robot_name_id} No data received from Book Recognition AI Service")
            except Exception as e:
                self.publish_robot_log(f"책 위치 인식 실패: TCP 통신 오류 - {str(e)}", False)
                self.get_logger().error(f"Robot : {self.robot_name_id} TCP communication error: {e}")
            self.waiting_for_response = False
            self.publish_state()

        threading.Thread(target=tcp_thread, daemon=True).start()

    def mission_response_callback(self, msg):
        if self.control_mission_state == 1 and msg.success:
            self.publish_robot_log(f"책장 위치({self.book_place.x}, {self.book_place.y})로 이동 완료", msg.success)
            self.get_logger().info(f'Robot : {self.robot_name_id} received mission response: {msg.message}, moving to state 3')
            self.control_mission_state = 4
            self.waiting_for_response = False
            self.publish_state()
        if self.control_mission_state == 6 and msg.success:
            self.publish_robot_log(f"픽업 위치({self.pickup_place.x}, {self.pickup_place.y})로 이동 완료", msg.success)
            self.get_logger().info(f'Robot : {self.robot_name_id} received mission response: {msg.message}, moving to state 7')
            self.control_mission_state = 7
            self.waiting_for_response = False
            self.publish_state()

    def place_response_callback(self, msg):
        if self.control_mission_state == 7 and msg.success:
            self.publish_robot_log("바구니에서 책 꺼내기 완료", msg.success)
            self.get_logger().info(f'Robot : {self.robot_name_id} received pose response: {msg.message}, moving to state 8')
            self.control_mission_state = 8
            self.waiting_for_response = False
            self.publish_state()

    # def pick_response_callback(self, msg):
    #     if self.control_mission_state == 4 and msg.success:
    #         self.get_logger().info(f'Received pick response: {msg.message}')
    #         self.waiting_for_response = False
    #         self.publish_state()

    def pick_response_callback(self, msg):
        if self.control_mission_state == 4 and msg.data:
            self.publish_robot_log(f"책장에서 '{self.book_name}' 책 꺼내기 완료", msg.data)
            self.get_logger().info(f'Robot : {self.robot_name_id} received pick response: {msg.data}')
            self.control_mission_state = 5
            self.waiting_for_response = False
            self.publish_state()

    def basket_check_response_callback(self, msg):
        if self.control_mission_state == 5 and msg.data == "ON":
            self.publish_robot_log("바구니에 책 담기 완료", True)
            self.get_logger().info(f'Robot : {self.robot_name_id} received book check basket: {msg.data}')
            self.control_mission_state = 6
            self.waiting_for_response = False
            self.publish_state()

    def counter_check_response_callback(self, msg):
        if self.control_mission_state == 8 and msg.data == "ON":
            self.publish_robot_log(f"픽업란에 '{self.book_name}' 책 배치 완료", True)
            complete_msg = Bool()
            complete_msg.data = True
            self.robot2_mission_complete_pub.publish(complete_msg)
            self.get_logger().info(f'Robot : {self.robot_name_id} received counter check: {msg.data}')
            self.control_mission_state = 0
            self.book_name = None
            self.book_place = None
            self.pickup_place = None
            self.book_pose = None
            self.waiting_for_response = False
            self.publish_state()

    def publish_state(self):
        msg = Int32()
        msg.data = self.control_mission_state
        self.state_pub.publish(msg)
        self.get_logger().info(f'Published state: {self.control_mission_state}')
    
    def publish_robot_state(self):
        msg = String()
        msg.data = self.robot_name_id + " : " + self.libro_robot_state
        self.libro_robot_state_pub.publish(msg)
        #self.get_logger().info(f'Published robot state: {self.libro_robot_state}')

    def publish_robot_log(self, robot_log, success):
        msg = LibroRobotLog()
        current_time = self.get_clock().now().to_msg()
        msg.time = float(current_time.sec) + float(current_time.nanosec) / 1e9  # Convert to float32 seconds
        msg.robot_name_id = self.robot_name_id
        msg.order_number = self.order_number if self.order_number else 0
        msg.robot_log = robot_log
        msg.success = success
        self.libro_robot_log_pub.publish(msg)
        self.get_logger().info(f'Robot : {self.robot_name_id} published robot log: {robot_log} (success: {success})')

def main(args=None):
    rclpy.init(args=args)
    node = LibroControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()