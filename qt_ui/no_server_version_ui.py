import os, sys, math, threading, copy
import time
import requests
# import bcrypt # 서버에 있는 비밀번호는 암호화 되어있어서 해석하려면 이 모듈을 써야 함

from PyQt5.QtWidgets import QMainWindow, QApplication,QMainWindow, QWidget, QVBoxLayout, QDateTimeEdit, QLabel, QHeaderView
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QPoint, QSettings, QRegExp, QSignalBlocker, pyqtSignal, QAbstractTableModel, QDateTime
from PyQt5.QtGui import QColor, QRegExpValidator, QPainter, QMovie, QPixmap, QPen
from PyQt5 import uic

from libro_ui_ros import RosTopicInfo
import rclpy
from datetime import datetime
from libro_toggle_publisher import LibroTogglePublisher

# UI 파일 연결
main_ui = uic.loadUiType("Libro_main_ui.ui")[0]

# 로그인 정보
ID = "admin"
PASSWORD = "1234"

## 화면을 띄우는데 사용되는 Class 선언
class LibroAdmin(QMainWindow, main_ui): 
    start_dt = QDateTime(2000, 1, 1, 1, 0, 0)
    libro1_toggle_signal = pyqtSignal(bool)
    libro2_toggle_signal = pyqtSignal(bool)
    libro3_toggle_signal = pyqtSignal(bool)
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Libro Admin")
        self.toggle_publisher = None

        # 타이머 선언
        self.timer = QTimer()

        # 초기 화면은 로그인 페이지 (index 0)
        self.stackedWidget.setCurrentIndex(0)

        # ROS Thread 연결
        self.ros_topic = RosTopicInfo()

        rclpy.init(args=None)
        self.ros_node = LibroTogglePublisher()

        
        # QDateTimeEdit에 값 세팅
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
        
        self.ros_topic.libro1_pose_received.connect(self.current_area_update)
        self.ros_topic.libro2_pose_received.connect(self.current_area_update)
        self.ros_topic.libro3_pose_received.connect(self.current_area_update)
        
        self.ros_topic.libro_state_received.connect(self.current_state_update)

        self.ros_topic.libro1_erorr_received.connect(self.current_erorr1_update)
        self.ros_topic.libro2_erorr_received.connect(self.current_erorr2_update)
        self.ros_topic.libro3_erorr_received.connect(self.current_erorr3_update)

        self.ros_topic.libro1_battery_received.connect(self.current_battery1_update)
        self.ros_topic.libro2_battery_received.connect(self.current_battery2_update)
        self.ros_topic.libro3_battery_received.connect(self.current_battery3_update)

        self.ros_topic.libro1_state_received.connect(self.current_libro1_update)
        self.ros_topic.libro2_state_received.connect(self.current_libro2_update)
        self.ros_topic.libro3_state_received.connect(self.current_libro3_update)

        self.libro1_toggle_signal.connect(self.send_toggle1_to_ros)
        self.libro2_toggle_signal.connect(self.send_toggle2_to_ros)      
        self.libro3_toggle_signal.connect(self.send_toggle3_to_ros)  
        self.libro1_activate_sw.toggled.connect(self.libro1_sw_toggled)
        self.libro2_activate_sw.toggled.connect(self.libro2_sw_toggled)
        self.libro3_activate_sw.toggled.connect(self.libro3_sw_toggled)

        self.start_dateTimeEdit.setCalendarPopup(True)
        self.start_dateTimeEdit.setDisplayFormat('yyyy-MM-dd HH:mm:ss')

        self.end_dateTimeEdit.setDateTime(QDateTime.currentDateTime())
        self.end_dateTimeEdit.setCalendarPopup(True)
        self.end_dateTimeEdit.setDisplayFormat('yyyy-MM-dd HH:mm:ss')

        self.today_option_btn.clicked.connect(self.set_today_button)
        self.week1_option_btn.clicked.connect(self.set_week_ago_button)
        self.month1_option_btn.clicked.connect(self.set_month_ago_button)
        self.month3_option_btn.clicked.connect(self.set_3months_ago_button)
        self.log_search_btn.clicked.connect(self.search_log_date)
        
        

        self.time_timer = QTimer(self) #QTimer 객체를 생성
        self.time_timer.timeout.connect(self.update_log_time)
        self.time_timer.start(1000)  # 1초마다 갱신
        
        self.ros_topic.log_list_received.connect(self.robot_log)
        self.category_search.addItems(["시간", "로봇ID", "주문번호", "로봇로그", "성공/실패"])
        self.log_search_btn.clicked.connect(self.filter_log_table)
        self.search_reset_btn.clicked.connect(self.reset_log_search)
        self.log_search_lineEdit.returnPressed.connect(self.filter_log_table)

        self.ros_topic.start()

        # 로그인 버튼 클릭과 로그인 함수 연결
        self.login_btn.clicked.connect(self.check_login)
        # 비밀번호 입력창에서 엔터로 로그인
        self.lineEdit_password.returnPressed.connect(self.login_btn.click)
        
        
        # 로그인 성공한 아이디 저장 기능 
        settings = QSettings("fourfour", "LibroAdmin") # QSettings의 파일 저장 방식
        saved_id = settings.value("saved_id", "") #저장된 아이디를 불러오고, 없다면 공백으로 채움
        self.lineEdit_id.setText(saved_id)

        # 저장된 ID가 있다면 체크박스를 체크상태로 둔다.
        if saved_id:
            self.id_save_check_btn.setChecked(True)

        # tab2: robot_activate_sw 상태 정의
        self.libro1_activate_sw.setChecked(True)
        self.libro2_activate_sw.setChecked(True)
        self.libro3_activate_sw.setChecked(True)

        # 맵 위에 그릴 로봇 좌표 저장 공간, 초기 위치 설정
        self.robot_positions = {
                1: (1010, 20),
                2: (1010, 30),
                3: (1010, 40)
            }
        
        # 퍼블리시 해서 들어오는 로봇의 실제 좌표값을 따라가지 못할까봐, 주기적으로 호출해서 그림
        self.timer.timeout.connect(self.draw_robot_position)
        self.timer.start(33) #33ms 약 30fps

        if self.map_image_lb.pixmap() is not None:
            self.map_image_origin = self.map_image_lb.pixmap().copy()
        else:
            self.map_image_origin = None
        
        self.draw_robot_position() # 시작할 때 한 번은 그려지게 초기 선언


###################################################################################
    # 로그인 확인 - (서버 연결용)
    # def check_login(self):
    #     user_id = self.lineEdit_id.text()
    #     user_pw = self.lineEdit_password.text()
        
    #     if len(user_pw) < 4:
    #         self.login_fail_label.setText("비밀번호는 4자리 이상 입력해주세요.")
    #         self.lineEdit_password.clear()
    #         return

    #     try:
    #         # 서버에서 사용자 리스트 가져오기
    #         login_response = requests.get("http://192.168.0.138:8000/users", timeout= 3) # 서버 미응답 시간 5초로 제한
    #         input_login = login_response.json()

    #         matched_user = None

    #         for u in input_login:
    #             if u["email"] == user_id and u["role"] == "admin": # 관리자만 로그인 할 수 있도록 제한
    #                 matched_user = u
    #                 break  # 첫 번째 일치하는 사용자만 찾으면 끝냄
            
    #         if matched_user:
    #             hashed_pw = matched_user["password"].encode("utf-8")
    #             input_pw = user_pw.encode("utf-8")

    #             # bcrypt로 암호화 된 비밀번호 확인
    #             if bcrypt.checkpw(input_pw, hashed_pw):
                    
    #                 settings = QSettings("fourfour", "LibroAdmin")
    #                 # 체크박스가 눌린 채로 로그인을 성공하게 되면 아이디 저장
    #                 if self.id_save_check_btn.isChecked():
    #                     settings.setValue("saved_id", user_id)
    #                 else:
    #                     settings.remove("saved_id")
        
    #                 self.login_fail_label.setText("")
    #                 self.stackedWidget.setCurrentIndex(1)  # 관리자 페이지로 전환
    #                 return

    #         # 로그인 실패 처리/ 입력창 초기화
    #         self.login_fail_label.setText("아이디 또는 비밀번호가 올바르지 않습니다.")
    #         self.lineEdit_password.clear()
    #         self.lineEdit_id.clear()

    #     except Exception as e:
    #         print("서버 오류:", e)
    #         self.login_fail_label.setText("서버 연결 실패")

###################################################################################
    # ## 로그인 확인 - 아이디 & 비번 코드에서 저장해서 씀.

    def check_login(self):
        user_id = self.lineEdit_id.text()
        user_pw = self.lineEdit_password.text()

        if user_id == ID and user_pw == PASSWORD:

            settings = QSettings("fourfour", "LibroAdmin")
            # 체크박스가 눌린 채로 로그인을 성공하게 되면 아이디 저장
            if self.id_save_check_btn.isChecked():
                settings.setValue("saved_id", user_id)
            else:
                settings.remove("saved_id")

            self.login_fail_label.setText("")
            self.stackedWidget.setCurrentIndex(1)  # 관리자 페이지로 전환
        
        elif len(user_pw) < 4:
            self.login_fail_label.setText("비밀번호는 4자리 이상 입력해주세요.")
            self.lineEdit_password.clear()

        else:
            self.login_fail_label.setText("아이디 또는 비밀번호를 확인해주세요")
            # 아이디와 비밀번호 입력창 초기화
            self.lineEdit_id.clear()
            self.lineEdit_password.clear()

###################################################################################    

    # 실제 로봇이 받는 좌표와 PyQT 맵에서 표현하는 좌표 위치가 달라서 변환 과정을 해주는 함수.
    def robot_to_pyqt_coords(self, x, y):
        img_width = 960 # 이미지 크기
        img_height = 485
        margin = 5      # 테두리를 구성하는 픽셀 크기
        draw_width = img_width - 2 * margin   # PyQT 맵 이미지에서 표현할 범위 950
        draw_height = img_height - 2 * margin # PyQT 맵 이미지에서 표현할 범위 475
        # x축 변환/ 로봇은 최대 2, PyQT 이미지에서는 최대 950, 테두리 5px
        img_x = int(margin + (x / 2) * draw_width)
        # y축 변환/ 로봇은 최대 1, PyQT 이미지에서는 최대 475
        img_y = int(margin + ((1 - y) / 1) * draw_height)
        
        return img_x+5, img_y +18


    # 로봇 실시간 좌표로 위치 업데이트
    def current_area_update(self, robot_id, x, y):
        # print(f"robot_id= {robot_id}, x= {x}, y= {y}") # 로봇 좌표 흔들려서 추가한 코드

        if robot_id == 1:
            x = round(x, 2)
            y = round(y, 2)
            self.libro1_area_lb.setText(f"X: {x}, Y: {y}")
        
        elif robot_id == 2:
            x = round(x, 2)
            y = round(y, 2)
            self.libro2_area_lb.setText(f"X: {x}, Y: {y}")
            # print(x,y)
        
        elif robot_id == 3:
            x = round(x, 2)
            y = round(y, 2)
            self.libro3_area_lb.setText(f"X: {x}, Y: {y}")


        # 좌표 값을 받아서 그림 그릴 저장소에 업데이트
        self.robot_positions[robot_id] = (self.robot_to_pyqt_coords(x, y))

        self.draw_robot_position() #좌표가 토픽으로 들어오면 그리기 함수 실행
    # 맵 위에 픽셀로 로봇 현재 위치 표시
    def draw_robot_position(self):
        # 이미지가 없으면 그림을 그리지 않겠다.
        if self.map_image_origin is None:
            return
        
        # 현재 map_image가 바뀌지 않게 복사해서 사용. 항상 원본에 복사
        pixmap = self.map_image_origin.copy()
        
        painter = QPainter(pixmap)
        color_map = {1: 'blue', 2: 'red', 3: 'green'}
        radius = 17 #그려지는 픽셀 원 반지름
        
        # 로봇 좌표 저장소에서 꺼내서 for문을 돌림.
        for robot_id, (x, y) in self.robot_positions.items():
            pen = QPen(QColor(color_map.get(robot_id, 'yellow')))
            pen.setWidth(1) # 그려지는 원 테두리 두께
            painter.setPen(pen)
            painter.setBrush(QColor(color_map.get(robot_id, 'yellow'))) # 원 내부 색상
            
            # 중심이 (x, y)이고 지름이 2*radius인 원을 그림
            painter.drawEllipse(int(x) - radius, int(y) - radius, 2*radius, 2*radius)
        
        painter.end()
        
        self.map_image_lb.setPixmap(pixmap)

    # state가 영어로 들어오니까 gui에 업데이트 되는 것은 한글로 변경하고 싶음.
    # def translate_state(self, state):
    #     mapping = {
    #         'charging': '충전중',
    #         'waiting': '대기중',
    #         'pickup': '픽업중',
    #         'nav': '길 안내중'
    #     }
    #     return mapping.get(state, state)

    # 로봇 상태 업데이트
    def current_state_update(self, result_list):
        state = {k: v for k, v in result_list}
        self.libro1_state_lb.setText(state.get('libro1', 'none'))
        self.libro1_state_tab2.setText(state.get('libro1', 'none'))
        self.libro2_state_lb.setText(state.get('libro2', 'none'))
        self.libro2_state_tab2.setText(state.get('libro2', 'none'))
        self.libro3_state_lb.setText(state.get('libro3', 'none'))
        self.libro3_state_tab2.setText(state.get('libro3', 'none'))

    # 로봇 에러 업데이트
    def current_erorr1_update(self, message_1):
        self.error_detail_1.setText(message_1)
        self.libro1_error_lb.setText('error')
        blocker = QSignalBlocker(self.libro1_activate_sw)
        self.libro1_activate_sw.setChecked(False)
        self.toggle1 = 1
        self.libro1_toggle_signal.emit(True)

        
    def current_erorr2_update(self, message_2):
        self.error_detail_2.setText(message_2)
        self.libro2_error_lb.setText('error')
        blocker = QSignalBlocker(self.libro1_activate_sw)
        self.libro2_activate_sw.setChecked(False)
        self.toggle2 = 1
        self.libro2_toggle_signal.emit(True)
        

    def current_erorr3_update(self, message_3):
        self.error_detail_3.setText(message_3)
        self.libro3_error_lb.setText('error')
        blocker = QSignalBlocker(self.libro3_activate_sw)
        self.libro3_activate_sw.setChecked(True)
        self.toggle = 1
        self.libro3_toggle_signal.emit(True)


    def libro1_sw_toggled(self, checked1):
        if checked1:
            self.error_detail_1.clear()
            self.libro1_error_lb.setText('Normal')
            self.toggle1 = 0
        else:
            self.toggle1 = 1
            
        self.libro1_toggle_signal.emit(bool(self.toggle1))  # 시그널로 값 전달


    def libro2_sw_toggled(self, checked2):
        if checked2:
            self.error_detail_2.clear()
            self.libro2_error_lb.setText('Normal')
            self.toggle2 = 0
        else:
            self.toggle2 = 1
            
        self.libro2_toggle_signal.emit(bool(self.toggle2))  # 시그널로 값 전달


    def libro3_sw_toggled(self, checked3):
        if checked3:
            self.error_detail_3.clear()
            self.libro3_error_lb.setText('Normal')
            self.toggle3 = 0
        else:
            self.toggle3 = 1
            
        self.libro3_toggle_signal.emit(bool(self.toggle3))  # 시그널로 값 전달

    def send_toggle1_to_ros(self, value):
        self.ros_node.publish_toggle(value, 1)

    def send_toggle2_to_ros(self, value):
        self.ros_node.publish_toggle(value, 2)

    def send_toggle3_to_ros(self, value):
        self.ros_node.publish_toggle(value, 3)

        #로봇 배터리
    def set_battery1_style(self, bar, libro1_battery_info):

        if libro1_battery_info > 20:
            style = """
            QProgressBar {
                border: 2px solid #27548A;
                border-radius: 5px;
                text-align: center;
                background-color: #E0E0E0;
                height: 20px;
            }
            QProgressBar::chunk {
                border-radius: 5px;
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #76c7c0,
                    stop:1 #4CAF50
                );
            }
            """
            bar.setStyleSheet(style)
            bar.setValue(libro1_battery_info)
            bar.setFormat("%p%")
        else:
            style = """
            QProgressBar {      
                border: 2px solid #27548A;
                border-radius: 5px;
                text-align: center;
                background-color: #E0E0E0;
                height: 20px;
            }
            QProgressBar::chunk {
                border-radius: 5px;
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #ff6161,
                    stop:1 #d32f2f
                );
            }
            """
            bar.setStyleSheet(style)
            bar.setValue(libro1_battery_info)
            bar.setFormat("%p%")

    def set_battery2_style(self, bar, libro2_battery_info):

        if libro2_battery_info > 20:
            style = """
            QProgressBar {
                border: 2px solid #27548A;
                border-radius: 5px;
                text-align: center;
                background-color: #E0E0E0;
                height: 20px;
            }
            QProgressBar::chunk {
                border-radius: 5px;
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #76c7c0,
                    stop:1 #4CAF50
                );
            }
            """
            bar.setStyleSheet(style)
            bar.setValue(libro2_battery_info)
            bar.setFormat("%p%")
        else:
            style = """
            QProgressBar {      
                border: 2px solid #27548A;
                border-radius: 5px;
                text-align: center;
                background-color: #E0E0E0;
                height: 20px;
            }
            QProgressBar::chunk {
                border-radius: 5px;
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #ff6161,
                    stop:1 #d32f2f
                );
            }
            """
            bar.setStyleSheet(style)
            bar.setValue(libro2_battery_info)
            bar.setFormat("%p%")

    def set_battery3_style(self, bar, libro3_battery_info):

        if libro3_battery_info > 20:
            style = """
            QProgressBar {
                border: 2px solid #27548A;
                border-radius: 5px;
                text-align: center;
                background-color: #E0E0E0;
                height: 20px;
            }
            QProgressBar::chunk {
                border-radius: 5px;
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #76c7c0,
                    stop:1 #4CAF50
                );
            }
            """
            bar.setStyleSheet(style)
            bar.setValue(libro3_battery_info)
            bar.setFormat("%p%")
        else:
            style = """
            QProgressBar {      
                border: 2px solid #27548A;
                border-radius: 5px;
                text-align: center;
                background-color: #E0E0E0;
                height: 20px;
            }
            QProgressBar::chunk {
                border-radius: 5px;
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:0,
                    stop:0 #ff6161,
                    stop:1 #d32f2f
                );
            }
            """
            bar.setStyleSheet(style)
            bar.setValue(libro3_battery_info)
            bar.setFormat("%p%")

    def current_battery1_update(self, libro1_battery_info):
        self.battery_1.setValue(libro1_battery_info)   #스타일 적용한거 넣기
        self.set_battery1_style(self.battery_1, libro1_battery_info)  #벨류값

    def current_battery2_update(self, libro2_battery_info):
        self.battery_2.setValue(libro2_battery_info)
        self.set_battery2_style(self.battery_2, libro2_battery_info)

    def current_battery3_update(self, libro3_battery_info):
        self.battery_3.setValue(libro3_battery_info)
        self.set_battery3_style(self.battery_3, libro3_battery_info)


    #로봇 상세 상태 업데이트
    #책 픽업

    #hasattr예시
    #obj = Example()
    # print(hasattr(obj, "attribute"))  # 출력: True
    # print(hasattr(obj, "missing"))    # 출력: False

###libro_1 상세
    #책 픽업
    def current_libro1_update(self, status1):

        if hasattr(self, 'prev_status1') and self.prev_status1 == status1:
            return  
        # 이전 상태와 다르면 작업 수행
        self.prev_status1 = status1

        self.libro1_img1_lb.clear()
        self.libro1_img2_lb.clear()
        self.libro1_img3_lb.clear()

        if status1 == '0x00':
            pixmap = QPixmap("image/waiting.png")
            self.libro1_img2_lb.setPixmap(pixmap)

        elif status1 == '0x01':
            pixmap_1 = QPixmap("image/basezone.png")
            pixmap_3 = QPixmap("image/bookshelf.png")
            self.movie = QMovie("image/loading.gif")
            self.movie.setSpeed(800)
            self.libro1_img2_lb.setMovie(self.movie)
            self.movie.start()
            self.libro1_img1_lb.setPixmap(pixmap_1)
            self.libro1_img3_lb.setPixmap(pixmap_3)

        elif status1 in ['0x02','0x03','0x04','0x05','0x06','0x07']:
            pixmap = QPixmap("image/bookshelf_work.png")
            self.libro1_img2_lb.setPixmap(pixmap)

        elif status1 == '0x08':
            pixmap_1 = QPixmap("image/bookshelf.png")
            pixmap_3 = QPixmap("image/pickup_zone.png")
            self.movie = QMovie("image/loading.gif")
            self.movie.setSpeed(800)
            self.libro1_img2_lb.setMovie(self.movie)
            self.movie.start()
            self.libro1_img1_lb.setPixmap(pixmap_1)
            self.libro1_img3_lb.setPixmap(pixmap_3)

        elif status1 in ['0x09','0x0a','0x0b']:
            pixmap = QPixmap("image/pickup.png")
            self.libro1_img2_lb.setPixmap(pixmap)

        # 길안내
        elif status1 == '0x0c':
            pixmap_1 = QPixmap("image/basezone.png")
            pixmap_3 = QPixmap("image/user.png")  
            self.movie = QMovie("image/loading.gif")
            self.movie.setSpeed(800)
            self.libro1_img2_lb.setMovie(self.movie)
            self.movie.start()
            self.libro1_img1_lb.setPixmap(pixmap_1)
            self.libro1_img3_lb.setPixmap(pixmap_3)

        elif status1 == '0x0d':
            pixmap = QPixmap("image/user_check.png")
            self.libro1_img2_lb.setPixmap(pixmap)

        elif status1 == '0x0e':
            pixmap_1 = QPixmap("image/user.png")
            pixmap_3 = QPixmap("image/goal.png")
            self.movie = QMovie("image/loading.gif")
            self.movie.setSpeed(800)
            self.libro1_img2_lb.setMovie(self.movie)
            self.movie.start()
            self.libro1_img1_lb.setPixmap(pixmap_1)
            self.libro1_img3_lb.setPixmap(pixmap_3)

        elif status1 == '0x0f':
            pixmap = QPixmap("image/stop.png")
            self.libro1_img2_lb.setPixmap(pixmap)

####libro_2 상세
        #책 픽업

    def current_libro2_update(self, status2):
        if hasattr(self, 'prev_status2') and self.prev_status2 == status2:
            return  
        # 이전 상태와 다르면 작업 수행
        self.prev_status2 = status2

        self.libro2_img1_lb.clear()
        self.libro2_img2_lb.clear()
        self.libro2_img3_lb.clear()

        if status2 == '0x00':
            pixmap = QPixmap("image/waiting.png")
            self.libro2_img2_lb.setPixmap(pixmap)

        elif status2 == '0x01':
            pixmap_1 = QPixmap("image/basezone.png")
            pixmap_3 = QPixmap("image/bookshelf.png")
            self.movie = QMovie("image/loading.gif")
            self.movie.setSpeed(800)
            self.libro2_img2_lb.setMovie(self.movie)
            self.movie.start()
            self.libro2_img1_lb.setPixmap(pixmap_1)
            self.libro2_img3_lb.setPixmap(pixmap_3)

        elif status2 in ['0x02','0x03','0x04','0x05','0x06','0x07']:
            pixmap = QPixmap("image/bookshelf_work.png")
            self.libro2_img2_lb.setPixmap(pixmap)

        elif status2 == '0x08':
            pixmap_1 = QPixmap("image/bookshelf.png")
            pixmap_3 = QPixmap("image/pickup_zone.png")
            self.movie = QMovie("image/loading.gif")
            self.movie.setSpeed(800)
            self.libro2_img2_lb.setMovie(self.movie)
            self.movie.start()
            self.libro2_img1_lb.setPixmap(pixmap_1)
            self.libro2_img3_lb.setPixmap(pixmap_3)

        elif status2 in ['0x09','0x0a','0x0b']:
            pixmap = QPixmap("image/pickup.png")
            self.libro2_img2_lb.setPixmap(pixmap)

        # 길안내
        elif status2 == '0x0c':
            pixmap_1 = QPixmap("image/basezone.png")
            pixmap_3 = QPixmap("image/user.png")  
            self.movie = QMovie("image/loading.gif")
            self.movie.setSpeed(800)
            self.libro2_img2_lb.setMovie(self.movie)
            self.movie.start()
            self.libro2_img1_lb.setPixmap(pixmap_1)
            self.libro2_img3_lb.setPixmap(pixmap_3)

        elif status2 == '0x0d':
            pixmap = QPixmap("image/user_check.png")
            self.libro2_img2_lb.setPixmap(pixmap)

        elif status2 == '0x0e':
            pixmap_1 = QPixmap("image/user.png")
            pixmap_3 = QPixmap("image/goal.png")
            self.movie = QMovie("image/loading.gif")
            self.movie.setSpeed(800)
            self.libro2_img2_lb.setMovie(self.movie)
            self.movie.start()
            self.libro2_img1_lb.setPixmap(pixmap_1)
            self.libro2_img3_lb.setPixmap(pixmap_3)

        elif status2 == '0x0f':
            pixmap = QPixmap("image/stop.png")
            self.libro2_img2_lb.setPixmap(pixmap)


##libro_3 상세
    #책 픽업
    def current_libro3_update(self, status3):

        if hasattr(self, 'prev_status3') and self.prev_status3 == status3:
            return  
        #이전 상태와 다르면 작업 수행
        self.prev_status3 = status3

        self.libro3_img1_lb.clear()
        self.libro3_img2_lb.clear()
        self.libro3_img3_lb.clear()

        if status3 == '0x00':
            pixmap = QPixmap("image/waiting.png")
            self.libro3_img2_lb.setPixmap(pixmap)

        elif status3 == '0x01':
            pixmap_1 = QPixmap("image/basezone.png")
            pixmap_3 = QPixmap("image/bookshelf.png")
            self.movie = QMovie("image/loading.gif")
            self.movie.setSpeed(800)
            self.libro3_img2_lb.setMovie(self.movie)
            self.movie.start()
            self.libro3_img1_lb.setPixmap(pixmap_1)
            self.libro3_img3_lb.setPixmap(pixmap_3)

        elif status3 in ['0x02','0x03','0x04','0x05','0x06','0x07']:
            pixmap = QPixmap("image/bookshelf_work.png")
            self.libro3_img2_lb.setPixmap(pixmap)

        elif status3 == '0x08':
            pixmap_1 = QPixmap("image/bookshelf.png")
            pixmap_3 = QPixmap("image/pickup_zone.png")
            self.movie = QMovie("image/loading.gif")
            self.movie.setSpeed(800)
            self.libro3_img2_lb.setMovie(self.movie)
            self.movie.start()
            self.libro3_img1_lb.setPixmap(pixmap_1)
            self.libro3_img3_lb.setPixmap(pixmap_3)

        elif status3 in ['0x09','0x0a','0x0b']:
            pixmap = QPixmap("image/pickup.png")
            self.libro3_img2_lb.setPixmap(pixmap)

        # 길안내
        elif status3 == '0x0c':
            pixmap_1 = QPixmap("image/basezone.png")
            pixmap_3 = QPixmap("image/user.png")  
            self.movie = QMovie("image/loading.gif")
            self.movie.setSpeed(800)
            self.libro3_img2_lb.setMovie(self.movie)
            self.movie.start()
            self.libro3_img1_lb.setPixmap(pixmap_1)
            self.libro3_img3_lb.setPixmap(pixmap_3)

        elif status3 == '0x0d':
            pixmap = QPixmap("image/user_check.png")
            self.libro3_img2_lb.setPixmap(pixmap)

        elif status3 == '0x0e':
            pixmap_1 = QPixmap("image/user.png")
            pixmap_3 = QPixmap("image/goal.png")
            self.movie = QMovie("image/loading.gif")
            self.movie.setSpeed(800)
            self.libro3_img2_lb.setMovie(self.movie)
            self.movie.start()
            self.libro3_img1_lb.setPixmap(pixmap_1)
            self.libro3_img3_lb.setPixmap(pixmap_3)

        elif status3 == '0x0f':
            pixmap = QPixmap("image/stop.png")
            self.libro3_img2_lb.setPixmap(pixmap)


    # tab3 Q테이블 뷰 class정의
    def now_time(self):
        now = datetime.now()
        return now
    
    #지금 리스트 2025-05-23T19:59:54.215844값


    #현재 시간
    def update_log_time(self):
        now = datetime.now()
        today = now.strftime("%Y-%m-%d")
        current_time = now.strftime("%H:%M:%S")
        self.today_lb.setText(today)
        self.time_lb.setText(current_time)

    def set_today_button(self):
    # 오늘 날짜의 00:00:00 생성
        today = QDateTime.currentDateTime()
        today_zero = QDateTime(today.date())  # 시간은 00:00:00으로 자동 설정됨
        self.start_dateTimeEdit.setDateTime(today_zero)
        now = datetime.now()
        self.end_dateTimeEdit.setDateTime(now)

    def set_week_ago_button(self):
        today_zero = QDateTime.currentDateTime()
        today_zero = QDateTime(today_zero.date())
        week_ago_zero = today_zero.addDays(-7)
        self.start_dateTimeEdit.setDateTime(week_ago_zero)
        now = datetime.now()
        self.end_dateTimeEdit.setDateTime(now)

    def set_month_ago_button(self):
        today_zero = QDateTime.currentDateTime()
        today_zero = QDateTime(today_zero.date())
        month_ago_zero = today_zero.addMonths(-1)
        self.start_dateTimeEdit.setDateTime(month_ago_zero)
        now = datetime.now()
        self.end_dateTimeEdit.setDateTime(now)

    def set_3months_ago_button(self):
        today_zero = QDateTime.currentDateTime()
        today_zero = QDateTime(today_zero.date())
        three_months_ago_zero = today_zero.addMonths(-3)
        self.start_dateTimeEdit.setDateTime(three_months_ago_zero)
        now = datetime.now()
        self.end_dateTimeEdit.setDateTime(now)

    def reset_log_search(self):
        # 1. 날짜 초기화: 오늘 00:00:00 ~ 현재시간
        today = QDateTime.currentDateTime()
        today_zero = QDateTime(today.date())  # 00:00:00
        reset_date = today_zero.addYears(-3)
        self.start_dateTimeEdit.setDateTime(reset_date)
        self.end_dateTimeEdit.setDateTime(today)

        # 2. 카테고리(콤보박스) 첫 번째 항목으로 초기화
        self.category_search.setCurrentIndex(0)  # 첫 번째("시간")로

        # 3. 검색어(QLineEdit) 초기화
        self.log_search_lineEdit.clear()

        # 4. 테이블에 전체 로그 다시 표시
        headers = ["                                   시간                                 ", "            로봇ID        ", "          주문번호            ", "                                                        로봇로그                                                               ", "    성공/실패   "]
        self.model = LogTableModel(self.new_list, headers)
        self.log_table_view.setModel(self.model)
        self.log_table_view.horizontalHeader().setFixedHeight(40)
        self.log_table_view.resizeColumnsToContents()



#log 테이블 데이터및 스타일 시트
    def robot_log(self, log_list):
        # print(type(log_list))  # 데이터 확인용
        self.new_list = [] 
        headers = ["                                   시간                                 ", "            로봇ID        ", "          주문번호            ", "                                                        로봇로그                                                               ", "    성공/실패   "]

        for row in log_list:
            dt_str = row[0].split('.')[0]  # '2025-05-23T19:59:54' 
            #형태로 자름, 2025-03-09 11:13:46.90984로 바뀜
            # dt_obj = datetime.strptime(row[0], '%Y-%m-%d %H:%M:%S.%f')
            dt_obj = datetime.fromisoformat(row[0])

            new_row = [dt_obj] + row[1:]
            self.new_list.append(new_row)

        self.model = LogTableModel(self.new_list, headers)
        self.log_table_view.setModel(self.model)
        self.log_table_view.horizontalHeader().setFixedHeight(40)  # 원하는 높이(px)로 지정
        self.log_table_view.resizeColumnsToContents()
        layout = QVBoxLayout()
        layout.setAlignment(self.log_table_view, Qt.AlignCenter)

        return self.new_list

    def search_log_date(self):###검색버튼 여기에다 박아버림 씅ㅂ.........
        start_dt = self.start_dateTimeEdit.dateTime().toPyDateTime()
        end_dt = self.end_dateTimeEdit.dateTime().toPyDateTime()
        filtered_list = [
            row for row in self.new_list
            if start_dt <= row[0] <= end_dt
        ]
        headers = ["                                   시간                                    ", "            로봇ID        ", "          주문번호            ", "                                                         로봇로그                                                               ", "    성공/실패   "]
        self.model = LogTableModel(filtered_list, headers)
        self.log_table_view.setModel(self.model)
        self.log_table_view.horizontalHeader().setFixedHeight(40)
        self.log_table_view.resizeColumnsToContents()
        
    def get_log_search_value(self): #log_search_lineEdit에 넣은 텍스트 str

        value = self.log_search_lineEdit.text()
        return value

    def filter_log_table(self):
        # 날짜 필터 값 가져오기
        start_dt = self.start_dateTimeEdit.dateTime().toPyDateTime()
        end_dt = self.end_dateTimeEdit.dateTime().toPyDateTime()
        # 카테고리(컬럼명)와 검색어
        category = self.category_search.currentText()
        keyword = self.log_search_lineEdit.text().strip()
        headers = ["                                    시간                                    ", "            로봇ID        ", "          주문번호            ", "                                                        로봇로그                                                               ", "    성공/실패   "]
        if category not in headers:
            return
        col_idx = headers.index(category)
        # 두 조건 모두 반영해서 필터링
        filtered = []
        for row in self.new_list:
            # 1) 날짜 범위 조건
            if not (start_dt <= row[0] <= end_dt):
                continue
            # 2) 카테고리/검색어 조건
            cell_value = row[col_idx]
            if isinstance(cell_value, datetime):
                cell_value = cell_value.strftime("%Y-%m-%d %H:%M:%S")
            if keyword == "" or keyword.lower() in str(cell_value).lower():
                filtered.append(row)
        # 결과 테이블에 표시
        self.model = LogTableModel(filtered, headers)
        self.log_table_view.setModel(self.model)
        self.log_table_view.horizontalHeader().setFixedHeight(40)
        self.log_table_view.resizeColumnsToContents()
        self.log_table_view.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        # header = self.log_table_view.horizontalHeader()
        # header.setSectionResizeMode(QHeaderView.Stretch)
        

class LogTableModel(QAbstractTableModel): #큐테이블 뷰
    def __init__(self, data, headers, parent=None):
        super().__init__(parent)
        self._data = data
        self._headers = headers

    def rowCount(self, parent=None):
        return len(self._data)

    def columnCount(self, parent=None):
        return len(self._headers)

    def data(self, index, role=Qt.DisplayRole):
        if not index.isValid():
            return None
        if role == Qt.DisplayRole:
            return str(self._data[index.row()][index.column()])
        return None

    def headerData(self, section, orientation, role=Qt.DisplayRole):
        if role == Qt.DisplayRole and orientation == Qt.Horizontal:
            return self._headers[section]
        return None
    

if __name__ == "__main__":
    # QApplication : 프로그램을 실행시켜주는 클래스
    app = QApplication(sys.argv)
    
    # Window Class의 인스턴스 생성
    window = LibroAdmin()
    
    # 프로그램 화면을 보여주는 코드
    window.show()
    
    # 프로그램을 이벤트 루프로 진입시키는(프로그램을 작동 시키는) 코드
    sys.exit(app.exec())
