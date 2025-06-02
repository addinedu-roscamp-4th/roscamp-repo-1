import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import os

class BasketCheckPublisher(Node):
    def __init__(self):
        super().__init__('book_check_basket_publisher')
        self.declare_parameter('robot_id', 'libro1')  # 기본값으로 libro1 설정 / libro2 , libro3 .....
        self.robot_id = self.get_parameter('robot_id').value
        self.empty_basket_pub = self.create_publisher(String, f'{self.robot_id}/empty_basket_info', 10)
        self.check_basket_pub = self.create_publisher(String, f'{self.robot_id}/check_basket_info', 10)
        self.empty_basket_sub = self.create_subscription(String, f'{self.robot_id}/empty_basket', self.check_empty_basket_callback, 10)
        self.check_basket_sub = self.create_subscription(String, f'{self.robot_id}/check_basket', self.double_check_basket_callback, 10)

        timer_period = 0.02  # 20ms로 변경 (더 빠른 주기)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Get parameter values with defaults
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)

        arduino_port = self.get_parameter('arduino_port').value
        baud_rate = self.get_parameter('baud_rate').value

        # 현재 바구니 상태 저장 변수
        self.current_status = ""

        try:
            self.ser = serial.Serial(arduino_port, baud_rate, timeout=0)  # timeout=0으로 설정
            time.sleep(3)  # Wait for Arduino to initialize
            self.ser.reset_input_buffer()  # 버퍼 초기화
            self.get_logger().info(f'Connected to Arduino on port {arduino_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            raise

    def timer_callback(self):
        try:
            if self.ser.in_waiting:
                raw_data = self.ser.readline()
                try:
                    status = raw_data.decode('utf-8').strip()
                    # 완전한 데이터 라인인지 확인
                    if status and ("B1 :" in status and "B2 :" in status):
                        self.current_status = status
                        self.get_logger().info(f'Book check: "{status}"')
                except UnicodeDecodeError:
                    self.get_logger().warn('Received invalid data from Arduino, skipping...')
        except serial.SerialException as e:
            self.get_logger().error(f'Error reading from Arduino: {str(e)}')

    def check_empty_basket_callback(self, msg):
        try:
            if self.current_status:  # Use the latest status
                if "B1 : OFF" in self.current_status and "B2 : OFF" in self.current_status:
                    response = String()
                    response.data = "B1"
                    self.empty_basket_pub.publish(response)
                    self.get_logger().info('Both baskets are empty, publishing B1')
                elif "B1 : OFF" in self.current_status:
                    response = String()
                    response.data = "B1"
                    self.empty_basket_pub.publish(response)
                    self.get_logger().info('Basket 1 is empty, publishing B1')
                elif "B2 : OFF" in self.current_status:
                    response = String()
                    response.data = "B2"
                    self.empty_basket_pub.publish(response)
                    self.get_logger().info('Basket 2 is empty, publishing B2')
        except Exception as e:
            self.get_logger().error(f'Error in check_empty_basket_callback: {str(e)}')

    def double_check_basket_callback(self, msg):
        try:
            basket_to_check = msg.data  # B1 or B2
            if self.current_status:  # Use the latest status
                response = String()
                if basket_to_check == "B1":
                    if "B1 : ON" in self.current_status:
                        response.data = "ON"
                    else:
                        response.data = "OFF"
                elif basket_to_check == "B2":
                    if "B2 : ON" in self.current_status:
                        response.data = "ON"
                    else:
                        response.data = "OFF"
                
                self.check_basket_pub.publish(response)
                self.get_logger().info(f'Basket {basket_to_check} status: {response.data}')
        except Exception as e:
            self.get_logger().error(f'Error in double_check_basket_callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    try:
        book_check_basket_publisher = BasketCheckPublisher()
        rclpy.spin(book_check_basket_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {str(e)}')
    finally:
        if 'book_check_basket_publisher' in locals():
            book_check_basket_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()