import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import os

class BookCheckPublisher(Node):
    def __init__(self):
        super().__init__('book_check_basket_publisher')
        self.declare_parameter('robot_id', 'libro1')  # 기본값으로 libro1 설정 / libro2 , libro3 .....
        self.robot_id = self.get_parameter('robot_id').value
        self.empty_basket_pub = self.create_publisher(String, f'{self.robot_id}/empty_basket_info', 10)
        self.check_basket_pub = self.create_publisher(String, f'{self.robot_id}/check_basket_info', 10)
        self.empty_basket_sub = self.create_subscription(String, f'{self.robot_id}/empty_basket', self.check_empty_basket_callback, 10)
        self.check_basket_sub = self.create_subscription(String, f'{self.robot_id}/check_basket', self.double_check_basket_callback, 10)

        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Get parameter values with defaults
        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)

        arduino_port = self.get_parameter('arduino_port').value
        baud_rate = self.get_parameter('baud_rate').value

        try:
            self.ser = serial.Serial(arduino_port, baud_rate)
            time.sleep(3)  # Wait for Arduino to initialize
            self.get_logger().info(f'Connected to Arduino on port {arduino_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            raise

    def timer_callback(self):
        try:
            msg = String()
            msg.data = self.ser.readline().decode().strip()
            self.publisher_.publish(msg)
            self.last_check = msg.data  # 최근 값 저장
            self.get_logger().info(f'Book check: "{msg.data}"')
        except serial.SerialException as e:
            self.get_logger().error(f'Error reading from Arduino: {str(e)}')

    def check_empty_basket_callback(self, msg):
        try:
            current_status = self.ser.readline().decode().strip()
            if "B1 : OFF" in current_status and "B2 : OFF" in current_status:
                response = String()
                response.data = "B1"
                self.empty_basket_pub.publish(response)
                self.get_logger().info('Both baskets are empty, publishing B1')
            elif "B1 : OFF" in current_status:
                response = String()
                response.data = "B1"
                self.empty_basket_pub.publish(response)
                self.get_logger().info('Basket 1 is empty, publishing B1')
            elif "B2 : OFF" in current_status:
                response = String()
                response.data = "B2"
                self.empty_basket_pub.publish(response)
                self.get_logger().info('Basket 2 is empty, publishing B2')
        except serial.SerialException as e:
            self.get_logger().error(f'Error reading from Arduino: {str(e)}')

    def double_check_basket_callback(self, msg):
        try:
            basket_to_check = msg.data  # B1 or B2
            current_status = self.ser.readline().decode().strip()
            
            response = String()
            if basket_to_check == "B1":
                if "B1 : ON" in current_status:
                    response.data = "ON"
                else:
                    response.data = "OFF"
            elif basket_to_check == "B2":
                if "B2 : ON" in current_status:
                    response.data = "ON"
                else:
                    response.data = "OFF"
            
            self.check_basket_pub.publish(response)
            self.get_logger().info(f'Basket {basket_to_check} status: {response.data}')
        except serial.SerialException as e:
            self.get_logger().error(f'Error reading from Arduino: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    try:
        book_check_basket_publisher = BookCheckPublisher()
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