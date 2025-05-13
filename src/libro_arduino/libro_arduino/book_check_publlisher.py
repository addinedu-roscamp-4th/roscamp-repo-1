import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
import os

class BookCheckPublisher(Node):
    def __init__(self):
        super().__init__('book_check_basket_publisher')
        self.publisher_ = self.create_publisher(String, 'book_check_basket', 10)
        timer_period = 0.5
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
            self.get_logger().info(f'Book check: "{msg.data}"')
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