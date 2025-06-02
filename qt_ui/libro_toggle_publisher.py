import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class LibroTogglePublisher(Node):
    def __init__(self):
        super().__init__('libro_toggle_publisher')
        self.publisher_1 = self.create_publisher(Bool, '/libro1/stop', 10)
        self.publisher_2 = self.create_publisher(Bool, '/libro2/stop', 10)
        self.publisher_3 = self.create_publisher(Bool, '/libro3/stop', 10)


    def publish_toggle(self, value: bool, num):
        msg = Bool()
        msg.data = value
        if num == 1:
            self.publisher_1.publish(msg)
        elif num ==2:
            self.publisher_2.publish(msg)
        else:
            self.publisher_3.publish(msg)