import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from libro_mobile_srvs.srv import SetGoalPose
import math
class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('goal_publisher_node')
        # Create publisher for goal pose
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # Create subscriber for pinky_mission_state
        self.mission_state_subscriber_ = self.create_subscription(
            Int32,
            '/pinky_mission_state',
            self.mission_state_callback,
            10)
         # Create subscriber for battery
        self.battery_subscriber_ = self.create_subscription(
            Float32,
            '/pinky_battery_present',
            self.battery_callback,
            10
        )
        # Create subscriber for tracked pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/tracked_pose',
            self.tracked_pose_callback,
            10)
        # Create service for setting goal pose
        self.srv = self.create_service(
            SetGoalPose,
            'set_goal_pose',
            self.set_goal_pose_callback
        )
        # Initialize current goal pose: 초기위치 설정
        self.current_goal_pose = PoseStamped()
        self.current_goal_pose.header.frame_id = 'map'
        self.current_goal_pose.pose.position.x = 0.01
        self.current_goal_pose.pose.position.y = 0.01
        self.current_goal_pose.pose.orientation.w = 0.999
        self.current_goal_pose.pose.orientation.z = 0.001
        # 현재 충전 중인지 여부
        self.charging = False
        # Flag to control goal publishing
        self.goal_reached = False
        # Position tolerance (in meters)
        self.position_tolerance = 0.3
        # Orientation tolerance (in radians)
        self.orientation_tolerance = 0.2  # 약 11.4도
        # Create timer to publish current goal pose
        self.timer = self.create_timer(1.0, self.publish_goal_pose)
        self.get_logger().info('Goal Publisher Node has been started.')
        # 배터리 기준과 상태
        self.low_battery_threshold = 60.0
        self.battery_low = False
    def calculate_distance(self, pose1, pose2):
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx*dx + dy*dy)
    def calculate_orientation_error(self, pose1, pose2):
        # Calculate the angle between two quaternions
        q1 = pose1.orientation
        q2 = pose2.orientation
        # Calculate dot product
        dot_product = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z
        # Ensure dot product is within valid range
        dot_product = max(-1.0, min(1.0, dot_product))
        # Calculate angle in radians
        angle = 2.0 * math.acos(abs(dot_product))
        return angle
    def tracked_pose_callback(self, msg):
        if not self.goal_reached:
            position_error = self.calculate_distance(msg.pose, self.current_goal_pose.pose)
            orientation_error = self.calculate_orientation_error(msg.pose, self.current_goal_pose.pose)
            #self.get_logger().info(f'Position error: {position_error:.3f}m, Orientation error: {math.degrees(orientation_error):.1f}°')
            if position_error < self.position_tolerance and orientation_error < self.orientation_tolerance:
                self.goal_reached = True
                self.get_logger().info('Goal reached! Stopping goal publishing.')
    def set_goal_pose_callback(self, request, response):
        if self.charging:
            response.success = False
            response.message = 'Currently navigating to charging station. Goal pose ignored.' # 충전 중인 경우 이동 거부
            return response
        self.get_logger().info('Received new goal pose request')
        self.goal_reached = False
        self.current_goal_pose.pose.position.x = request.x
        self.current_goal_pose.pose.position.y = request.y
        self.current_goal_pose.pose.orientation.w = request.w
        self.current_goal_pose.pose.orientation.z = request.z
        self.publisher_.publish(self.current_goal_pose)
        response.success = True
        response.message = f'Goal pose updated to x={request.x}, y={request.y}, w={request.w}, z={request.z}'
        return response
    def mission_state_callback(self, msg):
        mission_state = msg.data
        self.get_logger().info(f'Mission state: {mission_state}')
        # Always check conditions when mission_state OR battery status changes
        self.try_send_to_charging_station(mission_state)
    def battery_callback(self, msg):
        battery_percentage = msg.data
        self.get_logger().info(f'Battery percentage: {battery_percentage:.2f}%')
        self.battery_low = battery_percentage < self.low_battery_threshold
        # Also check here
        self.try_send_to_charging_station(self.last_mission_state)
    def try_send_to_charging_station(self, mission_state):
        self.last_mission_state = mission_state  # store last state
        if mission_state == 0 and self.battery_low and not self.charging:
            self.get_logger().info('Low battery and mission idle. Sending robot to charging station.')
            self.goal_reached = False
            self.charging = True
            self.current_goal_pose.pose.position.x = 0.7
            self.current_goal_pose.pose.position.y = -0.3
            self.current_goal_pose.pose.orientation.w = 0.707
            self.current_goal_pose.pose.orientation.z = 0.707
            self.publisher_.publish(self.current_goal_pose)
        elif mission_state != 0 or not self.battery_low:
            self.charging = False
    def publish_goal_pose(self):
        if not self.goal_reached:
            self.publisher_.publish(self.current_goal_pose)
            self.get_logger().info(f'Publishing goal pose: x={self.current_goal_pose.pose.position.x}, y={self.current_goal_pose.pose.position.y}')
def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()





