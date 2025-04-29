import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from libro_mobile_srvs.srv import SetGoalPose
import math

class GoalPublisherNode(Node):

    def __init__(self):
        super().__init__('goal_publisher_node')
        # Create publisher for goal pose
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
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
        
        # Initialize current goal pose
        self.current_goal_pose = PoseStamped()
        self.current_goal_pose.header.frame_id = 'map'
        self.current_goal_pose.pose.position.x = 0.5
        self.current_goal_pose.pose.position.y = 0.1
        self.current_goal_pose.pose.orientation.w = 0.707
        self.current_goal_pose.pose.orientation.z = 0.707
        
        # Flag to control goal publishing
        self.goal_reached = False
        
        # Position tolerance (in meters)
        self.position_tolerance = 0.3
        
        # Orientation tolerance (in radians)
        self.orientation_tolerance = 0.2  # 약 11.4도
        
        # Create timer to publish current goal pose
        self.timer = self.create_timer(1.0, self.publish_goal_pose)
        
        self.get_logger().info('Goal Publisher Node has been started.')
    
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
            
            self.get_logger().info(f'Position error: {position_error:.3f}m, Orientation error: {math.degrees(orientation_error):.1f}°')
            
            if position_error < self.position_tolerance and orientation_error < self.orientation_tolerance:
                self.goal_reached = True
                self.get_logger().info('Goal reached! Stopping goal publishing.')
    
    def set_goal_pose_callback(self, request, response):
        self.get_logger().info('Received new goal pose request')
        
        # Reset goal reached flag when new goal is set
        self.goal_reached = False
        
        # Update current goal pose with new values
        self.current_goal_pose.pose.position.x = request.x
        self.current_goal_pose.pose.position.y = request.y
        self.current_goal_pose.pose.orientation.w = request.w
        self.current_goal_pose.pose.orientation.z = request.z
        
        # Publish the new goal pose immediately
        self.publisher_.publish(self.current_goal_pose)
        
        response.success = True
        response.message = f'Goal pose updated to x={request.x}, y={request.y}, w={request.w}, z={request.z}'
        return response
    
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
