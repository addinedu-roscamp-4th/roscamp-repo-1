import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from libro_mobile_srvs.srv import SetGoalPose

class GoalPublisherNode(Node):

    def __init__(self):
        super().__init__('goal_publisher_node')
        # Create publisher for goal pose
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
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
        
        # Create timer to publish current goal pose
        self.timer = self.create_timer(1.0, self.publish_goal_pose)
        
        self.get_logger().info('Goal Publisher Node has been started.')
    
    def set_goal_pose_callback(self, request, response):
        self.get_logger().info('Received new goal pose request')
        
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