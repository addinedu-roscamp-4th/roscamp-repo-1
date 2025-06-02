"""

StateMachine으로 구현된 PID controller 노드 


"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64, String,Bool
from tf2_msgs.msg import TFMessage 
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
import math
from geometry_msgs.msg import PoseStamped
from navigation.pid import PID
from rclpy.qos import QoSProfile, ReliabilityPolicy
from libro_control_msgs.msg import PinkyResponse
from libro_control_msgs.msg import PinkyRequest
from rclpy.duration import Duration

# 받아오는 속도조절
qos = QoSProfile(depth=1)
qos.reliability = ReliabilityPolicy.BEST_EFFORT


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle)) # 정규화

class PIDController(Node):
    def __init__(self):
        super().__init__('PID_controller')

        self.namespace = self.get_namespace()
        self.aruco_map = f"aruco{self.namespace[6]}/map"
        
        # 오차 허용범위
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('distance_tolerance', 0.01)
        
        # PID 계수 설정
        self.declare_parameter('angular_P', 0.01)
        self.declare_parameter('angular_I', 0.0)
        self.declare_parameter('angular_D', 0.0)
        self.declare_parameter('angular_max_state', 0.03)
        self.declare_parameter('angular_min_state', -0.03)
        
        self.declare_parameter('linear_P', 0.08)
        self.declare_parameter('linear_I', 0.0)
        self.declare_parameter('linear_D', 0.0)
        self.declare_parameter('linear_max_state', 0.08)
        self.declare_parameter('linear_min_state', -0.08)
        
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value

        # Angular PID 초기화
        angular_P = self.get_parameter('angular_P').value
        angular_I = self.get_parameter('angular_I').value
        angular_D = self.get_parameter('angular_D').value
        angular_max_state = self.get_parameter('angular_max_state').value
        angular_min_state = self.get_parameter('angular_min_state').value
        self.angular_pid = PID()
        self.angular_pid.P = angular_P
        self.angular_pid.I = angular_I
        self.angular_pid.D = angular_D
        self.angular_pid.max_state = angular_max_state
        self.angular_pid.min_state = angular_min_state
        
        # Linear PID 초기화
        linear_P = self.get_parameter('linear_P').value
        linear_I = self.get_parameter('linear_I').value
        linear_D = self.get_parameter('linear_D').value
        linear_max_state = self.get_parameter('linear_max_state').value
        linear_min_state = self.get_parameter('linear_min_state').value
        self.linear_pid = PID()
        self.linear_pid.P = linear_P
        self.linear_pid.I = linear_I
        self.linear_pid.D = linear_D
        self.linear_pid.max_state = linear_max_state
        self.linear_pid.min_state = linear_min_state


        # 마지막 처리 시간을 저장할 변수 (rclpy.time.Time 객체)
        self.last_processed_time = self.get_clock().now()
        self.i = 0
        # 최소 처리 간격 설정 (예: 0.1초마다 처리, 즉 10Hz)
        self.min_process_interval = Duration(seconds=0.1)

        # 상태 변수 설정: 초기 상태는 "idle"
        self.state = "idle"
        self.goal_theta = 0.0

        # /Path에 저장된 경로 리스트
        self.path_poses = []   # 전체 Path 리스트
        self.goal_index = 0    # 현재 따라갈 index

        # Subscriber 생성
        self.pose_subscriber = self.create_subscription(TFMessage,'/tf', self.pose_callback, qos)  # 현재 로봇의 위치
        self.goal_pose_subscriber = self.create_subscription(Path,f'{self.namespace}/planned_path',self.goal_pose_callback,10)  # 따라갈 경로
        self.create_subscription(PinkyRequest, f'{self.namespace}/goal_place', self.goal_callback, 10) # 목표위치

        ###################### 장애물 감지 ########################################
        self.obstacle_detected = False # 기본값=장애물 없음
        self.create_subscription(Bool, f'{self.namespace}/obstacle_detected', self.obstacle_callback, 10) # 장애물 여부 판단 노드 구독

        ###################### 강제종료 #########################################
        self.emergency = False
        self.create_subscription(Bool, f'{self.namespace}/stop_mobile', self.stop_callback, 10) # 강제종료 토픽 구독

        # Publisher 생성
        self.cmd_vel_publisher = self.create_publisher(Twist, f'{self.namespace}/cmd_vel', 10) # 선속도와 각속도 퍼블리시
        self.error_publisher = self.create_publisher(Float64, f'{self.namespace}/error', 10)
        self.next_goal_publisher = self.create_publisher(PoseStamped, f'{self.namespace}/next_goal', 10) 
        self.current_publisher = self.create_publisher(PoseStamped, f'{self.namespace}/current', 10) 
        self.goal_response_pub = self.create_publisher(PinkyResponse, f'{self.namespace}/goal_info',10)
        
        # 상태 publisher 추가
        self.state_publisher = self.create_publisher(String, f'{self.namespace}/helper_state', 10)

        # 파라미터 동적 재구성을 위한 콜백 등록
        self.add_on_set_parameters_callback(self.parameter_callback)


    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_publisher.publish(msg)


    ################### 장애물 감지 코드 #####################################
    def obstacle_callback(self, msg: Bool):
        """
        장애물 감지 시 obstacle avoidance 상태로 전환합니다
        """
        self.obstacle_detected = msg.data
        if self.obstacle_detected:
            self.state="obstacle_avoidance"
            self.publish_state()

    ################ 강제종료 코드 #########################################
    def stop_callback(self, msg: Bool):
        """
        강제종료시 종료상태로 전환합니다
        """
        self.emergency = msg.data
        if self.emergency:
            self.state="kill"
            self.publish_state()


    def goal_callback(self,msg):
        self.goal_theta = msg.theta

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'angle_tolerance':
                self.angle_tolerance = param.value
                self.get_logger().info(f"Updated angle_tolerance: {param.value}")
            elif param.name == 'distance_tolerance':
                self.distance_tolerance = param.value
                self.get_logger().info(f"Updated distance_tolerance: {param.value}")
            elif param.name == 'angular_P':
                self.angular_pid.P = param.value
                self.get_logger().info(f"Updated angular_PID P: {param.value}")
            elif param.name == 'angular_I':
                self.angular_pid.I = param.value
                self.get_logger().info(f"Updated angular_PID I: {param.value}")
            elif param.name == 'angular_D':
                self.angular_pid.D = param.value
                self.get_logger().info(f"Updated angular_PID D: {param.value}")
            elif param.name == 'angular_max_state':
                self.angular_pid.max_state = param.value
                self.get_logger().info(f"Updated angular_PID max_state: {param.value}")
            elif param.name == 'angular_min_state':
                self.angular_pid.min_state = param.value
                self.get_logger().info(f"Updated angular_PID min_state: {param.value}")
            elif param.name == 'linear_P':
                self.linear_pid.P = param.value
                self.get_logger().info(f"Updated linear_PID P: {param.value}")
            elif param.name == 'linear_I':
                self.linear_pid.I = param.value
                self.get_logger().info(f"Updated linear_PID I: {param.value}")
            elif param.name == 'linear_D':
                self.linear_pid.D = param.value
                self.get_logger().info(f"Updated linear_PID D: {param.value}")
            elif param.name == 'linear_max_state':
                self.linear_pid.max_state = param.value
                self.get_logger().info(f"Updated linear_PID max_state: {param.value}")
            elif param.name == 'linear_min_state':
                self.linear_pid.min_state = param.value
                self.get_logger().info(f"Updated linear_PID min_state: {param.value}")
        return SetParametersResult(successful=True)
    
    def goal_pose_callback(self, msg):
        """
        목표경로를 받고 상태를 'rotate to goal'로 전환합니다
        """
        if msg.poses:
            self.path_poses = msg.poses
            self.goal_index = 0

            self.state = "rotate_to_goal"
            self.publish_state()
            self.get_logger().info(f"Received new path with {len(self.path_poses)} poses.")

        if not msg.poses:
            self.get_logger().warn("Received empty path.")
            return
        
    def pose_callback(self, msg):
        """
        arucoX/map를 기준으로 로봇의 현재 pose를 계산합니다
        """

        # 현재 시간을 rclpy.time.Time 객체로 가져옴
        current_time = self.get_clock().now()

        if not self.path_poses:
            return
        # 마지막 처리 시간으로부터 최소 처리 간격 이상 지났는지 확인
        if (current_time - self.last_processed_time) >= self.min_process_interval:
            # self.get_logger().info(current_time - self.last_processed_time)
            for transform in msg.transforms:
                if transform.child_frame_id == self.aruco_map: 
                    # self.get_logger().info(f"{transform.child_frame_id}")
                    x = transform.transform.translation.x
                    y = transform.transform.translation.y
                    # Quaternion을 Yaw(θ)로 변환
                    q = transform.transform.rotation
                    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
                    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
                    theta = math.atan2(siny_cosp, cosy_cosp)

                    class Pose:
                        pass
                    current_pose = Pose()
                    current_pose.x = x
                    current_pose.y = y
                    current_pose.theta = theta

                    
                    # 처리 완료 후 마지막 처리 시간 업데이트
                    self.last_processed_time = current_time

                    self.control(current_pose) 
                    break
                
    def control(self, current_pose):
        """
        상태에 따라 다른 함수로 넘겨주고 속도를 받아 퍼블리시 합니다
        """
        twist_msg = Twist()
        
        if self.state == "rotate_to_goal":
            twist_msg = self.handle_rotate_to_goal(current_pose)
        elif self.state == "move_to_goal":
            twist_msg = self.handle_move_to_goal(current_pose)
        elif self.state == "rotate_to_final":
            twist_msg = self.handle_rotate_to_final(current_pose)
        elif self.state == "goal_reached":
            twist_msg = self.handle_goal_reached()
        elif self.state == "obstacle_avoidance":
            twist_msg = self.handle_obstacle(current_pose)
        elif self.state == "pause_after_rotate":
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.state = "rotate_to_goal"
        elif self.state == "kill":
            twist_msg = self.handle_emergency(current_pose)
        
        
        self.cmd_vel_publisher.publish(twist_msg)
    
    def handle_rotate_to_goal(self, current_pose):
        """
        다음 목표지점을 향해 방향을 정렬합니다
        """
        twist_msg = Twist()
        error_msg = Float64()

        # print("rotate to goal")

        goal = self.path_poses[self.goal_index].pose

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map" 
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose = goal  # 기존 pose 넣기

        self.next_goal_publisher.publish(pose_msg)

        # print(goal.position.x)
        
        desired_heading = math.atan2(goal.position.y - current_pose.y,
                                     goal.position.x - current_pose.x)
        error_angle = normalize_angle(desired_heading - current_pose.theta) # 라디안 값을 정규화

        error_msg.data = error_angle
        self.error_publisher.publish(error_msg)
        
        if abs(error_angle) > self.angle_tolerance: # 오차각도가 허용오차 범위 이상이면
            angular_correction = self.angular_pid.update(error_angle)
            twist_msg.angular.z = angular_correction
            twist_msg.linear.x = 0.0

            self.state = "pause_after_rotate"
        else:                                        # 방향오차 보정 완료 됐으면
            # self.get_logger().info(f"final angle error :{error_angle}")
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.state = "move_to_goal"  # 상태 전환
            self.get_logger().info("Heading aligned. Switching to move_to_goal state.")
            self.publish_state()

        return twist_msg

    def handle_move_to_goal(self, current_pose):
        """
        다음 목표지점으로 전진합니다
        """
        # print("moving to goal")
        twist_msg = Twist()
        error_msg = Float64()
        
        goal = self.path_poses[self.goal_index].pose

        # pose_msg = PoseStamped()
        # pose_msg.header.frame_id = "map"  # 또는 적절한 프레임 이름
        # pose_msg.header.stamp = self.get_clock().now().to_msg()
        # pose_msg.pose = goal  # 기존 pose 넣기

        # self.next_goal_publisher.publish(pose_msg)
        
        dx = goal.position.x - current_pose.x
        dy = goal.position.y - current_pose.y
        distance_error = dx * math.cos(current_pose.theta) + dy * math.sin(current_pose.theta)
        error_msg.data = distance_error
        self.error_publisher.publish(error_msg)
        
        if abs(distance_error) > self.distance_tolerance:  # 거리오차가 허용오차 범위 이상이면
            linear_correction = self.linear_pid.update(distance_error)
            twist_msg.linear.x = linear_correction
            twist_msg.angular.z = 0.0 
        else:                                             # 거리오차 보정 완료 됐으면
            # print("moving to next goal")
            self.goal_index += 1                          # 다음 목표지점으로 넘어간다
            if self.goal_index < len(self.path_poses):
                self.state = "rotate_to_goal"   # 상태전환
            else:                                         # 다음 목표지점이 최종위치라면         
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.state = "rotate_to_final"  # 상태전환
            self.publish_state()
            
        # print(f'dist error :{distance_error}')
        # print(f'move :{twist_msg.linear.x}')

        return twist_msg

    def handle_rotate_to_final(self, current_pose):
        """
        최종도착지에서 방향을 정렬합니다
        """
        # print("final rotate end")
        twist_msg = Twist()
        error_msg = Float64()
        
        # goal = self.path_poses[-1].pose  # 최종 도착지점

        # 쿼터니언 값 변환
        # q = goal.orientation
        # siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        # cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        # yaw = math.atan2(siny_cosp, cosy_cosp)

        # 오차계산
        final_error = normalize_angle(self.goal_theta - current_pose.theta)
        error_msg.data = final_error
        self.error_publisher.publish(error_msg)
        
        if abs(final_error) > self.angle_tolerance:        # 방향오차가 허용오차 범위 이상이면 
            angular_correction = self.angular_pid.update(final_error)  
            twist_msg.angular.z = angular_correction        # 방향오차 보정
            twist_msg.linear.x = 0.0
        else:
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0
            self.state = "goal_reached"   # 상태 전환
            self.get_logger().info("Final orientation reached. Goal achieved.")
            self.publish_state()
        
        return twist_msg

    def handle_goal_reached(self):
        """
        목표지점 도달 후 정지합니다
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        # 미션성공 발행
        response_msg = PinkyResponse()
        response_msg.success = True
        response_msg.message = "Goal reached successfully!"  # 또는 실패 시 에러 메시지
        self.goal_response_pub.publish(response_msg)

        self.path_poses = []  # reset
        self.state="idle" # 대기상태로 전환
        self.publish_state()
        self.get_logger().info("Goal completed. Waiting for new goal.")
        return twist_msg
    
    ######################### 장애물 감지 코드 #########################################
    def handle_obstacle(self, current_pose):
        """
        장애물 감지시 정지 혹은 우회합니다
        """
        twist_msg = Twist()

        # 장애물이 감지되었을 때 로봇을 천천히 회피 혹은 정지
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        self.get_logger().warn("Obstacle detected. Switching to obstacle_avoidance state.")

        # 상태가 변경되지 않았다면, 로그 출력 및 상태 publish
        if self.state != "obstacle_avoidance":
            self.state = "obstacle_avoidance"
            self.publish_state()
            
        # 장애물이 사라졌는지 확인
        if not self.obstacle_detected:
            self.get_logger().info("Obstacle cleared. Resuming path tracking.")
            # 장애물이 사라졌으면 이전 상태로 복귀
            # 남은 path가 있다면 진행, 아니면 goal 도달
            if self.goal_index < len(self.path_poses):
                self.state = "rotate_to_goal"
            else:
                self.state = "goal_reached"
            self.publish_state()

        return twist_msg
    
    ####################### 강제종료 코드 ##########################################
    def handle_emergency(self,current_pose):
        """
        강제종료 합니다
        """
        twist_msg = Twist()

        # 강제종료 시 멈춤
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0

        # 상태가 변경되지 않았다면, 로그 출력 및 상태 publish
        if self.state != "kill":
            self.state = "kill"
            self.publish_state()
            
        self.get_logger().warn("Emergency Stop. SHUTDOWN")
        # 강제종료 후에는 다른 상태로 돌아가지 않음
        return twist_msg
    

def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()