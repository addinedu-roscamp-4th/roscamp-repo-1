"""

A* 알고리즘 기반 Global Planner 노드
  - 목표지점(/goal)을 받아 생성한 경로를 /planned_path로 발행합니다

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from navigation.map_drawer import MapDrawer
from navigation.astar import Astar
from navigation.map_loader import MapLoader
from libro_control_msgs.msg import PinkyRequest
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # 맵 정보 받아오기
        self.pgm_path = "./src/navigation/map/libro_map.png"
        self.yaml_path = "./src/navigation/map/libro_map.yaml"
        self.map_loader = MapLoader(self.pgm_path, self.yaml_path)
        self.grid, self.resolution, self.origin, self.height, self.width = self.map_loader.load_map()
        self.namespace = self.get_namespace()
        self.aruco_map = f"aruco{self.namespace[6]}/map"
        
        self.global_planner = Astar()
        self.goal = None

        # Publisher 및 Subscriber 생성
        self.create_subscription(PinkyRequest, f'{self.namespace}/goal_place', self.goal_callback, 10) # 목표위치
        self.path_pub = self.create_publisher(Path, f'{self.namespace}/planned_path', 10)       # 생성한 경로
        self.path_pub2 = self.create_publisher(Path, f'{self.namespace}/show_path', 10) 
        self.start_pub = self.create_publisher(PoseStamped, f'{self.namespace}/start', 10)   ## 출발지=로봇의 현재 위치 (디버깅용-현재위치 잘 받아오는지 확인)
        
        # 로봇의 현재위치를 받아오기 위한 TF 관련설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_robot_pose(self):
        try:
            now = rclpy.time.Time()
            # wait_for_transform_async 메서드는 3개의 인자만 필요합니다.
            self.tf_buffer.wait_for_transform_async('map', self.aruco_map, now)  # Duration 인자 제거
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                self.aruco_map,
                rclpy.time.Time()
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return (x, y)
        except TransformException as ex:
            self.get_logger().warn(f'현재 위치를 가져오는 데 실패했습니다: {ex}')
            return None

    def goal_callback(self, msg):
        robot_pos = self.get_robot_pose()
        if robot_pos is None:
            self.get_logger().warn('현재 위치를 사용할 수 없습니다.')
            return
        
        ############## 디버깅 확인용 추후 삭제 ####################
        # 퍼블리시할 메시지 생성
        start_msg = PoseStamped()
        start_msg.header.stamp = self.get_clock().now().to_msg()
        start_msg.header.frame_id = 'map'
        start_msg.pose.position.x = robot_pos[0]
        start_msg.pose.position.y = robot_pos[1]
        start_msg.pose.position.z = 0.0
        start_msg.pose.orientation.w = 1.0

        self.start_pub.publish(start_msg)  # RViz에서 확인 가능

        ############## 디버깅 확인용 추후 삭제 #####################

        start = self.pose_to_grid_position(*robot_pos) # 출발지=로봇의 현재위치: 월드 좌표계 -> 맵 좌표계
        goal = self.pose_to_grid_position(msg.x,msg.y)   # 도착지 : 월드 좌표계 -> 맵 좌표계
        self.get_logger().info(f'경로 계획 시작: {start} -> {goal}')

        # map_drawer = MapDrawer(self.pgm_path, self.yaml_path)
        # map_img = map_drawer.save_debug_image(start_pose=start, goal_pose=goal) 

        path, show_path = self.global_planner.run(self.grid, start, goal) # A* 알고리즘으로 맵정보, 출발지 맵 좌표, 도착지 맵 좌표 전달
        if path:
            self.get_logger().warn("경로를 찾았습니다.")
            self.publish_path(path)
            self.show_path(show_path)
        else:
            self.get_logger().warn("경로를 찾을 수 없습니다.")


    def pose_to_grid_position(self, x, y):
        """
        월드 좌표계 -> 맵 좌표계 변환
         - 월드 좌표계: 실수 단위의 실제위치
         - 맵 좌표계: Occupancygrid에서의 셀 인덱스
        """
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        
        # y 축 반전
        gy_flipped = self.height - 1 - gy

        return (gy_flipped, gx)

    def grid_to_pose(self, y, x):
        """
        맵 좌표계 -> 월드 좌표계 변환
         - 월드 좌표계: 실수 단위의 실제위치
         - 맵 좌표계: Occupancygrid에서의 셀 인덱스
        """
        y_flipped = self.height - 1 - y

        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x * self.resolution + self.origin[0]
        pose.pose.position.y = y_flipped * self.resolution + self.origin[1]
        pose.pose.position.z = 0.0

        # q = quaternion_from_euler(0, 0, 0)

        # pose.pose.orientation.x = q[0]
        # pose.pose.orientation.y = q[1]
        # pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = 1.0

        return pose

    def publish_path(self, path):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for y, x in path:
            msg.poses.append(self.grid_to_pose(y, x))  # 전달받은 경로점을 월드 좌표계로 변환하여 경로에 추가
        self.path_pub.publish(msg)

    def show_path(self, path):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for y, x in path:
            msg.poses.append(self.grid_to_pose(y, x))  # 전달받은 경로점을 월드 좌표계로 변환하여 경로에 추가
        self.path_pub2.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()