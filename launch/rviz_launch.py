from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    map_yaml_file = LaunchConfiguration('map_yaml_file').perform(context)

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('libro_aruco'),
        'rviz',
        'aruco_view.rviz'
    ])

    return [
        # RViz2 노드
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_libro_aruco',
            output='screen',
            arguments=['-d', rviz_config_file]  # RViz 설정 파일 지정
        ),

        # map_server 노드
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_yaml_file},
            ]
        ),

        # Lifecycle Manager for map_server
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',  # 다른 lifecycle_manager와 구분되는 이름
            output='screen',
            parameters=[
                {'autostart': True},
                {'node_names': ['map_server']}  # 관리할 노드 이름 목록
            ]
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml_file', default_value=PathJoinSubstitution([
            FindPackageShare('libro_aruco'), 'map', 'map.yaml'])),
        OpaqueFunction(function=launch_setup)
    ])
