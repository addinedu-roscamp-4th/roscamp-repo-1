from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    marker_id = LaunchConfiguration('marker_id').perform(context)

    return [
        # ArUco map broadcaster
        Node(
            package='libro_aruco',
            executable='aruco_tf_broadcaster',
            name='aruco_tf_broadcaster',
            output='screen',
            parameters=[{
                'marker_id': int(marker_id),
                # 'map_frame': 'map'
            }]
        ),

        # ArUco Odom publisher
        Node(
            package='libro_aruco',
            executable='aruco_odom_tf_broadcaster',
            name='aruco_odom_f_broadcaster',
            output='screen',
            parameters=[{
                'marker_id': int(marker_id),
                # 'base_link_frame': 'base_footprint',
            }]
        ),

        # ArUco initialpose publisher
        Node(
            package='libro_aruco',
            executable='aruco_initialpose_publisher',
            name='aruco_initialpose_publisher',
            output='screen',
            parameters=[{'marker_id': int(marker_id)}]
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('marker_id', default_value='1'),
        OpaqueFunction(function=launch_setup)
    ])
