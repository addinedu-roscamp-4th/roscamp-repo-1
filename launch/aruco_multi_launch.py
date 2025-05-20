from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def launch_setup(context, *args, **kwargs):
    marker_id = LaunchConfiguration('marker_id').perform(context)
    robot_namespace = f'libro{marker_id}'

    robot_nodes = GroupAction(
        actions=[
            PushRosNamespace(namespace=robot_namespace),

            Node(
                package='libro_aruco',
                executable='aruco_tf_broadcaster',
                name='aruco_tf_broadcaster',
                output='screen',
                parameters=[{
                    'marker_id': int(marker_id),
                }]
            ),

            Node(
                package='libro_aruco',
                executable='aruco_odom_tf_broadcaster',
                output='screen',
                parameters=[{
                    'marker_id': int(marker_id),
                }]
            ),

            Node(
                package='libro_aruco',
                executable='aruco_initialpose_publisher',
                output='screen',
                parameters=[{
                    'marker_id': int(marker_id),
                }]
            ),
        ]
    )

    return [robot_nodes]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('marker_id', default_value='1'),
        OpaqueFunction(function=launch_setup)
    ])
