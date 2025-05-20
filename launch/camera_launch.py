from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera_device = LaunchConfiguration('camera_device').perform(context)

    camera_link = f'{camera_name}_link'

    calibration_file = PathJoinSubstitution([
        FindPackageShare('libro_aruco'),
        'params',
        'camera_param.yaml'
    ]).perform(context)

    return [
        # 카메라 퍼블리셔 노드
        Node(
            package='libro_aruco',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'camera_device': camera_device,
                'camera_name': camera_name,
                'calibration_file': calibration_file,
                'publish_rate': 30.0,
                'frame_width': 640,
                'frame_height': 480
            }]
        ),

        # 정적 TF (Map → Camera)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_tf_broadcaster',
            output='screen',
            arguments=[
                '1.02278013', '0.51618255', '1.64612302',
                '0.99894806', '0.00466156', '0.00100031', '0.04560753',
                'map', camera_link
            ]
        ),

        # ArUco 마커 포즈 퍼블리셔 노드
        Node(
            package='libro_aruco',
            executable='aruco_pose_publisher',
            name='aruco_pose_publisher',
            output='screen',
            parameters=[{
                'camera_name': camera_name,
                'camera_frame': camera_link,
                'marker_size': 0.07
            }]
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_name', default_value='global_camera'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video2'),
        OpaqueFunction(function=launch_setup)
    ])
