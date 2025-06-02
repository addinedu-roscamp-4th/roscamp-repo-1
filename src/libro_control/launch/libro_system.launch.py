from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Active Libro Checker Node (starts immediately)
        Node(
            package='libro_control',
            executable='active_libro_checker',
            name='active_libro_checker_node',
            output='screen'
        ),

        # # Cabinet Check Publisher Node (starts after 3 seconds)
        # TimerAction(
        #     period=3.0,
        #     actions=[
        #         Node(
        #             package='libro_arduino',
        #             executable='cabinet_check_publisher',
        #             name='cabinet_check_publisher',
        #             parameters=[{
        #                 'arduino_port': '/dev/ttyACM0',
        #                 'baud_rate': 9600
        #             }],
        #             output='screen'
        #         )
        #     ]
        # ),

        # Libro Task Manager Node (starts after 6 seconds)
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='libro_control',
                    executable='libro_task_manager',
                    name='libro_task_manager_node',
                    output='screen'
                )
            ]
        )
    ]) 