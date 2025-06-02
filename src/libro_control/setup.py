from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'libro_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'libro_control_msgs', 'libro_picker_srvs', 'libro_arduino'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='addinedu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'libro_robot_controller = libro_control.libro_robot_controller_node:main',
            'libro_task_manager = libro_control.libro_task_manager_node:main',
            'active_libro_checker = libro_control.active_libro_checker_node:main',
        ],
    },
)
