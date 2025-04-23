from setuptools import find_packages, setup

package_name = 'libro_picker_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='addinedu@todo.todo',
    description='ROS2 controller node for MyCobot 280 robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = libro_picker_controller.robot_controller:main',
            'robot_control_from_keyboard = libro_picker_controller.robot_control_from_keyboard:main',
        ],
    },
)
