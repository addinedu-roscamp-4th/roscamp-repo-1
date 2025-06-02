from setuptools import setup

package_name = 'libro_picker_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='addinedu@todo.todo',
    description='Keyboard teleoperation node for libro picker',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_teleop_service = libro_picker_teleop.keyboard_teleop_service:main',
            'keyboard_teleop_topic = libro_picker_teleop.keyboard_teleop_topic:main',
        ],
    },
)
