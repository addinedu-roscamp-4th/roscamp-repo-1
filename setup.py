import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'libro_aruco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 설치
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.xml'))),
        # 파라미터 파일 설치
        (os.path.join('share', package_name, 'params'), glob(os.path.join('params', '*.yaml'))),
        # RViz 설정 파일 설치
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'map'), glob(os.path.join('map', '*.yaml'))),
        (os.path.join('share', package_name, 'map'), glob(os.path.join('map', '*.png'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='sohee.forest@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = libro_aruco.camera_publisher:main',
            'aruco_pose_publisher = libro_aruco.aruco_pose_publisher:main',
            'aruco_tf_broadcaster = libro_aruco.aruco_tf_broadcaster:main',
            'aruco_odom_tf_broadcaster = libro_aruco.aruco_odom_tf_broadcaster:main',
            'aruco_initialpose_publisher = libro_aruco.aruco_initialpose_publisher:main',
            # 'aruco_odom_publisher = libro_aruco.aruco_odom_publisher:main',
            # 'aruco_odom_base_tf_broadcaster = libro_aruco.aruco_odom_base_tf_broadcaster:main',
            # 'static_aruco_publisher = libro_aruco.static_aruco_publisher:main',
            # 'odom_publisher = libro_aruco.odom_publisher:main',
        ],
    },
)
