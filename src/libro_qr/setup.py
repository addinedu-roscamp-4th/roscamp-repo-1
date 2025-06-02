from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'libro_qr'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/emotion', glob('emotion/*.gif')),
        ('share/' + package_name + '/emotion', glob('emotion/*.jpeg')),
        ('share/' + package_name + '/emotion/angry', glob('emotion/angry/*')),
        ('share/' + package_name + '/emotion/basic', glob('emotion/basic/*')),
        ('share/' + package_name + '/emotion/bored', glob('emotion/bored/*')),
        ('share/' + package_name + '/emotion/fun', glob('emotion/fun/*')),
        ('share/' + package_name + '/emotion/happy', glob('emotion/happy/*')),
        ('share/' + package_name + '/emotion/hello', glob('emotion/hello/*')),
        ('share/' + package_name + '/emotion/interest', glob('emotion/interest/*')),
        ('share/' + package_name + '/emotion/sad', glob('emotion/sad/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='addinedu',
    maintainer_email='addinedu@todo.todo',
    description='QR code user checker node with LCD display and emotion expressions',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'user_checker_node = libro_qr.user_checker_node:main'
        ],
    },
)
