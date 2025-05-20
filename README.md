# 빌드
`colcon build --symlink-install --packages-select libro_aruco`

# 터미널 재시작
`exec $SHELL`

# 카메라 노드 실행
`ros2 launch libro_aruco camera_launch.py num_robots:=3 camera_device:='/dev/video2'`

# 특정 id의 아르코마커 odom 노드 실행
`ros2 launch libro_aruco aruco_launch.py marker_id:=3`

# 네임스페이스 (libroX) 지정하여 특정 id의 아르코마커 odom 노드 실행
`ros2 launch libro_aruco aruco_multi_launch.py marker_id:=3`

# 시각화 실행
`ros2 launch libro_aruco rviz_launch.py`
