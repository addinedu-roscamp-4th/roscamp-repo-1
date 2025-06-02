# ros2 패키지

1. 메인 컨트롤 서버 실행
 
``` 
ros2 launch libro_control libro_system.launch.py
```

2. 로봇 활성화 : 필요한 libro 개수만큼 실행

``` 
ros2 run libro_control libro_robot_controller --ros-args -p robot_id:=libro1
```
  이름 : libro1, libro2, libro3 ..

3. 픽업함 서버 실행 : 아두이노 연결한 노트북에서 실행
```
ros2 run libro_arduino cabinet_check_publisher
```
4. 바구니 노드 실행 : 아두이노 연결한 노트북에서 실행
```
ros2 run libro_arduino basket_check_publisher
```
5. 핑키 유저 식별 노드
```
ros2 run libro_qr user_checker_node
```
