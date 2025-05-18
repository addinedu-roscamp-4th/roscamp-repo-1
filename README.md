# Libro Robot Controller Node
`libro_robot_controller_node`는 도서관 로봇의 제어를 담당하는 ROS2 노드입니다. 
이 노드는 `robot_id` 파라미터를 통해 여러 로봇을 독립적으로 제어할 수 있습니다

## Robot ID Parameter
- 기본값: `libro_1`
- 설정 방법:
  ```bash
  ros2 run libro_control libro_robot_controller --ros-args -p robot_id:=libro_1
  ```
  ```bash
  ros2 run libro_control libro_robot_controller --ros-args -p robot_id:=libro_2
  ```

## Topic Structure (robot_id = 'libro_1' 기준)

### Publishers
| Topic Name | Message Type | Description | Target |
|------------|--------------|-------------|--------|
| `libro_1/goal_place` | PinkyRequest | 로봇의 목적지 이동 요청 (책장/픽업 위치) | 주행 패키지 |
| `libro_1/pick_book` | JetcobotPickRequest | 책장에서 책을 집기 위한 요청 (x, y, z, angle) | 로봇팔 패키지 |
| `libro_1/place_book` | JetcobotPlaceRequest | 바구니에서 책을 꺼내기 위한 요청 | 로봇팔 패키지 |
| `libro_1/check_basket` | String | 바구니 확인 요청 ("CHECK") | 아두이노 패키지 |
| `libro_1/check_cabinet` | String | 픽업란 확인 요청 ("CHECK") | 아두이노 패키지 |
| `libro_1/state` | LibroRobotState | 로봇의 현재 상태 정보 (0.5초 주기) | PyQt 패키지 |
| `libro_1/completed` | Bool | 미션 완료 상태 | task_manager |
| `libro_1/stop_arm` | Bool | 로봇팔 작업 정지/재시작 요청 | 로봇팔 패키지 |
| `libro_1/stop_mobile` | Bool | 로봇 이동 정지/재시작 요청 | 주행 패키지 |
| `robot_log` | LibroRobotLog | 로봇 미션 수행 로그 (모든 로봇 공유) | PyQt 패키지 |

### Subscribers
| Topic Name | Message Type | Description | Target |
|------------|--------------|-------------|--------|
| `request_book` | BookPickUp | 책 픽업 미션 요청 수신 | task_manager | 
| `libro_1/goal_info` | PinkyResponse | 목적지 이동 응답 | 주행 패키지 |
| `libro_1/pick_info` | JetcobotPickResponse | 책장에서 책 집기 작업 응답 | 로봇팔 패키지 |
| `libro_1/place_info` | JetcobotPlaceResponse | 바구니에서 책 꺼내기 작업 응답 | 로봇팔 패키지 |
| `libro_1/check_basket_info` | String | 바구니 확인 응답 | 아두이노 패키지 |
| `libro_1/check_cabinet_info` | String | 픽업란 확인 응답 | 아두이노 패키지 |
| `libro_1/stop` | Bool | 로봇 작업 정지/재시작 요청 수신 | PyQt 패키지 |

## State Machine
* 로봇은 다음 절차를 순차적으로 수행함으로서 책 픽업 미션을 수행합니다.
1. 대기중
2. 도서관 책장 이동
3. 책 집기 자세 요청
4. 책장에서 책 꺼내기
5. 바구니 확인
6. 픽업 위치 이동
7. 바구니에서 책 꺼내기
8. 픽업란 확인
* 로봇 작업 정지 / 재시작 : 위 8개의 절차 도중 정지 & 재시작했을시 해당 절차를 이어서 수행

# Libro Task Manager Node

`libro_task_manager_node`는 도서관 로봇 시스템의 중앙 제어 노드입니다. 여러 로봇의 상태를 모니터링하고 작업을 효율적으로 분배하는 역할을 합니다.

## Overview
- 로봇 상태 관리 (충전중, 대기중, 픽업미션중, 길안내미션중)
- 배터리 레벨 모니터링 및 충전 관리
- 서비스 요청 관리 및 로봇 작업 할당
- 로봇 상태 및 작업 로그 관리
- 실행 명령어
  ```
  ros2 run libro_control libro_task_manager_node
  ```

## Topic Structure

### Publishers
| Topic Name | Message Type | Description | Target |
|------------|--------------|-------------|--------|
| `request_book` | BookPickUp | 책 픽업 미션 요청 | robot_controller |
| `request_nav` | Navigate | 길 안내 미션 요청 | robot_controller |
| `libro_robot_states` | String | 모든 로봇의 상태 정보 (JSON) | PyQt 패키지 |

### Subscribers
| Topic Name | Message Type | Description | Target |
|------------|--------------|-------------|--------|
| `robot_log` | LibroRobotLog | 로봇 미션 수행 로그 | robot_controller |
| `libro_1/battery_present` | Float32 | 로봇 1의 배터리 레벨 | 주행 패키지 |
| `libro_1/completed` | Bool | 로봇 1의 미션 완료 상태 | robot_controller |
| `libro_2/battery_present` | Float32 | 로봇 2의 배터리 레벨 | 주행 패키지 |
| `libro_2/completed` | Bool | 로봇 2의 미션 완료 상태 | robot_controller |

## Robot States
```python
class RobotState(Enum):
    CHARGING = 'charging'      # 충전중
    WAITING = 'waiting'        # 대기중
    PICKUP_MISSION = 'pickup'  # 픽업미션중
    NAVIGATION_MISSION = 'nav' # 길안내미션중
```

## Battery Management
- 배터리 임계값:
  - LOW_BATTERY_THRESHOLD: 20%
  - CHARGED_BATTERY_THRESHOLD: 80%
- 배터리 상태에 따른 자동 전환:
  - 대기중 + 배터리 20% 미만 → 충전중
  - 충전중 + 배터리 80% 이상 → 대기중

## Service Request Management
- 로컬 JSON 파일에서 서비스 요청 관리
- 파일 경로: `/home/addinedu/libro_total/src/libro_control/libro_control/libro_service_request_log.json`
- 지원하는 요청 유형:
  1. 책 픽업 요청 (`book_request`)
     - 책 이름
     - 책장 위치
     - 픽업 위치
     - 주문 번호
  2. 길 안내 요청 (`navigate_request`)
     - 사용자 이메일
     - 주문 번호
     - 시작 위치
     - 목적지 위치

## Task Processing
- 1초마다 작업 큐 확인
- 작업 할당 조건:
  1. 로봇이 대기중 상태
  2. 현재 작업이 없음
  3. 서비스 요청이 존재
- 작업 완료 시:
  1. 로봇 상태를 대기중으로 변경
  2. 현재 작업 정보 초기화

## Logging
- 로봇 작업 로그 저장
- 파일 경로: `/home/addinedu/libro_total/src/libro_control/libro_control/libro_robot_log.json`
- 로그 항목:
  - 타임스탬프
  - 로봇 ID
  - 주문 번호
  - 작업 로그
  - 성공 여부

## State Publishing
- 10Hz로 로봇 상태 발행
- JSON 형식으로 모든 로봇의 현재 상태 전송
- 상태 전이 검증 로직 포함
