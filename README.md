# robit_navigation

ROBIT 21기 휴머노이드 내비게이션 (Python).
꼬깔을 A-star로 피해서 목표까지 가고, PID로 경로를 따라감. RViz2 시뮬레이션 포함.
Ubuntu 24.04 + ROS2 Jazzy 기준.

## 구성

| 파일 | 내 담당? | 하는 일 |
|---|---|---|
| `robit_navigation/nav_core.py` | ✅ | 좌표 변환, PID 제어, 도착 판단, 당근(Carrot) 따라가기 |
| `robit_navigation/astar.py` | ✅ | 꼬깔 부풀리기 + 격자 A-star 경로 계획 |
| `robit_navigation/obstacle_memory.py` | ✅ | Vision이 준 꼬깔(로봇 기준)을 필드 기준으로 바꿔서 기억 |
| `robit_navigation/navigation_node.py` | ✅ | ROS2 노드: 위치·목표·꼬깔 받기 → 경로 계획 → 걸음 명령·상태 보내기 |
| `robit_navigation/fake_robot_node.py` | ❌ 연습용 | 가짜 로봇 + 가짜 Vision (Walking, Localization, Vision 흉내) + RViz2 그림 |
| `launch/sim.launch.py` | | 노드 2개 + RViz2 한 번에 실행, 꼬깔 위치·튜닝 값도 여기서 바꿈 |
| `test/` | | 계산 확인 테스트 (`python3 -m pytest test`) |

## 토픽 (임시 이름, 팀 합의 후 변경)

| 토픽 | 메시지 | 방향 | 내용 |
|---|---|---|---|
| `/robot_pose` | geometry_msgs/Pose2D | 받음 | 내 위치 x, y [m], theta [rad] (Localization) |
| `/nav_goal` | geometry_msgs/Pose2D | 받음 | 목표 x, y [m], theta [rad] (Taskplanner) |
| `/vision_obstacles` | geometry_msgs/PoseArray | 받음 | 카메라에 보이는 꼬깔, frame_id `base_footprint`: x 앞, y 왼쪽 [m] (Vision `vision_packages`의 obstacle[] 대신) |
| `/goal_pose` | geometry_msgs/PoseStamped | 받음 | RViz2 "2D Goal Pose"로 찍은 목표 (테스트용) |
| `/nav_cmd` | geometry_msgs/Twist | 보냄 | linear.x 앞, linear.y 왼쪽 [m/s], angular.z 회전 [rad/s] (Taskplanner) |
| `/nav_status` | std_msgs/String | 보냄 | IDLE / MOVING / ARRIVED / NO_POSE / BLOCKED (Taskplanner) |
| `/nav_path` | nav_msgs/Path | 보냄 | A-star 경로 (RViz2 표시용) |
| `/nav_debug` | visualization_msgs/MarkerArray | 보냄 | 목표, 기억한 꼬깔 범위, 당근, 상태 글자 (RViz2 표시용) |

좌표 약속은 ROS 표준(앞 +x, 왼쪽 +y, 반시계 +)을 임시로 사용. Vision 토픽 문서(왼쪽 +)와 같음.

## 빌드

```bash
# robit_navigation 폴더를 ~/nav_ws/src/ 안에 넣은 뒤
cd ~/nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install`로 빌드하면 Python 코드를 고친 뒤 다시 빌드하지 않아도 됨 (launch 파일, 새 파일 추가는 다시 빌드).

## 실행

```bash
ros2 launch robit_navigation sim.launch.py
```

## RViz2에서 마우스로 조작

| 버튼 (RViz2 위쪽) | 하는 일 |
|---|---|
| **2D Goal Pose** | 필드를 클릭하고 끌어서 목표 위치·방향 정하기 (끈 방향 = 도착 후 바라볼 방향) |
| **2D Pose Estimate** | 가짜 로봇을 클릭한 곳으로 옮기기 (Reset 대신) |

화면 보는 법:

- 주황 원기둥: 실제 꼬깔 / 연한 주황 원판: 내비게이션이 **기억한** 꼬깔의 위험 범위
- 연한 파란 부채꼴: 가짜 카메라 시야 (이 안의 꼬깔만 Vision이 알려줌)
- 파란 선: A-star 경로 / 노란 공: 당근(Carrot)
- 초록 화살표·선: 로봇과 지나온 길 / 빨간 화살표: 목표
- 로봇 위 글자: `IDLE` / `MOVING 2.37m` / `ARRIVED` / `BLOCKED`

## 꼬깔 기억 (Vision 형식)

Vision은 지금 카메라에 보이는 꼬깔만 로봇 기준으로 알려줌. 그래서:

1. 받은 꼬깔을 내 위치·방향으로 필드 기준으로 바꿔서 기억 (꼬깔은 경기 중 안 움직임)
2. 같은 꼬깔을 다시 보면 위치를 조금씩 보정 (Vision 오차 줄이기)
3. 두 번 이상 본 꼬깔만 인정 (`min_hits`, 한 번 잘못 본 것 무시)
4. 카메라에 보여야 할 곳인데 `forget_time`초 동안 안 보이면 지움

## PID 제어

이동(앞뒤, 좌우)과 회전에 각각 PID. `ki`, `kd`를 0으로 두면 P만 쓰는 것과 같음.

| 방식 | 설정 | 언제 |
|---|---|---|
| P | ki = 0, kd = 0 (기본값) | 기본 |
| PD | kd만 올림 | 목표를 지나치거나 흔들릴 때 (D = 브레이크) |
| PID | ki, kd 둘 다 올림 | 미끄러짐 때문에 목표 앞에서 못 미칠 때 (I = 남은 오차 밀어내기) |

- I는 쌓이는 양에 상한이 있음 (`i_limit_move`, `i_limit_turn`), D는 부드럽게 만들어서 씀 (`d_alpha`)
- 새 목표가 오면 쌓인 I를 지움
- 경로 중간(당근 따라가기)에서는 I를 안 씀 (당근은 항상 앞에 있어서 오차가 0이 안 되므로)

PID 실험 (미끄러짐 흉내):

```bash
ros2 param set /fake_robot_node slip 0.1          # 걸을 때 뒤로 0.1m/s 미끄러짐 -> P만이면 목표 12.5cm 앞에서 멈춤
ros2 param set /navigation_node ki_move 0.5       # I 추가 -> 도착
```

## 실행 중에 튜닝 값 바꾸기

```bash
ros2 param set /navigation_node tol_yaw_deg 3.0
ros2 param list /navigation_node
```

- 실행 중 변경 가능: kp/ki/kd_move, i_limit_move, kp/ki/kd_turn, i_limit_turn, d_alpha, max_move, max_turn, tol_dist, tol_yaw_deg, carrot_points, robot_radius, margin, cone_radius, replan_period
- 시작할 때만: rate, field_length, field_width, grid_resolution, view_angle_deg, view_range, merge_dist, min_hits, forget_time
- 잘못된 값(음수 등)은 거부됨. 껐다 켜면 launch 파일 값으로 돌아감

## 테스트

```bash
cd ~/nav_ws/src/robit_navigation
python3 -m pytest test
```
