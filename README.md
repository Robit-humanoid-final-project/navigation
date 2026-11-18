# robit_navigation

ROBIT 21기 휴머노이드 내비게이션 (Python).
꼬깔을 A-star로 피해서 목표까지 가고, PID로 경로를 따라감. RViz2 시뮬레이션 포함.
Ubuntu 24.04 + ROS2 Jazzy 기준.

## 구성

| 파일 | 내 담당? | 하는 일 |
|---|---|---|
| `robit_navigation/nav_core.py` | ✅ | 좌표 변환, PID 제어, 도착 판단, 당근(Carrot) 따라가기 |
| `robit_navigation/astar.py` | ✅ | 꼬깔 부풀리기 + 격자 A-star 경로 계획 |
| `robit_navigation/ball_approach.py` | ✅ | 공을 밀지 않고 공 뒤(킥 자리)로 돌아가는 경로 |
| `robit_navigation/teammate.py` | ✅ | 우리 팀 다른 로봇 피하기: 움직이는 꼬깔 취급, 양보, 안전 정지 |
| `robit_navigation/obstacle_memory.py` | ✅ | Vision이 준 꼬깔(로봇 기준)을 필드 기준으로 바꿔서 기억 |
| `robit_navigation/navigation_node.py` | ✅ | ROS2 노드: 위치·목표·꼬깔 받기 → 경로 계획 → 걸음 명령·상태 보내기 |
| `robit_navigation/nav_gui_node.py` | ❌ 연습용 | Navigation UI (Localization UI와 같은 배치): 필드 보기, 위치 입력, 튜닝 |
| `robit_navigation/fake_robot_node.py` | ❌ 연습용 | 가짜 로봇 + 가짜 Vision (Walking, Localization, Vision 흉내) + RViz2 그림 |
| `launch/sim.launch.py` | | 로봇 2대(각각 내비게이션 + 가짜 로봇) + UI 한 번에 실행, 꼬깔 위치·튜닝 값도 여기서 바꿈 |
| `test/` | | 계산 확인 테스트 (`python3 -m pytest test`) |

## 토픽 (임시 이름, 팀 합의 후 변경)

| 토픽 | 메시지 | 방향 | 내용 |
|---|---|---|---|
| `/robot_pose` | geometry_msgs/Pose2D | 받음 | 내 위치 x, y [m], theta [rad] (Localization) |
| `/nav_goal` | geometry_msgs/Pose2D | 받음 | 목표 x, y [m], theta [rad] (Taskplanner) |
| `/vision_obstacles` | geometry_msgs/PoseArray | 받음 | 카메라에 보이는 꼬깔, frame_id `base_footprint`: x 앞, y 왼쪽 [m] (Vision `vision_packages`의 obstacle[] 대신) |
| `/ball_pose` | geometry_msgs/PointStamped | 받음 | 공 위치, frame_id `map`이면 필드 기준, 그 밖은 로봇 기준 (x 앞, y 왼쪽) [m] |
| `/nav_cancel` | std_msgs/Empty | 받음 | 목표 취소: 멈추고 IDLE (Taskplanner, UI의 Stop·Reset) |
| `/teammate_pose` | geometry_msgs/Pose2D | 받음 | 우리 팀 다른 로봇 위치 x, y [m] (필드 기준, 실제로는 UDP) |
| `/goal_pose` | geometry_msgs/PoseStamped | 받음 | RViz2 "2D Goal Pose"로 찍은 목표 (테스트용) |
| `/nav_cmd` | geometry_msgs/Twist | 보냄 | linear.x 앞, linear.y 왼쪽 [m/s], angular.z 회전 [rad/s] (Taskplanner) |
| `/nav_status` | std_msgs/String | 보냄 | IDLE / MOVING / ARRIVED / NO_POSE / BLOCKED / WAITING (Taskplanner) |
| `/nav_path` | nav_msgs/Path | 보냄 | A-star 경로 (RViz2 표시용) |
| `/nav_debug` | visualization_msgs/MarkerArray | 보냄 | 목표, 기억한 꼬깔 범위, 당근, 상태 글자 (RViz2 표시용) |

좌표 약속은 ROS 표준(앞 +x, 왼쪽 +y, 반시계 +)을 임시로 사용. Vision 토픽 문서(왼쪽 +)와 같음.

로봇 2대 시뮬레이션에서는 토픽 앞에 로봇 이름이 붙음: `/robot1/robot_pose`, `/robot2/nav_goal` ...
공(`/ball_pose`)은 같이 씀. robot1의 `teammate_pose` = `/robot2/robot_pose` (반대도 같음)

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
# RViz2도 같이 보려면
ros2 launch robit_navigation sim.launch.py rviz:=true
```

## Navigation UI

| 위치 | 내용 |
|---|---|
| 왼쪽 위 | 필드: 로봇(파란 네모), 꼬깔, 공, 경로, 위험 범위, 카메라 시야 |
| 오른쪽 위 | 로봇 기준 화면 (로봇이 보는 방향 = 위), 상태, 걸음 명령 |
| 왼쪽 아래 | 조작할 로봇 고르기 (Robot 1 / 2), Robot / Goal / Ball 위치 입력 → Set, Go / Stop (목표 취소, 그 자리에 멈춤) / Reset (목표 취소 + 시작 위치로) |
| 오른쪽 아래 | 튜닝 탭 (PID, Path, Fake Robot): 값 넣고 Apply, Read = 지금 값 읽기 / Text = 기록 |

필드 위 마우스: 왼쪽 클릭 후 끌기 = 목표, 오른쪽 클릭 = 공 옮기기, Ctrl + 왼쪽 끌기 = 로봇 옮기기

## RViz2에서 마우스로 조작 (rviz:=true 일 때)

| 버튼 (RViz2 위쪽) | 하는 일 |
|---|---|
| **2D Goal Pose** | 필드를 클릭하고 끌어서 목표 위치·방향 정하기 (끈 방향 = 도착 후 바라볼 방향) |
| **2D Pose Estimate** | 가짜 로봇을 클릭한 곳으로 옮기기 (Reset 대신) |
| **Publish Point** | 공을 클릭한 곳으로 옮기기 |

화면 보는 법:

- 주황 원기둥: 실제 꼬깔 / 연한 주황 원판: 내비게이션이 **기억한** 꼬깔의 위험 범위
- 연한 파란 부채꼴: 가짜 카메라 시야 (이 안의 꼬깔만 Vision이 알려줌)
- 파란 선: A-star 경로 / 노란 공: 당근(Carrot)
- 하얀 공: 공 / 연한 하얀 원판: 공 위험 범위 (로봇 중심이 들어가면 공을 밂)
- 초록 화살표·선: 로봇과 지나온 길 / 빨간 화살표: 목표
- 로봇 위 글자: `IDLE` / `MOVING 2.37m` / `ARRIVED` / `BLOCKED` / `WAITING`

## 꼬깔 기억 (Vision 형식)

Vision은 지금 카메라에 보이는 꼬깔만 로봇 기준으로 알려줌. 그래서:

1. 받은 꼬깔을 내 위치·방향으로 필드 기준으로 바꿔서 기억 (꼬깔은 경기 중 안 움직임)
2. 같은 꼬깔을 다시 보면 위치를 조금씩 보정 (Vision 오차 줄이기)
3. 두 번 이상 본 꼬깔만 인정 (`min_hits`, 한 번 잘못 본 것 무시)
4. 카메라에 보여야 할 곳인데 `forget_time`초 동안 안 보이면 지움

## 로봇 2대 서로 피하기

1. 다른 로봇을 **움직이는 꼬깔**로 보고 A-star가 돌아감 (다른 로봇이 15cm 넘게 움직이면 바로 다시 계획)
2. **양보**: `yield_to_teammate: True`인 로봇(robot2)은 다른 로봇이 가까이(`yield_dist` 0.9m), 앞쪽에서, 움직이고 있으면 멈춤 → `WAITING`
   - 둘 다 피하려고만 하면 정면으로 마주칠 때 같이 우물쭈물하기 때문
   - 다른 로봇이 서 있으면 양보 안 함 (서 있는 로봇은 돌아서 감)
3. **안전 정지**: 다른 로봇이 바로 앞(`stop_dist` 0.45m)에 붙으면 누구든 멈춤
4. 목표에 다른 로봇이 서 있어서 길이 없으면, 다른 로봇을 빼고 경로를 짬 (곧 비킬 테니까)

UI에서 **Robot 1 / Robot 2** 버튼으로 조작할 로봇을 고름 (노란 테두리 = 고른 로봇)

## 공 피해서 킥 자리로 가기

패스·슛을 하려면 공 바로 뒤에 서야 하는데, 그냥 직진하면 공을 밀어 버림. 그래서 (`avoid_ball: True`):

1. 공도 꼬깔처럼 피해야 할 장애물로 넣음
2. 목표가 공 바로 옆(킥 자리)이면: 공 반대편 "진입점"까지 A-star로 돌아가고, 진입점부터 똑바로 들어감
3. 이미 공 뒤 일직선 위에 있으면 물러나지 않고 바로 들어감

킥 자리 예시: 공 (bx, by)를 +x 쪽으로 차려면 목표 = (bx - 0.3, by, 0도)

```bash
# 공이 (0, 1)일 때, +x 쪽으로 찰 자리로
ros2 topic pub --once /nav_goal geometry_msgs/msg/Pose2D "{x: -0.3, y: 1.0, theta: 0.0}"
# 비교: 공 피하기 끄기 -> 공을 밀고 지나감 (가짜 로봇 창에 "공을 밀었음!")
ros2 param set /robot1/navigation_node avoid_ball false
```

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
ros2 param set /robot1/fake_robot_node slip 0.1          # 걸을 때 뒤로 0.1m/s 미끄러짐 -> P만이면 목표 12.5cm 앞에서 멈춤
ros2 param set /robot1/navigation_node ki_move 0.5       # I 추가 -> 도착
# (UI의 Fake Robot / PID 탭에서 해도 같음)
```

## 실행 중에 튜닝 값 바꾸기

```bash
ros2 param set /robot1/navigation_node tol_yaw_deg 3.0
ros2 param list /robot1/navigation_node
```

- 실행 중 변경 가능: kp/ki/kd_move, i_limit_move, kp/ki/kd_turn, i_limit_turn, d_alpha, max_move, max_turn, tol_dist, tol_yaw_deg, carrot_points, robot_radius, margin, cone_radius, replan_period, avoid_ball, ball_radius, approach_margin, teammate_radius, yield_to_teammate, yield_dist, stop_dist
- 시작할 때만: rate, field_length, field_width, grid_resolution, view_angle_deg, view_range, merge_dist, min_hits, forget_time
- 잘못된 값(음수 등)은 거부됨. 껐다 켜면 launch 파일 값으로 돌아감

## 테스트

```bash
cd ~/nav_ws/src/robit_navigation
python3 -m pytest test
```
