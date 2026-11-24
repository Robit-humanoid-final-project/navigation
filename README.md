# robit_navigation

ROBIT 21기 휴머노이드 내비게이션 (Python).
꼬깔을 A-star로 피해서 목표까지 가고, PID로 경로를 따라감. 시뮬레이션 UI 포함.
Ubuntu 24.04 + ROS2 Jazzy 기준.

## 구성

| 파일 | 내 담당 | 하는 일 |
|---|---|---|
| `robit_navigation/nav_core.py` | ✅ | 좌표 변환, PID 제어, 도착 판단, 당근(Carrot) 따라가기 |
| `robit_navigation/astar.py` | ✅ | 꼬깔 부풀리기 + 격자 A-star 경로 계획 |
| `robit_navigation/ball_approach.py` | ✅ | 공을 장애물로 피하는 경로, 공 앞 자리 계산 |
| `robit_navigation/teammate.py` | ✅ | 우리 팀 다른 로봇 피하기: 움직이는 꼬깔 취급, 양보, 안전 정지 |
| `robit_navigation/obstacle_memory.py` | ✅ | world_model 꼬깔(필드 기준)을 기억, 유령 꼬깔 지우기 |
| `robit_navigation/navigation_node.py` | ✅ | ROS2 노드: 위치·목표·꼬깔 받기 → 경로 계획 → 걸음 명령·상태 보내기 |
| `robit_navigation/nav_gui_node.py` | ❌ 연습용 | Navigation UI (Localization UI와 같은 배치): 필드 보기, 위치 입력, 튜닝 |
| `robit_navigation/fake_robot_node.py` | ❌ 연습용 | 가짜 로봇 + 가짜 Localization (Walking, Localization, Vision 흉내) + 화면 그림 |
| `launch/sim.launch.py` | | 로봇 2대(각각 내비게이션 + 가짜 로봇) + UI 한 번에 실행, 꼬깔 위치·튜닝 값도 여기서 바꿈 |
| `test/` | | 계산 확인 테스트 (`python3 -m pytest test`) |

## 토픽 (팀 메시지 humanoid_interfaces)

| 토픽 | 메시지 | 방향 | 내용 |
|---|---|---|---|
| `nav_command` | humanoid_interfaces/NavCommand | 받음 (Taskplanner) | `nav_enable` (false = 멈춤), 목표 `x, y, theta`, `pos_tolerance`, `theta_tolerance` (0 이하면 기본값), `forbid_right_half` (Ready), `avoid_ball` |
| `world_model` | humanoid_interfaces/WorldModel | 받음 (Localization) | `robot_pose`, `ball_pose`·`ball_detected`, `obstacles` (지금 보이는 꼬깔, 기억해서 씀), `is_localized` (아직 미구현이라 `require_localized: False`) |
| `teammate_pose` | geometry_msgs/Pose2D | 받음 (UDP, 메시지 미정) | 우리 팀 다른 로봇 위치 |
| `nav_feedback` | humanoid_interfaces/NavFeedback | 보냄 (Taskplanner) | `status` 0 대기 / 1 이동 중 / 2 도착 / 3 길 없음, `eta_seconds` 도착 예상 시간 (남은 경로 / 최대 속도, 길 없음 -1) |
| `master2ik` | humanoid_interfaces/Master2IkMsg | 보냄 | 보행 명령. 내 속도를 비율로 바꿈: x_length = (앞 속도 / max_move) x ik_max_x, y·yaw도 같음. flag 1 걷기, 0 멈춤. 멈추면 0을 한 번만 보냄. ik_max_* 는 임시 값 (보행 담당과 확인 중) |
| `nav_cmd` | geometry_msgs/Twist | 보냄 | 같은 명령을 m/s, rad/s로 (가짜 로봇·UI용). linear.x 앞, linear.y 왼쪽, angular.z 회전 |
| `nav_path` | nav_msgs/Path | 보냄 | A-star 경로 (화면 표시용) |
| `nav_debug` | visualization_msgs/MarkerArray | 보냄 | 목표, 꼬깔·공 위험 범위, 당근, 상태 글자 (화면 표시용) |

좌표: 필드 기준 = 원점 필드 가운데, 앞(상대 골대 쪽) +x, 왼쪽 +y, 반시계 + / 단위 m, rad
로봇 2대 시뮬레이션에서는 토픽 앞에 로봇 이름이 붙음: `/robot1/nav_command`, `/robot2/world_model` ...

빌드하려면 `humanoid_interfaces` 패키지도 `~/nav_ws/src`에 있어야 함 (팀 GitHub)

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

## 실제 로봇에서 실행

시뮬레이션(`sim.launch.py`)은 가짜 로봇이 같이 켜지니까 **실제 로봇에서 쓰면 안 됨**. 실제 로봇은 `robot.launch.py`:

```bash
# 먼저 Localization, 보행(ik_walk) 켜기 (ROS_DOMAIN_ID 같게)
ros2 launch robit_navigation robot.launch.py                  # robot1 + UI
ros2 launch robit_navigation robot.launch.py robot:=robot2    # robot2 + UI
ros2 launch robit_navigation robot.launch.py gui:=false       # UI 없이 (Taskplanner가 nav_command)
```

- 토픽 이름이 팀 코드와 같음: `nav_command`, `world_model`, `nav_feedback`, `master2ik`
- UI는 "실제 로봇 모드": 로봇·공·꼬깔은 world_model 값, 목표 클릭·Go·Ready·Playing·공 앞으로·Stop 가능
- 로봇·공 옮기기, 모두 세팅 위치로, Reset은 숨김 (손으로 놓기), 마우스는 왼쪽 클릭·끌기(목표)만
- **UI를 닫으면 로봇 멈춤** (nav_enable = false)
- 내비게이션만 Ctrl+C로 끄면 보행이 마지막 명령으로 계속 걸을 수 있음 → 먼저 Stop 또는
  `ros2 topic pub --once nav_command humanoid_interfaces/msg/NavCommand "{nav_enable: false}"`
- 방향이 반대면 `ik_max_x`, `ik_max_y`, `ik_max_yaw`를 음수로 (UI 탭에 없음, `ros2 param set /navigation_node ik_max_x -25.0`)

## 앞걸음 위주로 걷기

휴머노이드는 옆걸음·뒷걸음이 느리고 잘 넘어짐. 그래서

| 상황 | 걷는 방법 |
|---|---|
| 가려는 쪽이 정면에서 30도(`turn_in_place_deg`)보다 벗어남 | 제자리 회전 (15도 `walk_again_deg` 밑으로 오면 다시 걷기) |
| 거의 정면 | 앞으로 걷기, 옆걸음은 최대 `max_side` 0.05 m/s |
| 목표 0.3m(`near_dist`) 안 | 마지막 방향을 보면서 옆걸음(`max_side_near` 0.15)·작은 뒷걸음(`max_back` 0.05)으로 맞춤 |
| 위치 도착 | 제자리에서 마지막 방향 맞추기 → ARRIVED |

## Navigation UI

| 위치 | 내용 |
|---|---|
| 왼쪽 위 | 필드: 로봇(파란 네모), 꼬깔, 공, 경로, 위험 범위, 카메라 시야 |
| 오른쪽 위 | 로봇 기준 화면 (로봇이 보는 방향 = 위), 상태, 걸음 명령 |
| 왼쪽 아래 | 조작할 로봇 고르기 (Robot 1 / 2), Robot / Goal / Ball / Ready 위치 입력 → Set, Go / Stop (목표 취소, 그 자리에 멈춤) / Reset (목표 취소 + 세팅 위치로) / 공 앞 거리 + 공 앞으로 |
| 오른쪽 아래 | 튜닝 탭 (PID, Path): 값 넣고 Apply, Read = 지금 값 읽기 / Text = 기록 |

필드 위 마우스: 왼쪽 클릭 = 목표 (가던 방향 그대로 바라봄), 왼쪽 끌기 = 목표 + 바라볼 방향, 오른쪽 클릭 = 공 옮기기, Ctrl + 왼쪽 끌기 = 로봇 옮기기

게임 상태: **Ready** / **Playing** 버튼 (켜진 쪽이 색으로 보이고 필드 왼쪽 위에도 표시). 처음 켜면 Playing이라 바로 목표를 줄 수 있음

## 꼬깔 기억

Localization(`world_model`)은 지금 카메라에 보이는 꼬깔만 필드 기준으로 알려줌. 그대로 쓰면 고개를 돌리는 순간 꼬깔이 사라짐. 그래서:

1. 받은 꼬깔을 기억 (꼬깔은 경기 중 안 움직임)
2. 같은 꼬깔을 다시 보면 위치를 조금씩 보정 (오차 줄이기)
3. 두 번 이상 본 꼬깔만 인정 (`min_hits`, 한 번 잘못 본 것 무시)
4. 카메라에 보여야 할 곳인데 `forget_time`초 동안 안 보이면 지움 (카메라 시야 `view_angle_deg`, `view_range`는 실제 카메라와 맞추기)
5. 카메라 밖이어도 `max_unseen_time`초(기본 10초) 동안 다시 안 보이면 지움 (내 위치가 잠깐 틀렸을 때 엉뚱한 곳에 찍힌 유령 꼬깔 방지)
6. 필드 밖에 보인 꼬깔은 버림
7. UI **꼬깔 기억 지우기** 버튼 (`nav_clear_cones` 토픽): 위치가 크게 틀렸다가 다시 잡혔을 때 한 번에 지우기

## 필드 밖으로 안 나가기

필드 밖으로 나가면 30초 페널티. 그래서 필드 선 안쪽 `boundary_margin`(기본 0.2m) 띠를 A-star 지도에서 막음.

- 로봇 반지름 0.15m → 몸이 선 밖으로 안 나가고 5cm 여유
- 목표가 선 근처(띠 안)나 필드 밖이면 `BLOCKED`
- 시작 위치가 선 위여도 안쪽으로 빠져나와서 감
- UI의 주황 안쪽 테두리 = 로봇 중심이 다닐 수 있는 범위
- 가짜 로봇: 로봇 중심이 선 밖으로 나가면 "필드 밖으로 나감!" 경고

## 경기 시작 준비 (규정 순서)

1. **모두 세팅 위치로**: 두 로봇을 세팅 위치(LEFT 진영 터치라인)에 놓음
2. **Ready**: RIGHT 진영 금지를 켜고, 두 로봇이 각자 **Ready 위치**까지 스스로 걸어가서 멈춤
3. 둘 다 도착하면 Text에 **준비 완료**
4. Playing 버튼을 누르면 경기 시작 (RIGHT 진영 제한 해제)

세팅 위치는 규정 그림대로 LEFT 진영 위·아래 라인의 절반 지점, 라인보다 뒤 (-2.25, ±3.1). 출발점이 필드 밖이어도 안쪽으로 들어와서 감

Ready 위치는 로봇마다 UI의 "Ready 위치" 줄에서 바꿈 (LEFT 진영이면 자유, 기본 robot1 (-0.8, -1.2), robot2 (-0.8, 1.2))

## Ready 때 RIGHT 진영 금지

규정: Ready 이동 중 RIGHT 진영에 들어가면 실패. 그래서 Taskplanner가 `nav_command`의 `forbid_right_half`를 true로 보내면

- 로봇 중심이 가운데선에서 `center_margin`(기본 0.2m) 왼쪽까지만 다님 (몸 반지름 0.15m → 5cm 여유)
- RIGHT 진영 목표는 `BLOCKED`, 이미 RIGHT 진영에 있으면 왼쪽으로 빠져나옴
- Playing이 되면 false로 보내서 제한을 풂
- UI: **Ready (LEFT만)** 버튼 (두 로봇 같이 적용), 켜지면 다닐 수 있는 범위가 빨간 테두리

```bash
ros2 topic pub --once /robot1/nav_command humanoid_interfaces/msg/NavCommand "{nav_enable: true, x: -0.8, y: -1.2, theta: 0.0, forbid_right_half: true, avoid_ball: true}"
```

## 로봇 2대 서로 피하기

1. 다른 로봇을 **움직이는 꼬깔**로 보고 A-star가 돌아감 (다른 로봇이 15cm 넘게 움직이면 바로 다시 계획)
2. **양보**: `yield_to_teammate: True`인 로봇(robot2)은 다른 로봇이 가까이(`yield_dist` 0.9m), 앞쪽에서, 움직이고 있으면 멈춤 → `WAITING`
   - 둘 다 피하려고만 하면 정면으로 마주칠 때 같이 우물쭈물하기 때문
   - 다른 로봇이 서 있으면 양보 안 함 (서 있는 로봇은 돌아서 감)
3. **안전 정지**: 다른 로봇이 바로 앞(`stop_dist` 0.45m)에 붙으면 누구든 멈춤
4. 목표에 다른 로봇이 서 있어서 길이 없으면, 다른 로봇을 빼고 경로를 짬 (곧 비킬 테니까)

UI에서 **Robot 1 / Robot 2** 버튼으로 조작할 로봇을 고름 (노란 테두리 = 고른 로봇)

## 공 앞으로 가기

- 공은 꼬깔처럼 장애물로 피함 (공 주변 0.37m 안으로 안 들어감 = 공을 안 밀음). 목표가 그 안이면 `BLOCKED`
- 내비게이션은 **공 앞**(공에서 찰 방향 반대로 "공 앞 거리" 뒤, 찰 방향 바라보기)까지만 데려감
- 그 안쪽 킥 정렬은 **Taskplanner**가 맡음 (그동안 Taskplanner가 `nav_enable = false`로 내비게이션을 멈춤)
- 목표가 없을 때(IDLE)는 정지 명령을 한 번만 보내고 조용히 있음 (킥 정렬 명령과 안 섞이게)
- UI: **공 앞 거리**(기본 0.5m, 실험하면서 정하기) + **공 앞으로** 버튼 = 고른 로봇이 상대 골대 쪽을 보고 공 앞에 섬

- UI에서는 공 피하기가 항상 켜짐 (끄고 켜는 건 Taskplanner가 `NavCommand.avoid_ball`로)

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

실제 로봇이 미끄러져서 목표 앞에서 못 미치고 멈추면, UI PID 탭에서 Ki 이동을 조금씩 올림 (예: 0.1 -> 0.3 -> 0.5)

## 실행 중에 튜닝 값 바꾸기

```bash
ros2 param set /robot1/navigation_node tol_yaw_deg 3.0
ros2 param list /robot1/navigation_node
```

- 실행 중 변경 가능: kp/ki/kd_move, i_limit_move, kp/ki/kd_turn, i_limit_turn, d_alpha, max_move, max_turn, tol_dist, tol_yaw_deg, carrot_points, robot_radius, margin, cone_radius, replan_period, ball_radius, require_localized, teammate_radius, yield_to_teammate, yield_dist, stop_dist, boundary_margin, center_margin, turn_in_place_deg, walk_again_deg, near_dist, max_side, max_side_near, max_back
- 시작할 때만: rate, field_length, field_width, grid_resolution, view_angle_deg, view_range, merge_dist, min_hits, forget_time
- 잘못된 값(음수 등)은 거부됨. 껐다 켜면 launch 파일 값으로 돌아감

## 테스트

```bash
cd ~/nav_ws/src/robit_navigation
python3 -m pytest test
```
