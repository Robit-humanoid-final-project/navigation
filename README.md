# robit_navigation

ROBIT 21기 휴머노이드 내비게이션 (Python).
꼬깔을 A-star로 피해서 목표까지 가고, PID로 경로를 따라감. 실제 로봇용 UI 포함.
Ubuntu 24.04 + ROS2 Jazzy 기준.

## 구성

| 파일 | 내 담당 | 하는 일 |
|---|---|---|
| `robit_navigation/nav_core.py` | ✅ | 좌표 변환, PID 제어, 도착 판단, 당근(Carrot) 따라가기, 보행 값 바꾸기(to_walk) |
| `robit_navigation/astar.py` | ✅ | 꼬깔 부풀리기 + 격자 A-star 경로 계획 |
| `robit_navigation/ball_approach.py` | ✅ | 공을 장애물로 피하는 경로, 공 앞 자리 계산 |
| `robit_navigation/teammate.py` | ✅ | 우리 팀 다른 로봇 피하기: 움직이는 꼬깔 취급, 양보, 안전 정지 |
| `robit_navigation/obstacle_memory.py` | ✅ | world_model 꼬깔(필드 기준) 기억, 유령 꼬깔 지우기 |
| `robit_navigation/navigation_node.py` | ✅ | ROS2 노드: 위치·목표·꼬깔 받기 → 경로 계획 → 보행 명령·상태 보내기 |
| `robit_navigation/nav_gui_node.py` | ❌ 확인용 | Navigation UI (Localization UI와 같은 배치): 필드 보기, 목표 주기, 튜닝 |
| `launch/robot.launch.py` | | 실제 로봇에서 내비게이션 + UI 실행 |
| `resource/field_map.png` | | UI 필드 그림 (Localization 맵) |
| `test/` | | 계산 확인 테스트 (`python3 -m pytest test`) |

## 토픽 (팀 메시지 humanoid_interfaces)

| 토픽 | 메시지 | 방향 | 내용 |
|---|---|---|---|
| `nav_command` | humanoid_interfaces/NavCommand | 받음 (Taskplanner) | `nav_enable` (false = 멈춤), 목표 `x, y, theta`, `pos_tolerance`, `theta_tolerance` (0 이하면 기본값), `forbid_right_half` (Ready), `avoid_ball` |
| `world_model` | humanoid_interfaces/WorldModel | 받음 (Localization) | `robot_pose`, `ball_pose`·`ball_detected`, `obstacles` (지금 보이는 꼬깔, 기억해서 씀), `is_localized` (아직 미구현이라 `require_localized: False`) |
| `teammate_pose` | geometry_msgs/Pose2D | 받음 (UDP, 메시지 미정) | 우리 팀 다른 로봇 위치 |
| `nav_clear_cones` | std_msgs/Empty | 받음 (UI) | 꼬깔 기억 지우기 |
| `nav_feedback` | humanoid_interfaces/NavFeedback | 보냄 (Taskplanner) | `status` 0 대기 / 1 이동 중 / 2 도착 / 3 길 없음·위치 모름, `eta_seconds` 도착 예상 시간 (남은 경로 / 최대 속도, 길 없음 -1) |
| `master2ik` | humanoid_interfaces/Master2IkMsg | 보냄 (보행) | 보행 명령. 속도를 비율로 바꿈: x_length = (앞 속도 / max_move) x ik_max_x, y·yaw도 같음. flag 1 걷기, 0 멈춤. **nav_enable = true일 때만 보냄** (false면 하나도 안 보냄, 멈추기는 Taskplanner) |
| `nav_path` | nav_msgs/Path | 보냄 | A-star 경로 (UI 표시용) |
| `nav_debug` | visualization_msgs/MarkerArray | 보냄 | 목표, 꼬깔·공 위험 범위, 당근, 상태 글자 (UI 표시용) |

좌표: 필드 기준 = 원점 필드 가운데, 앞(상대 골대 쪽) +x, 왼쪽 +y, 반시계 + / 단위 m, rad

빌드하려면 `humanoid_interfaces` 패키지도 같은 작업공간 `src`에 있어야 함 (팀 GitHub)

## 빌드

```bash
cd ~/colcon_ws            # 노트북은 ~/nav_ws
colcon build --symlink-install --packages-select humanoid_interfaces robit_navigation
source install/setup.bash
```

`--symlink-install`로 빌드하면 Python 코드를 고친 뒤 다시 빌드하지 않아도 됨 (launch 파일, 새 파일 추가는 다시 빌드).

## 실행 (실제 로봇)

```bash
# 먼저 카메라, 비전, 모터, 보행(ik_walk), IMU, Localization 켜기 (ROS_DOMAIN_ID 같게)
ros2 launch robit_navigation robot.launch.py                  # robot1 + UI
ros2 launch robit_navigation robot.launch.py robot:=robot2    # robot2 + UI
ros2 launch robit_navigation robot.launch.py gui:=false       # UI 없이 (Taskplanner가 nav_command)
```

- 토픽 이름이 팀 코드와 같음: `nav_command`, `world_model`, `nav_feedback`, `master2ik`
- **Taskplanner와 같이 돌릴 때는 UI 버튼을 누르지 말기** (UI도 `nav_command`를 보내서 명령이 섞임)
- 내비게이션은 `nav_enable = false`면 보행 명령을 안 보냄 (팀 약속). UI 없이 멈출 때는 보행에 직접:
  `ros2 topic pub --once master2ik humanoid_interfaces/msg/Master2IkMsg "{flag: 0.0}"`
- **보행 값**: 시작할 때 보행 담당 yaml(`~/colcon_ws/src/walking_parm/walking_parm.yaml`)에서 `Nav_test_X`, `Nav_test_side_L`, `Nav_test_Yaw_L`을 읽어 `ik_max_x`, `ik_max_y`, `ik_max_yaw`로 씀
  - yaml 받기: `cd ~/colcon_ws/src && git clone https://github.com/Robit-humanoid-final-project/walking_parm.git`
  - yaml이 없으면 기본값(앞 30, 옆 10, 회전 5) 사용. 시작 로그에 어떤 값을 쓰는지 나옴
  - 다른 위치면 `walking_parm_file` 파라미터로 경로 지정
- 방향이 반대면 `ik_max_x`, `ik_max_y`, `ik_max_yaw`를 음수로 (`ros2 param set /navigation_node ik_max_x -30.0`)

## Navigation UI

| 위치 | 내용 |
|---|---|
| 왼쪽 위 | 필드 (Localization 맵 그림): 로봇, 꼬깔, 공, 경로, 위험 범위, 목표 |
| 오른쪽 위 | 로봇 기준 화면 (로봇이 보는 방향 = 위), 상태, 보행으로 나가는 값 (x, y, yaw, flag), 도착 예상 |
| 왼쪽 아래 | Ready / Playing, Goal 입력 + Go, Stop, Ready 위치, 공 앞 거리 + 공 앞으로, 꼬깔 기억 지우기 |
| 오른쪽 아래 | 튜닝 탭 (PID, Path): 값 넣고 Apply, Read = 지금 값 읽기 / Text = 기록 |

- 필드 위 마우스: 왼쪽 클릭 = 목표 (가던 방향 그대로 바라봄), 왼쪽 끌기 = 목표 + 바라볼 방향
- 게임 상태: **Ready** / **Playing** 버튼 (켜진 쪽이 색으로 보이고 필드 왼쪽 위에도 표시). 처음 켜면 Playing이라 바로 목표를 줄 수 있음
- **Stop, UI 닫기 = 로봇 멈춤** (nav_enable = false + UI가 보행 멈춤 master2ik flag 0을 직접 보냄)
- 공 피하기는 항상 켬 (끄고 켜는 건 Taskplanner가 `NavCommand.avoid_ball`로)

## 앞걸음 위주로 걷기

휴머노이드는 옆걸음·뒷걸음이 느리고 잘 넘어짐. 그래서

| 상황 | 걷는 방법 |
|---|---|
| 가려는 쪽이 정면에서 30도(`turn_in_place_deg`)보다 벗어남 | 제자리 회전 (15도 `walk_again_deg` 밑으로 오면 다시 걷기) |
| 거의 정면 | 앞으로 걷기, 옆걸음은 최대 `max_side` 0.05 m/s |
| 목표 0.3m(`near_dist`) 안 | 마지막 방향을 보면서 옆걸음(`max_side_near` 0.15)·작은 뒷걸음(`max_back` 0.05)으로 맞춤 |
| 위치 도착 | 제자리에서 마지막 방향 맞추기 → ARRIVED |

**한 번 정한 쪽 유지**
- 꼬깔이 정면에 있으면 왼쪽·오른쪽으로 돌아가는 길 길이가 비슷해서, 꼬깔 위치가 조금만 흔들려도 A-star가 매번 다른 쪽을 고름 → 왔다 갔다 하다 박음
- 그래서 다시 계획할 때 지금 길에서 40cm 넘게 벗어난 칸은 비용을 (1 + `keep_side_cost`)배로 함 (기본 0.5, 0 = 안 씀)
- 꼬깔이 조금 움직이면 같은 쪽으로 살짝 옮긴 길을 고르고, 그쪽이 막히면 다른 쪽으로 감

**꼬깔 모서리 안 깎기**
- 로봇에서 당근까지 직선이 꼬깔 범위를 지나면 당근을 한 칸씩 가까이 당김
- 경로 끝에서도 목표까지 직선이 안전할 때만 목표로 곧장 감 (목표가 꼬깔 바로 뒤면 경로 따라 돌아감)
- 지금 걷는 방향으로 1초 갔을 때 꼬깔 범위에 들어가면 걷지 않고 먼저 돎 (몸이 향한 쪽으로 걸으니까)

**도착 판단**
- 목표에서 `tol_dist`(기본 0.15m) 안, 방향 차이 `tol_yaw_deg`(기본 15도) 안이면 도착
- **도착하면 끝** (`arrive_latch` 2배): 한 번 도착하면 위치가 흔들려도 허용 범위의 2배(30cm, 30도)를 넘기 전까지는 다시 안 움직임
  → 랜드마크가 안 보여서 내 위치가 조금씩 튀어도 목표 근처에서 잔발로 왔다 갔다 하지 않음
- 정말로 밀려서 2배를 넘으면 다시 걸어감. 새 목표가 오면 래치 풀림. `arrive_latch` 0 = 래치 끔

## 꼬깔 기억

Localization(`world_model`)은 지금 카메라에 보이는 꼬깔만 필드 기준으로 알려줌. 그대로 쓰면 고개를 돌리는 순간 꼬깔이 사라짐. 그래서:

1. 받은 꼬깔을 기억
2. 같은 꼬깔을 다시 보면 위치를 조금씩 보정 (`merge_dist` 0.5m 안 = 같은 꼬깔)
3. 두 번 이상 본 꼬깔만 인정 (`min_hits`, 한 번 잘못 본 것 무시)
4. 카메라에 보여야 할 곳인데 `forget_time`초 동안 안 보이면 지움 (카메라 시야 `view_angle_deg`, `view_range`는 실제 카메라와 맞추기)
5. 카메라 밖이어도 `max_unseen_time`초(기본 10초) 동안 다시 안 보이면 지움 (유령 꼬깔 방지)
6. 필드 밖에 보인 꼬깔은 버림
7. UI **꼬깔 기억 지우기** 버튼: 한 번에 지우기 (내 위치가 크게 틀렸다가 다시 잡혔을 때)

꼬깔 위치를 **고정하지 않음**: 내 위치(특히 방향)가 틀린 순간에 고정되면 틀린 위치가 계속 남기 때문. 다시 보면 새 위치로 바뀜 (RoboCup Bit-Bots도 같은 방식)

## 필드 밖으로 안 나가기

필드 밖으로 나가면 30초 페널티. 그래서 필드 선 안쪽 `boundary_margin`(기본 0.2m) 띠를 A-star 지도에서 막음.

- 로봇 반지름 0.15m → 몸이 선 밖으로 안 나가고 5cm 여유
- 목표가 선 근처(띠 안)나 필드 밖이면 `BLOCKED`
- 시작 위치가 선 위나 밖(세팅 위치)이어도 안쪽으로 빠져나와서 감
- UI의 주황 안쪽 테두리 = 로봇 중심이 다닐 수 있는 범위

## Ready 때 RIGHT 진영 금지

규정: Ready 이동 중 RIGHT 진영에 들어가면 실패. 그래서 `nav_command`의 `forbid_right_half`가 true면

- 로봇 중심이 가운데선에서 `center_margin`(기본 0.2m) 왼쪽까지만 다님 (몸 반지름 0.15m → 5cm 여유)
- RIGHT 진영 목표는 `BLOCKED`, 이미 RIGHT 진영에 있으면 왼쪽으로 빠져나옴
- Playing이 되면 false로 보내서 제한을 풂
- UI: **Ready** 버튼 = RIGHT 진영 금지 + Ready 위치로 감, 도착하면 Text에 **준비 완료**. 켜지면 다닐 수 있는 범위가 빨간 테두리
- Ready 위치는 UI의 "Ready 위치" 줄에서 바꿈 (LEFT 진영이면 자유, 기본 robot1 (-0.8, -1.2), robot2 (-0.8, 1.2))

## 로봇 2대 서로 피하기

1. 다른 로봇을 **움직이는 꼬깔**로 보고 A-star가 돌아감 (다른 로봇이 15cm 넘게 움직이면 바로 다시 계획)
2. **양보**: `yield_to_teammate: True`인 로봇(robot2)은 다른 로봇이 가까이(`yield_dist` 0.9m), 앞쪽에서, 움직이고 있으면 멈춤 → `WAITING`
3. **안전 정지**: 다른 로봇이 바로 앞(`stop_dist` 0.45m)에 붙으면 누구든 멈춤
4. 목표에 다른 로봇이 서 있어서 길이 없으면, 다른 로봇을 빼고 경로를 짬 (곧 비킬 테니까)

다른 로봇 위치(`teammate_pose`)는 UDP 메시지가 정해지면 연결

## 공 앞으로 가기

- 공은 꼬깔처럼 장애물로 피함 (공 주변 0.37m 안으로 안 들어감 = 공을 안 밀음). 목표가 그 안이면 `BLOCKED`
- 내비게이션은 **공 앞**(공에서 찰 방향 반대로 "공 앞 거리" 뒤, 찰 방향 바라보기)까지만 데려감
- 그 안쪽 킥 정렬은 **Taskplanner**가 맡음 (그동안 `nav_enable = false`로 내비게이션을 멈춤)
- UI: **공 앞 거리**(기본 0.5m) + **공 앞으로** 버튼 = 상대 골대 쪽을 보고 공 앞에 섬

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
- 회전이 좌우로 왔다 갔다 하면 Kp 회전, 최대 회전을 낮추거나 Kd 회전을 조금 올림

## 실행 중에 튜닝 값 바꾸기

```bash
ros2 param set /navigation_node tol_yaw_deg 10.0
ros2 param list /navigation_node
```

- 실행 중 변경 가능: PID, 속도, 허용 오차, 도착 래치(`arrive_latch`), 크기·여유, 걷는 방식, 다른 로봇 피하기, ik_max_x/y/yaw (UI 탭에 있는 값 포함)
- 시작할 때만: rate, field_length, field_width, grid_resolution, view_angle_deg, view_range, merge_dist, min_hits, forget_time, max_unseen_time
- 잘못된 값(음수 등)은 거부됨. 껐다 켜면 navigation_node.py 기본값으로 돌아감

## 테스트

```bash
cd ~/nav_ws/src/robit_navigation
python3 -m pytest test
```
