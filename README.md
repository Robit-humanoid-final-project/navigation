# robit_navigation_cpp (C++ 버전)

꼬깔을 A-star로 피해서 목표까지 가는 내비게이션 노드 + RViz2 시뮬레이션.
Python 버전(`robit_navigation`)과 구조, 토픽, 동작이 같음.
Ubuntu 24.04 + ROS2 Jazzy 기준.

## 구성

| 파일 | 내 담당? | 하는 일 |
|---|---|---|
| `include/.../nav_core.hpp`, `src/nav_core.cpp` | ✅ | 좌표 변환, P 제어, 도착 판단, 당근(Carrot) 따라가기 |
| `include/.../astar.hpp`, `src/astar.cpp` | ✅ | 꼬깔 부풀리기 + 격자 A-star 경로 계획 |
| `src/navigation_node.cpp` | ✅ | ROS2 노드: 위치·목표·꼬깔 받기 → 경로 계획 → 걸음 명령·상태 보내기 |
| `src/fake_robot_node.cpp` | ❌ 연습용 | 가짜 로봇 + 가짜 꼬깔 (Walking, Localization, Vision 흉내) + RViz2 그림 |
| `test/test_core.cpp` | | 계산 확인 테스트 (ROS2 없이 실행) |
| `launch/sim.launch.py` | | 노드 2개 + RViz2 한 번에 실행, 꼬깔 위치도 여기서 바꿈 |

## 토픽 (임시 이름, 팀 합의 후 변경)

| 토픽 | 메시지 | 방향 | 내용 |
|---|---|---|---|
| `/robot_pose` | geometry_msgs/Pose2D | 받음 | 내 위치 x, y [m], theta [rad] |
| `/nav_goal` | geometry_msgs/Pose2D | 받음 | 목표 x, y [m], theta [rad] |
| `/obstacles` | geometry_msgs/PoseArray | 받음 | 꼬깔 위치 (필드 기준) [m] |
| `/nav_cmd` | geometry_msgs/Twist | 보냄 | linear.x 앞, linear.y 왼쪽 [m/s], angular.z 회전 [rad/s] |
| `/nav_status` | std_msgs/String | 보냄 | IDLE / MOVING / ARRIVED / NO_POSE / BLOCKED |
| `/nav_path` | nav_msgs/Path | 보냄 | A-star 경로 (RViz2 표시용) |
| `/nav_debug` | visualization_msgs/MarkerArray | 보냄 | 부풀린 꼬깔 범위, 당근 (RViz2 표시용) |

## 빌드

```bash
# robit_navigation_cpp 폴더를 ~/nav_ws/src/ 안에 넣은 뒤
cd ~/nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행

터미널 1:
```bash
ros2 launch robit_navigation_cpp sim.launch.py
```

터미널 2:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /nav_goal geometry_msgs/msg/Pose2D "{x: -1.5, y: 0.0, theta: 0.0}"
ros2 topic pub --once /nav_goal geometry_msgs/msg/Pose2D "{x: 3.0, y: 0.0, theta: 0.0}"
```

## 계산 테스트 (ROS2 없이)

```bash
~/nav_ws/install/robit_navigation_cpp/lib/robit_navigation_cpp/test_core
```

## C++ 코드를 고친 뒤

C++은 Python과 달리 고칠 때마다 다시 빌드해야 함:
```bash
cd ~/nav_ws
colcon build --symlink-install
source install/setup.bash
```
