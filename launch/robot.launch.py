"""
실제 로봇에서 실행: 내비게이션 1개 (+ UI)
  시뮬레이션(sim.launch.py)과 다르게 가짜 로봇이 없고, 토픽 이름도 팀 코드와 같음
    받기: nav_command (Taskplanner 또는 UI), world_model (Localization), teammate_pose (아직 없음)
    보내기: master2ik (보행), nav_feedback (Taskplanner)
  먼저 Localization, 보행(ik_walk) 켜기

실행:
  ros2 launch robit_navigation robot.launch.py                  # robot1, UI 같이
  ros2 launch robit_navigation robot.launch.py robot:=robot2    # robot2
  ros2 launch robit_navigation robot.launch.py gui:=false       # UI 없이 (Taskplanner가 명령)

UI 쓸 때: 목표 끌기·Go·Ready·공 앞으로·Stop 가능, UI를 닫으면 로봇 멈춤 (nav_enable = false)
값(PID, ik_max 등)은 navigation_node.py 의 기본값을 씀 (UI 탭이나 ros2 param set 으로 실행 중 변경 가능)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 세팅 위치, 준비 위치 (x, y, 각도) - sim.launch.py 와 같게
ROBOTS = {
    'robot1': {'start': (-2.25, -3.1, 90.0), 'ready': (-0.8, -1.2, 0.0), 'yield': False},
    'robot2': {'start': (-2.25, 3.1, -90.0), 'ready': (-0.8, 1.2, 0.0), 'yield': True},
}


def make_nodes(context):
    name = LaunchConfiguration('robot').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')
    if name not in ROBOTS:
        raise RuntimeError(f'robot 은 {list(ROBOTS)} 중 하나 (받은 값: {name})')
    r = ROBOTS[name]
    nodes = [
        Node(
            package='robit_navigation',
            executable='navigation_node',
            output='screen',
            parameters=[{'yield_to_teammate': r['yield']}],
        ),
    ]
    if gui:
        nodes.append(Node(
            package='robit_navigation',
            executable='nav_gui_node',
            output='screen',
            parameters=[{
                'real_robot': True,
                'robots': [name],
                'starts_x': [r['start'][0]], 'starts_y': [r['start'][1]], 'starts_yaw_deg': [r['start'][2]],
                'readys_x': [r['ready'][0]], 'readys_y': [r['ready'][1]], 'readys_yaw_deg': [r['ready'][2]],
            }],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='robot1', description='robot1 또는 robot2'),
        DeclareLaunchArgument('gui', default_value='true', description='UI 같이 켜기'),
        OpaqueFunction(function=make_nodes),
    ])
