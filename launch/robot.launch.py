"""
실제 로봇에서 실행: 내비게이션 (+ UI)
  받기: nav_command (Taskplanner 또는 UI), world_model (Localization), teammate_pose (아직 없음)
  보내기: master2ik (보행, nav_enable = true일 때만), nav_feedback (Taskplanner)
  먼저 Localization, 보행(ik_walk) 켜기

실행:
  ros2 launch robit_navigation robot.launch.py                  # robot1, UI 같이
  ros2 launch robit_navigation robot.launch.py robot:=robot2    # robot2
  ros2 launch robit_navigation robot.launch.py gui:=false       # UI 없이 (Taskplanner가 명령)

UI: 목표 클릭·Go·Ready·Playing·공 앞으로·Stop. Stop·UI 닫기 = nav_enable false + 보행 멈춤(flag 0)
Taskplanner와 같이 돌릴 때는 UI 버튼을 누르지 말기 (nav_command가 섞임)
값(PID, ik_max 등)은 navigation_node.py 의 기본값을 씀 (UI 탭이나 ros2 param set 으로 실행 중 변경 가능)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 준비 위치 (x, y, 각도): Ready 때 걸어갈 곳 (LEFT 진영이면 자유)
#   yield: 두 로봇이 마주칠 때 양보하는 로봇
ROBOTS = {
    'robot1': {'ready': (-0.8, -1.2, 0.0), 'yield': False},
    'robot2': {'ready': (-0.8, 1.2, 0.0), 'yield': True},
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
        x, y, yaw = r['ready']
        nodes.append(Node(
            package='robit_navigation',
            executable='nav_gui_node',
            output='screen',
            parameters=[{'robot': name, 'ready_x': x, 'ready_y': y, 'ready_yaw_deg': yaw}],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='robot1', description='robot1 또는 robot2'),
        DeclareLaunchArgument('gui', default_value='true', description='UI 같이 켜기'),
        OpaqueFunction(function=make_nodes),
    ])
