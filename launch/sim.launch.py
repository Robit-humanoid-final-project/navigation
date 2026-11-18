"""
시뮬레이션 한 번에 실행: 로봇 2대 (각각 내비게이션 + 가짜 로봇) + Navigation UI
실행: ros2 launch robit_navigation sim.launch.py
RViz2도 같이 보기: ros2 launch robit_navigation sim.launch.py rviz:=true

로봇마다 이름(robot1, robot2)이 붙어서 토픽도 따로 생김
  예) /robot1/robot_pose, /robot1/nav_goal, /robot2/robot_pose, /robot2/nav_goal ...
같이 쓰는 토픽: /ball_pose (공), /clicked_point (공 옮기기)
서로의 위치: robot1의 teammate_pose = /robot2/robot_pose, robot2의 teammate_pose = /robot1/robot_pose
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 모든 노드가 같은 값을 써야 하는 것들
FIELD = {'field_length': 9.0, 'field_width': 6.0}   # 경기장 9 x 6m
CONE_RADIUS = 0.1     # 꼬깔: 바닥 14 x 14cm 네모를 덮는 원 (m)
ROBOT_RADIUS = 0.15   # 로봇: 25(앞면) x 17cm 네모를 덮는 원 (m)
BALL_RADIUS = 0.07    # 공: 지름 14cm (m)
CAMERA = {'view_angle_deg': 60.0, 'view_range': 3.0}   # 카메라 시야 (Vision과 맞추기)
CONES = {'cones_x': [-1.0, 1.0, 2.2], 'cones_y': [-1.2, 0.0, -0.5]}   # 가짜 꼬깔 위치

# 로봇 2대: 이름, 시작 위치 (x, y, 각도), 다른 로봇 이름, 공을 가지는지, 양보하는 로봇인지
ROBOTS = [
    {'name': 'robot1', 'start': (-3.0, -3.0, 90.0), 'mate': 'robot2', 'has_ball': True, 'yield': False},
    {'name': 'robot2', 'start': (-3.0, 3.0, -90.0), 'mate': 'robot1', 'has_ball': False, 'yield': True},
]

NAV_PARAMS = {
    **FIELD,
    **CAMERA,
    # PID (ki, kd = 0 이면 P만 / kd만 올리면 PD / 둘 다 올리면 PID)
    'kp_move': 0.8,
    'ki_move': 0.0,
    'kd_move': 0.0,
    'i_limit_move': 0.1,
    'kp_turn': 1.5,
    'ki_turn': 0.0,
    'kd_turn': 0.0,
    'i_limit_turn': 0.3,
    'd_alpha': 0.5,
    'max_move': 0.3,
    'max_turn': 1.0,
    'tol_dist': 0.10,
    'tol_yaw_deg': 10.0,
    'grid_resolution': 0.1,
    'robot_radius': ROBOT_RADIUS,
    'margin': 0.15,
    'cone_radius': CONE_RADIUS,
    'carrot_points': 5,
    'replan_period': 1.0,
    # 공 피하기 (공을 밀지 않고 킥 자리로)
    'avoid_ball': True,
    'ball_radius': BALL_RADIUS,
    'approach_margin': 0.2,
    # 다른 로봇 피하기
    'teammate_radius': ROBOT_RADIUS,
    'yield_dist': 0.9,         # 양보하는 로봇: 다른 로봇이 이 거리 안에서 움직이면 멈춤 (m)
    'stop_dist': 0.45,         # 다른 로봇이 바로 앞 이 거리 안이면 안전 정지 (m)
    # 꼬깔 기억
    'merge_dist': 0.3,
    'min_hits': 2,
    'forget_time': 2.0,
}


def robot_nodes(r):
    """로봇 1대 = 내비게이션 노드 + 가짜 로봇 노드"""
    shared = [('ball_pose', '/ball_pose'),                         # 공은 같이 씀
              ('teammate_pose', f'/{r["mate"]}/robot_pose')]       # 다른 로봇 위치
    x, y, yaw = r['start']
    return [
        Node(
            package='robit_navigation',
            executable='navigation_node',
            namespace=r['name'],
            output='screen',
            parameters=[{**NAV_PARAMS, 'yield_to_teammate': r['yield']}],
            remappings=shared,
        ),
        Node(
            package='robit_navigation',
            executable='fake_robot_node',
            namespace=r['name'],
            output='screen',
            parameters=[{
                **FIELD,
                **CAMERA,
                **CONES,
                'start_x': x,
                'start_y': y,
                'start_yaw_deg': yaw,
                'cone_radius': CONE_RADIUS,
                'robot_radius': ROBOT_RADIUS,
                'has_ball': r['has_ball'],
                'ball_x': 0.0,         # 가짜 공 처음 위치 (필드 기준, m)
                'ball_y': 1.0,
                'ball_radius': BALL_RADIUS,
                'vision_noise': 0.03,  # 가짜 Vision 위치 오차 (m)
                'slip': 0.0,           # 걸을 때 뒤로 미끄러지는 속도 (m/s), PID 실험용
            }],
            remappings=shared + [('clicked_point', '/clicked_point')],
        ),
    ]


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('robit_navigation'), 'rviz', 'sim.rviz')

    nodes = [DeclareLaunchArgument('rviz', default_value='false', description='RViz2도 같이 켤지')]
    for r in ROBOTS:
        nodes += robot_nodes(r)
    nodes += [
        Node(
            package='robit_navigation',
            executable='nav_gui_node',
            output='screen',
            parameters=[{
                'robots': [r['name'] for r in ROBOTS],
                'starts_x': [r['start'][0] for r in ROBOTS],
                'starts_y': [r['start'][1] for r in ROBOTS],
                'starts_yaw_deg': [r['start'][2] for r in ROBOTS],
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]
    return LaunchDescription(nodes)
