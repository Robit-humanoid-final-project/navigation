"""
시뮬레이션 한 번에 실행: 로봇 2대 (각각 내비게이션 + 가짜 로봇) + Navigation UI
  팀 메시지(humanoid_interfaces) 사용: UI가 Taskplanner 대신 nav_command 를 보내고,
  가짜 로봇이 Localization 대신 world_model 을 보냄
실행: ros2 launch robit_navigation sim.launch.py
     UI의 "공 앞으로" 버튼 = 고른 로봇이 공 앞(공에서 골대 반대쪽으로 "공 앞 거리" 뒤)으로 감

로봇마다 이름(robot1, robot2)이 붙어서 토픽도 따로 생김
  예) /robot1/nav_command, /robot1/world_model, /robot1/nav_feedback, /robot2/nav_command ...
같이 쓰는 토픽: /ball_pose (공), /clicked_point (공 옮기기)
서로의 위치: robot1의 teammate_pose = /robot2/robot_pose, robot2의 teammate_pose = /robot1/robot_pose
"""
from launch import LaunchDescription
from launch_ros.actions import Node

# 모든 노드가 같은 값을 써야 하는 것들
FIELD = {'field_length': 9.0, 'field_width': 6.0}   # 경기장 9 x 6m
CONE_RADIUS = 0.1     # 꼬깔: 바닥 14 x 14cm 네모를 덮는 원 (m)
ROBOT_RADIUS = 0.15   # 로봇: 25(앞면) x 17cm 네모를 덮는 원 (m)
BALL_RADIUS = 0.07    # 공: 지름 14cm (m)
CAMERA = {'view_angle_deg': 60.0, 'view_range': 3.0}   # 카메라 시야 (Vision과 맞추기)
# 규정: 공과 장애물(3개)은 RIGHT 진영에 놓임 (경기마다 바뀜, 여기서 바꿔서 연습)
CONES = {'cones_x': [1.5, 2.5, 3.2], 'cones_y': [-1.0, 0.8, -0.3]}   # 가짜 꼬깔 위치
BALL = {'ball_x': 1.0, 'ball_y': 0.5}                                  # 공 처음 위치

# 로봇 2대
#   start   : 세팅 위치 (x, y, 각도) = LEFT 진영 위·아래 라인의 절반 지점, 라인보다 뒤 (규정 그림)
#   has_ball: 공은 가짜 로봇 하나만 가짐 (두 로봇 모두 공을 밀 수 있음)
#   ready   : 준비 위치 (x, y, 각도) = Ready 때 스스로 걸어가서 멈출 곳 (LEFT 진영이면 자유, UI에서 바꿀 수 있음)
#   yield   : 정면으로 마주칠 때 양보하는 로봇
ROBOTS = [
    {'name': 'robot1', 'start': (-2.25, -3.1, 90.0), 'ready': (-0.8, -1.2, 0.0),
     'mate': 'robot2', 'has_ball': True, 'yield': False},
    {'name': 'robot2', 'start': (-2.25, 3.1, -90.0), 'ready': (-0.8, 1.2, 0.0),
     'mate': 'robot1', 'has_ball': False, 'yield': True},
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
    # 앞걸음 위주 (휴머노이드는 옆걸음·뒷걸음이 느리고 잘 넘어짐)
    'turn_in_place_deg': 30.0,
    'walk_again_deg': 15.0,
    'near_dist': 0.3,
    'max_side': 0.05,
    'max_side_near': 0.15,
    'max_back': 0.05,
    'grid_resolution': 0.1,
    'robot_radius': ROBOT_RADIUS,
    'margin': 0.15,
    'center_margin': 0.2,      # Ready(LEFT 진영만) 때 로봇 중심이 가운데선에서 떨어져야 하는 거리 (m)
    'boundary_margin': 0.2,    # 로봇 중심이 필드 선에서 떨어져야 하는 거리 (m), 필드 밖 = 30초 페널티
    'cone_radius': CONE_RADIUS,
    'carrot_points': 5,
    'replan_period': 1.0,
    # 공 (공 피하기 켜기/끄기는 Taskplanner가 NavCommand.avoid_ball 로)
    'ball_radius': BALL_RADIUS,
    # Localization의 is_localized 가 구현되면 True 로 (위치를 못 믿으면 멈춤)
    'require_localized': False,
    # 다른 로봇 피하기
    'teammate_radius': ROBOT_RADIUS,
    'yield_dist': 0.9,         # 양보하는 로봇: 다른 로봇이 이 거리 안에서 움직이면 멈춤 (m)
    'stop_dist': 0.45,         # 다른 로봇이 바로 앞 이 거리 안이면 안전 정지 (m)
    # 보행(master2ik) 최대값: 임시 값 (보행 담당 답 오면 바꾸기, 음수 = 방향 반대)
    'ik_max_x': 25.0,
    'ik_max_y': 25.0,
    'ik_max_yaw': 10.0,
    # 꼬깔 기억
    'merge_dist': 0.5,
    'min_hits': 2,
    'forget_time': 2.0,
    'max_unseen_time': 10.0,
    # 꼬깔 고정: 멈춰 있을 때 꾸준히 보인 꼬깔을 고정 (꼬깔은 경기 중 안 움직임)
    'still_time': 1.0,
    'lock_hits': 10,
    'lock_spread': 0.15,
    'max_locked': 3,
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
                **BALL,                # 가짜 공 처음 위치 (필드 기준, m)
                'ball_radius': BALL_RADIUS,
                'vision_noise': 0.03,  # 가짜 Vision 위치 오차 (m)
            }],
            remappings=shared + [('clicked_point', '/clicked_point')],
        ),
    ]


def generate_launch_description():
    nodes = []
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
                'readys_x': [r['ready'][0] for r in ROBOTS],
                'readys_y': [r['ready'][1] for r in ROBOTS],
                'readys_yaw_deg': [r['ready'][2] for r in ROBOTS],
            }],
        ),
    ]
    return LaunchDescription(nodes)
