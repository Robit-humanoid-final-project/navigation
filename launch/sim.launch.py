"""
시뮬레이션 한 번에 실행: 내비게이션 노드 + 가짜 로봇(가짜 꼬깔, 가짜 Vision 포함) + RViz2
실행: ros2 launch robit_navigation sim.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# 두 노드가 같은 값을 써야 하는 것들
FIELD = {'field_length': 9.0, 'field_width': 6.0}   # 예시 값
CONE_RADIUS = 0.1                                   # 꼬깔 바닥 반지름 (m)
ROBOT_RADIUS = 0.2                                  # 로봇 몸통 반지름 (m)
BALL_RADIUS = 0.075                                 # 공 반지름 (m)
CAMERA = {'view_angle_deg': 60.0, 'view_range': 3.0}   # 카메라 시야 (Vision과 맞추기)


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('robit_navigation'), 'rviz', 'sim.rviz')

    return LaunchDescription([
        Node(
            package='robit_navigation',
            executable='navigation_node',
            output='screen',
            parameters=[{
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
                # 꼬깔 기억
                'merge_dist': 0.3,
                'min_hits': 2,
                'forget_time': 2.0,
            }],
        ),
        Node(
            package='robit_navigation',
            executable='fake_robot_node',
            output='screen',
            parameters=[{
                **FIELD,
                **CAMERA,
                'start_x': -3.0,
                'start_y': -3.0,
                'start_yaw_deg': 90.0,
                'cones_x': [-1.0, 1.0, 2.2],
                'cones_y': [-1.2, 0.0, -0.5],
                'cone_radius': CONE_RADIUS,
                'robot_radius': ROBOT_RADIUS,
                'ball_x': 0.0,         # 가짜 공 처음 위치 (필드 기준, m)
                'ball_y': 1.0,
                'ball_radius': BALL_RADIUS,
                'vision_noise': 0.03,  # 가짜 Vision 위치 오차 (m)
                'slip': 0.0,           # 걸을 때 뒤로 미끄러지는 속도 (m/s), PID 실험용
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
