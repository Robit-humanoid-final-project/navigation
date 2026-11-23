"""
공 근처 경로 계획 (ROS2와 상관없는 순수 계산)

공은 꼬깔처럼 피해야 할 장애물로 넣는다 (공을 밀지 않게).
  공 위험 범위 = 공 반지름 + 로봇 반지름 + 여유 (기본 0.07 + 0.15 + 0.15 = 0.37m)
  목표가 이 범위 안이면 갈 수 없음 (BLOCKED)

공 바로 앞(킥 자리)에서 골대 방향으로 맞추고 차는 건 Vision이 맡기로 함.
그래서 내비게이션은 "공 앞" = 공에서 찰 방향 반대로 일정 거리(공 앞 거리) 뒤까지만 데려간다.
공 앞 거리는 공 위험 범위보다 커야 함 (실험하면서 정하기, 기본 0.5m)
"""
import math


def plan_path(planner, start, goal, cones, ball=None, ball_radius=0.07):
    """
    planner: GridPlanner, start/goal: (x, y), cones: [(x, y, 반지름), ...]
    ball: (x, y) 또는 None (공을 피하지 않음)
    반환: 경로 점 목록 [(x, y), ...] (못 찾으면 None)
    """
    obstacles = list(cones)
    if ball is not None:
        obstacles.append((ball[0], ball[1], ball_radius))     # 공 = 작은 꼬깔
    planner.set_obstacles(obstacles)
    return planner.plan(start, goal)


def ball_front_goal(ball, aim, dist):
    """
    공 앞 자리: 공에서 aim 쪽으로 찬다고 할 때, 공 뒤 dist 만큼 떨어진 곳 + aim 쪽 바라보기
    ball, aim: (x, y) 필드 기준, dist: 공 앞 거리 (m)
    반환: (x, y, 방향 rad)
    예) 공 (2.0, 0.5), 골대 (4.5, 0.0), 0.5m -> 공에서 골대 반대쪽으로 0.5m 뒤, 골대 쪽 바라보기
    """
    dx, dy = aim[0] - ball[0], aim[1] - ball[1]
    yaw = math.atan2(dy, dx)
    return ball[0] - dist * math.cos(yaw), ball[1] - dist * math.sin(yaw), yaw
