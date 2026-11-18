"""
우리 팀 다른 로봇 피하기 (ROS2와 상관없는 순수 계산)

1. 다른 로봇은 "움직이는 꼬깔"로 보고 A-star가 돌아가게 한다 (navigation_node에서 장애물에 추가)
2. 둘 다 서로 피하려고만 하면 정면으로 마주칠 때 같이 우물쭈물하거나 부딪힌다. 그래서 양보 규칙:
   양보하는 로봇(yield_to_teammate = True)은 다른 로봇이
     - 가까이 있고 (yield_dist 안)
     - 내가 가려는 쪽에 있고 (바로 뒤가 아니고)
     - 움직이는 중이면
   잠깐 멈춘다 (WAITING). 다른 로봇은 멈춘 나를 꼬깔처럼 돌아서 지나간다.
   다른 로봇이 멈춰 있으면 양보하지 않는다 (서 있는 로봇 때문에 계속 기다리지 않게).
3. 목표에 다른 로봇이 서 있어서 길이 없으면, 다른 로봇을 빼고 경로를 짠다 (곧 비킬 테니까)
4. 안전 정지: 다른 로봇이 바로 앞(stop_dist 안)에 있으면 누구든 멈춘다 (부딪히지 않게)
"""
import math

from robit_navigation.ball_approach import plan_path
from robit_navigation.nav_core import normalize_angle


def plan_with_teammate(planner, start, goal, cones, mate, mate_radius, ball=None,
                       ball_radius=0.07, approach_margin=0.2):
    """다른 로봇을 꼬깔처럼 넣어서 경로 계획. 다른 로봇 때문에 길이 없으면 빼고 다시"""
    if mate is not None:
        path = plan_path(planner, start, goal, list(cones) + [(mate[0], mate[1], mate_radius)],
                         ball, ball_radius, approach_margin)
        if path is not None:
            return path
    return plan_path(planner, start, goal, cones, ball, ball_radius, approach_margin)


def ahead(robot, toward, point, half_angle):
    """point가 내가 가려는 방향(toward)에서 half_angle 안쪽에 있는지"""
    heading = math.atan2(toward[1] - robot[1], toward[0] - robot[0])
    bearing = math.atan2(point[1] - robot[1], point[0] - robot[0])
    return abs(normalize_angle(bearing - heading)) < half_angle


def too_close(robot, toward, mate, stop_dist=0.45, half_angle=math.radians(45.0)):
    """안전 정지: 다른 로봇이 바로 앞에 붙어 있으면 True"""
    if mate is None:
        return False
    return (math.hypot(mate[0] - robot[0], mate[1] - robot[1]) < stop_dist
            and ahead(robot, toward, mate, half_angle))


def look_ahead_point(path, robot, points):
    """경로에서 로봇과 가장 가까운 점부터 points개 앞의 점 (= 내가 가려는 쪽)"""
    nearest = min(range(len(path)), key=lambda k: math.hypot(path[k][0] - robot[0], path[k][1] - robot[1]))
    return path[min(nearest + points, len(path) - 1)]


def should_yield(robot, toward, mate, mate_speed, yield_dist=0.9, min_speed=0.03,
                 half_angle=math.radians(100.0)):
    """
    robot: 내 위치 (x, y, ...), toward: 내가 가려는 점 (x, y)
    mate: 다른 로봇 위치 (x, y) 또는 None, mate_speed: 다른 로봇 속도 (m/s)
    반환: 지금 멈춰서 양보해야 하면 True
    """
    if mate is None or mate_speed < min_speed:
        return False                                  # 모르거나 서 있음
    if math.hypot(mate[0] - robot[0], mate[1] - robot[1]) > yield_dist:
        return False                                  # 멀리 있음
    return ahead(robot, toward, mate, half_angle)     # 뒤쪽이 아니면 양보


class SpeedEstimator:
    """위치를 계속 받아서 속도 추정 (부드럽게)"""

    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.prev = None        # (x, y, t)
        self.speed = 0.0

    def update(self, x, y, t):
        if self.prev is not None:
            dt = t - self.prev[2]
            if dt > 1e-3:
                v = math.hypot(x - self.prev[0], y - self.prev[1]) / dt
                self.speed += self.alpha * (v - self.speed)
        self.prev = (x, y, t)
        return self.speed
