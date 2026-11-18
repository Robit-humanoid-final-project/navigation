"""로봇 2대가 서로를 움직이는 꼬깔처럼 피하는지 확인"""
import math

from robit_navigation.astar import GridPlanner
from robit_navigation.nav_core import ControllerParams, PathFollower, normalize_angle
from robit_navigation.teammate import (SpeedEstimator, look_ahead_point, plan_with_teammate,
                                      should_yield, too_close)

ROBOT_R = 0.15
R = math.radians


def simulate_two(starts, goals, cones=(), max_time=90.0, replan_every=0.5):
    """로봇 2대를 같이 움직임 (로봇 2가 양보). 반환: 둘 다 도착했는지, 두 로봇 몸 사이 최소 거리"""
    planners = [GridPlanner(-4.5, 4.5, -3.0, 3.0, robot_radius=ROBOT_R, margin=0.15) for _ in range(2)]
    followers = [PathFollower(ControllerParams(), 5) for _ in range(2)]
    poses = [list(s) for s in starts]
    paths, done = [None, None], [False, False]
    speeds = [SpeedEstimator(), SpeedEstimator()]     # 각 로봇이 본 "다른 로봇" 속도
    dt, min_gap = 0.1, 1e9
    for k in range(int(max_time / dt)):
        for i in range(2):
            if done[i]:
                continue
            mate = poses[1 - i]
            mate_speed = speeds[i].update(mate[0], mate[1], k * dt)
            if k % int(replan_every / dt) == 0 or paths[i] is None:
                paths[i] = plan_with_teammate(planners[i], poses[i][:2], goals[i][:2], cones,
                                              mate, ROBOT_R) or paths[i]
            if paths[i] is None:
                continue
            toward = look_ahead_point(paths[i], poses[i], 5)
            if too_close(poses[i], toward, mate):
                continue                                  # 안전 정지 (둘 다)
            if i == 1 and should_yield(poses[i], toward, mate, mate_speed):
                continue                                  # 로봇 2: 양보 (멈춤)
            c = followers[i].step(tuple(poses[i]), paths[i], goals[i], dt)
            if c.arrived:
                done[i] = True
                continue
            x, y, yaw = poses[i]
            cs, sn = math.cos(yaw), math.sin(yaw)
            poses[i] = [x + (cs * c.front - sn * c.left) * dt, y + (sn * c.front + cs * c.left) * dt,
                        normalize_angle(yaw + c.turn * dt)]
        gap = math.hypot(poses[0][0] - poses[1][0], poses[0][1] - poses[1][1]) - 2 * ROBOT_R
        min_gap = min(min_gap, gap)
        if all(done):
            return True, min_gap
    return False, min_gap


def test_swap_places_head_on():
    # 같은 줄에서 서로 자리 바꾸기 (정면으로 마주 옴)
    ok, gap = simulate_two([(-3.0, -2.5, R(90)), (-3.0, 2.5, R(-90))],
                           [(-3.0, 2.5, R(90)), (-3.0, -2.5, R(-90))])
    assert ok and gap > 0.0


def test_crossing_paths():
    # 길이 X자로 엇갈림
    ok, gap = simulate_two([(-3.0, -2.0, R(45)), (-3.0, 2.0, R(-45))],
                           [(-0.5, 2.0, 0.0), (-0.5, -2.0, 0.0)])
    assert ok and gap > 0.0


def test_goal_next_to_standing_teammate():
    # 다른 로봇이 서 있는 옆 자리로 가기 (패스 받을 자리 등)
    ok, gap = simulate_two([(-3.0, -2.0, R(90)), (-1.0, 0.0, 0.0)],
                           [(-1.0, 0.8, 0.0), (-1.0, 0.0, 0.0)])
    assert ok and gap > 0.0


def test_should_yield_rules():
    me, toward = (0.0, 0.0), (1.0, 0.0)                        # 나는 +x 쪽으로 가는 중
    assert should_yield(me, toward, (0.5, 0.0), 0.3)           # 앞에서 움직임 -> 양보
    assert not should_yield(me, toward, (0.5, 0.0), 0.0)       # 앞에 서 있음 -> 양보 안 함 (돌아감)
    assert not should_yield(me, toward, (-0.5, 0.0), 0.3)      # 바로 뒤 -> 양보 안 함
    assert not should_yield(me, toward, (2.0, 0.0), 0.3)       # 멀리 -> 양보 안 함


def test_safety_stop():
    me, toward = (0.0, 0.0), (1.0, 0.0)
    assert too_close(me, toward, (0.4, 0.0))                  # 바로 앞에 붙음 -> 멈춤
    assert not too_close(me, toward, (0.0, 0.4))              # 옆 -> 그냥 감
    assert not too_close(me, toward, (0.8, 0.0))              # 충분히 멂
