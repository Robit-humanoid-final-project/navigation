"""공을 밀지 않고 킥 자리에 서는지 확인"""
import math

from robit_navigation.astar import GridPlanner
from robit_navigation.ball_approach import plan_path
from robit_navigation.nav_core import ControllerParams, PathFollower, normalize_angle

BALL_R, ROBOT_R = 0.07, 0.15
R = math.radians


def make_planner():
    return GridPlanner(-4.5, 4.5, -3.0, 3.0, resolution=0.1, robot_radius=ROBOT_R, margin=0.15)


def simulate(start, goal, ball, cones=(), avoid_ball=True, max_time=60.0):
    """반환: 도착 여부, 마지막 위치, 로봇 몸과 공·꼬깔 사이 최소 거리 (음수 = 닿음)"""
    planner = make_planner()
    follower = PathFollower(ControllerParams(), 5)
    x, y, yaw = start
    dt, min_gap, path = 0.1, 1e9, None
    for k in range(int(max_time / dt)):
        if k % 10 == 0:
            path = plan_path(planner, (x, y), goal[:2], list(cones),
                             ball if avoid_ball else None, BALL_R)
            assert path is not None
        c = follower.step((x, y, yaw), path, goal, dt)
        if c.arrived:
            return True, (x, y, yaw), min_gap
        cs, sn = math.cos(yaw), math.sin(yaw)
        x += (cs * c.front - sn * c.left) * dt
        y += (sn * c.front + cs * c.left) * dt
        yaw = normalize_angle(yaw + c.turn * dt)
        min_gap = min(min_gap, math.hypot(x - ball[0], y - ball[1]) - ROBOT_R - BALL_R,
                      *[math.hypot(x - cx, y - cy) - ROBOT_R - cr for cx, cy, cr in cones])
    return False, (x, y, yaw), min_gap


def test_go_around_ball_to_kick_spot():
    # 로봇이 공 앞쪽(+x)에 있고, 공을 +x로 차려면 공 뒤(-x)로 돌아가야 함
    ball = (0.0, 1.0)
    kick = (ball[0] - 0.3, ball[1], 0.0)        # 공 뒤 30cm, +x 쪽 바라보기
    ok, pose, gap = simulate((1.0, 1.0, R(180)), kick, ball)
    assert ok
    assert gap > 0.0                            # 한 번도 공에 닿지 않음
    assert abs(normalize_angle(pose[2])) < R(10)   # 찰 방향을 보고 섬


def test_without_avoidance_pushes_ball():
    # 비교: 공 피하기를 끄면 공을 뚫고 지나감 (공에 닿음)
    ball = (0.0, 1.0)
    _, _, gap = simulate((1.0, 1.0, R(180)), (ball[0] - 0.3, ball[1], 0.0), ball, avoid_ball=False)
    assert gap < 0.0


def test_already_behind_ball_goes_straight():
    # 이미 공 뒤 진입로 위에 있으면 바로 들어감 (뒤로 물러나지 않음)
    planner = make_planner()
    ball = (0.0, 1.0)
    path = plan_path(planner, (-1.0, 1.0), (-0.3, 1.0), [], ball, BALL_R)
    assert all(abs(py - 1.0) < 1e-9 for _, py in path)      # 일직선
    assert all(px >= -1.0 - 1e-9 for px, _ in path)          # 뒤로 안 감


def test_ball_in_the_way_is_avoided():
    # 목표가 공에서 멀면 공은 그냥 장애물: 돌아서 감
    ball = (0.0, 0.0)
    ok, _, gap = simulate((-2.0, 0.0, 0.0), (2.0, 0.0, 0.0), ball)
    assert ok and gap > 0.0


def test_kick_spot_with_cones():
    # 꼬깔이 있어도 공 뒤로 돌아가기
    ball = (1.5, 0.5)
    cones = [(1.0, 0.0, 0.1), (2.2, -0.5, 0.1)]
    ok, _, gap = simulate((2.5, 1.0, R(180)), (ball[0] - 0.3, ball[1], 0.0), ball, cones)
    assert ok and gap > 0.0


def test_all_directions():
    # 찰 방향 12가지 x 로봇 출발 방향 12가지 = 144가지 모두 공을 안 밀고 도착
    ball = (-1.8, 1.4)
    for kick_deg in range(0, 360, 30):
        k = R(kick_deg)
        goal = (ball[0] - 0.3 * math.cos(k), ball[1] - 0.3 * math.sin(k), k)
        for s_deg in range(0, 360, 30):
            s = R(s_deg)
            start = (ball[0] + math.cos(s), ball[1] + math.sin(s), s + math.pi)
            ok, _, gap = simulate(start, goal, ball)
            assert ok and gap > 0.0, (kick_deg, s_deg)


def test_goal_touching_ball_is_rejected():
    # 목표에 서기만 해도 몸이 공에 닿으면 경로 없음 (BLOCKED)
    assert plan_path(make_planner(), (1.0, 1.0), (0.2, 1.0), [], (0.0, 1.0), BALL_R) is None


def test_ball_right_behind_cone():
    # 공이 꼬깔 바로 뒤에 있고, 꼬깔 위치를 몇 cm 잘못 알아도 (Vision 오차) 길을 찾음
    ball = (2.0, 0.0)
    for cone_x in (1.0, 1.03, 1.08, 1.15):
        cones = [(-1.0, -1.2, 0.1), (cone_x, 0.0, 0.1), (2.2, -0.5, 0.1)]
        ok, _, gap = simulate((-3.0, -3.0, R(90)), (1.7, 0.0, 0.0), ball, cones)
        assert ok and gap > 0.0, cone_x       # 공에도 꼬깔에도 안 닿음


def test_kick_spot_inside_cone_is_rejected():
    # 킥 자리가 꼬깔 범위 안이면 경로 없음 (BLOCKED)
    cones = [(1.5, 0.0, 0.1)]
    assert plan_path(make_planner(), (-1.0, 0.0), (1.7, 0.0), cones, (2.0, 0.0), BALL_R) is None
