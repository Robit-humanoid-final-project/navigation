"""공을 장애물로 피해서 공 앞(공 뒤 일정 거리)까지 가는지 확인"""
import math

from robit_navigation.astar import GridPlanner
from robit_navigation.ball_approach import ball_front_goal, plan_path
from robit_navigation.nav_core import ControllerParams, PathFollower, normalize_angle

BALL_R, ROBOT_R = 0.07, 0.15
FRONT = 0.5                        # 공 앞 거리 (m)
GOAL = (4.5, 0.0)                  # 상대 골대
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


def test_ball_front_goal():
    x, y, yaw = ball_front_goal((2.0, 0.0), GOAL, 0.5)
    assert abs(x - 1.5) < 1e-9 and abs(y) < 1e-9 and abs(yaw) < 1e-9     # 공 왼쪽 0.5m, 오른쪽 보기


def test_go_around_ball_to_front():
    # 로봇이 공 오른쪽(골대 쪽)에 있으면 공을 돌아서 공 앞(왼쪽)으로 감, 공은 안 밀음
    ball = (1.0, 1.0)
    goal = ball_front_goal(ball, GOAL, FRONT)
    ok, pose, gap = simulate((2.0, 1.0, R(180)), goal, ball)
    assert ok and gap > 0.0
    assert abs(normalize_angle(pose[2] - goal[2])) < R(10)                 # 골대 쪽 바라봄


def test_without_avoidance_pushes_ball():
    # 비교: 공 피하기를 끄면 공을 뚫고 지나감
    ball = (1.0, 1.0)
    _, _, gap = simulate((2.0, 1.0, R(180)), ball_front_goal(ball, GOAL, FRONT), ball, avoid_ball=False)
    assert gap < 0.0


def test_goal_inside_ball_zone_goes_to_edge():
    # 공 바로 뒤 0.3m(킥 자리)는 공 위험 범위(0.37m) 안이라 못 감
    # -> 공 범위 바로 바깥(목표에서 가장 가까운 곳)까지 가고, 그 안쪽은 Vision이 맡음
    pl = make_planner()
    path = plan_path(pl, (-1.0, 0.0), (1.7, 0.0), [], (2.0, 0.0), BALL_R)
    assert path is not None and pl.goal_moved
    assert math.hypot(path[-1][0] - 2.0, path[-1][1]) >= BALL_R - 0.05     # 공 범위 바깥
    assert math.hypot(path[-1][0] - 1.7, path[-1][1]) < 0.2                # 목표 근처


def test_goal_on_cone_far_away_still_moves():
    # 멀리 있는 목표에 꼬깔이 잠깐 겹쳐도 길 없음(None)이 아니라 가까운 곳으로 가는 길을 줌
    pl = make_planner()
    pl.set_obstacles([(2.0, 0.0, 0.1)])
    path = pl.plan((-3.0, 0.0), (2.1, 0.0))
    assert path is not None and pl.goal_moved
    assert math.hypot(path[-1][0] - 2.1, path[-1][1]) < 0.5


def test_ball_in_the_way_is_avoided():
    ball = (0.0, 0.0)
    ok, _, gap = simulate((-2.0, 0.0, 0.0), (2.0, 0.0, 0.0), ball)
    assert ok and gap > 0.0


def test_ball_front_with_cones():
    ball = (1.5, 1.0)
    cones = [(1.0, 0.0, 0.1), (2.2, -0.5, 0.1), (2.5, 1.5, 0.1)]
    ok, _, gap = simulate((-2.0, -2.0, R(90)), ball_front_goal(ball, GOAL, FRONT), ball, cones)
    assert ok and gap > 0.0


def test_all_directions():
    # 로봇 출발 방향 12가지 x 공 앞 방향(찰 방향) 12가지 = 144가지 모두 공을 안 밀고 도착
    ball = (-1.8, 1.4)
    for kick_deg in range(0, 360, 30):
        aim = (ball[0] + math.cos(R(kick_deg)), ball[1] + math.sin(R(kick_deg)))
        goal = ball_front_goal(ball, aim, FRONT)
        for s_deg in range(0, 360, 30):
            s = R(s_deg)
            start = (ball[0] + 1.2 * math.cos(s), ball[1] + 1.2 * math.sin(s), s + math.pi)
            ok, _, gap = simulate(start, goal, ball)
            assert ok and gap > 0.0, (kick_deg, s_deg)
