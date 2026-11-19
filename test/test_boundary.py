"""필드 밖으로 안 나가는지 확인 (필드 9 x 6m, 로봇 중심은 선에서 0.2m 안쪽)"""
import math

from robit_navigation.astar import GridPlanner
from robit_navigation.nav_core import ControllerParams, PathFollower, normalize_angle

ROBOT_R, EDGE = 0.15, 0.2
R = math.radians


def make_planner(cones=()):
    p = GridPlanner(-4.5, 4.5, -3.0, 3.0, robot_radius=ROBOT_R, margin=0.15, boundary_margin=EDGE)
    p.set_obstacles(list(cones))
    return p


def test_path_stays_inside():
    p = make_planner([(0.0, -2.2, 0.1)])                 # 선 근처 꼬깔: 선 쪽으로 돌아가면 안 됨
    path = p.plan((-3.0, -2.6), (3.0, -2.6))
    assert path is not None
    assert all(p.edge_distance(x, y) >= EDGE - 1e-9 for x, y in path[1:])   # 출발점 빼고 모두 안쪽
    assert all(p.is_free_point(x, y) for x, y in path[1:])


def test_goal_on_line_is_rejected():
    assert make_planner().plan((0.0, 0.0), (4.45, 0.0)) is None   # 목표가 선 바로 옆
    assert make_planner().plan((0.0, 0.0), (0.0, 3.5)) is None    # 목표가 필드 밖


def test_start_on_line_can_escape():
    # 시작 위치가 선 위여도 (세팅 위치 등) 안쪽으로 빠져나와서 감
    path = make_planner().plan((-3.0, -3.0), (-1.5, 0.0))
    assert path is not None
    assert path[1][1] > -3.0                              # 첫 걸음은 안쪽으로


def test_robot_body_never_leaves_field():
    # 선을 따라 길게 이동해도 로봇 몸이 필드 밖으로 안 나감
    p = make_planner([(-1.0, -2.3, 0.1), (1.5, -2.4, 0.1)])
    follower = PathFollower(ControllerParams(), 5)
    x, y, yaw = -3.5, -2.7, 0.0
    goal = (3.5, -2.6, 0.0)
    worst = 1e9
    for k in range(600):
        if k % 10 == 0:
            path = p.plan((x, y), goal[:2])
            assert path is not None
        c = follower.step((x, y, yaw), path, goal, 0.1)
        if c.arrived:
            break
        cs, sn = math.cos(yaw), math.sin(yaw)
        x += (cs * c.front - sn * c.left) * 0.1
        y += (sn * c.front + cs * c.left) * 0.1
        yaw = normalize_angle(yaw + c.turn * 0.1)
        worst = min(worst, p.edge_distance(x, y) - ROBOT_R)
    assert c.arrived
    assert worst > 0.0                                    # 몸 가장자리도 선 안쪽


# ---------------- Ready: LEFT 진영만 (가운데선 x = 0 을 넘으면 실패) ----------------
CENTER = 0.2


def test_left_only_path():
    p = make_planner()
    p.set_max_x(-CENTER)
    assert p.plan((-2.25, -2.7), (1.0, 0.0)) is None             # RIGHT 진영 목표는 거부
    path = p.plan((-2.25, -2.7), (-0.3, 2.0))                     # 가운데선 바로 옆 목표
    assert path is not None and all(x <= -CENTER + 1e-9 for x, _ in path[1:])
    p.set_max_x(None)
    assert p.plan((-2.25, -2.7), (1.0, 0.0)) is not None         # 제한 풀면 갈 수 있음


def test_left_only_escape_from_right():
    # 이미 RIGHT 진영에 있으면 왼쪽으로 빠져나옴
    p = make_planner()
    p.set_max_x(-CENTER)
    path = p.plan((0.5, 0.0), (-1.5, 0.0))
    assert path is not None and path[1][0] < 0.5


def test_ready_body_never_crosses_center():
    # 세팅 위치(터치라인)에서 가운데선 근처 준비 위치로: 로봇 몸이 x = 0 을 안 넘음
    for start, goal in [((-2.25, -3.0, R(90)), (-0.4, -0.8, 0.0)),
                        ((-2.25, 3.0, R(-90)), (-0.4, 0.8, 0.0)),
                        ((-2.25, -3.0, R(0)), (-0.35, 2.5, R(90)))]:
        p = make_planner()
        p.set_max_x(-CENTER)
        follower = PathFollower(ControllerParams(), 5)
        x, y, yaw = start
        worst = -1e9
        for k in range(600):
            if k % 10 == 0:
                path = p.plan((x, y), goal[:2])
                assert path is not None
            c = follower.step((x, y, yaw), path, goal, 0.1)
            if c.arrived:
                break
            cs, sn = math.cos(yaw), math.sin(yaw)
            x += (cs * c.front - sn * c.left) * 0.1
            y += (sn * c.front + cs * c.left) * 0.1
            yaw = normalize_angle(yaw + c.turn * 0.1)
            worst = max(worst, x + ROBOT_R)                       # 몸의 가장 오른쪽
        assert c.arrived, (start, goal)
        assert worst < 0.0, (start, goal, worst)
