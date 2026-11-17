"""A-star와 경로 따라가기 시뮬레이션: 꼬깔을 피해서 도착하는지, PID, 카메라에 보이는 꼬깔만 알 때"""
import math
import random

from robit_navigation.astar import GridPlanner
from robit_navigation.nav_core import ControllerParams, PathFollower, field_to_robot, normalize_angle
from robit_navigation.obstacle_memory import ObstacleMemory

CONES = [(-1.0, -1.2, 0.1), (1.0, 0.0, 0.1), (2.2, -0.5, 0.1)]
R = math.radians


def make_planner(cones=CONES):
    p = GridPlanner(-4.5, 4.5, -3.0, 3.0, resolution=0.1, robot_radius=0.2, margin=0.15)
    p.set_obstacles(cones)
    return p


def test_no_obstacle_is_straight():
    path = GridPlanner(-4.5, 4.5, -3.0, 3.0).plan((-2.0, 0.0), (2.0, 0.0))
    assert path is not None and all(abs(y) < 1e-9 for _, y in path)


def test_path_avoids_cones():
    p = make_planner()
    path = p.plan((-1.5, 0.0), (3.0, 0.0))
    assert path is not None and all(p.is_free_point(x, y) for x, y in path)


def test_blocked():
    assert make_planner().plan((-1.5, 0.0), (1.05, 0.0)) is None           # 목표가 꼬깔 위
    wall = [(0.0, y / 10.0, 0.1) for y in range(-30, 31, 3)]
    assert make_planner(wall).plan((-2.0, 0.0), (2.0, 0.0)) is None        # 꼬깔 벽


def simulate(start, goal, params=None, slip=0.0, max_time=60.0, vision=False):
    """가짜 로봇 시뮬레이션. slip: 걸을 때 뒤로 밀리는 속도, vision: 카메라에 보이는 꼬깔만 앎"""
    planner = make_planner([] if vision else CONES)
    follower = PathFollower(params or ControllerParams(), 5)
    memory = ObstacleMemory()
    rng = random.Random(1)
    x, y, yaw = start
    dt, min_clear, path, need_plan = 0.1, 1e9, None, True
    for k in range(int(max_time / dt)):
        if vision:
            seen = [field_to_robot((x, y, yaw), cx, cy) for cx, cy, _ in CONES
                    if memory.in_view((x, y, yaw), cx, cy)]
            seen = [(f + rng.uniform(-0.03, 0.03), l + rng.uniform(-0.03, 0.03)) for f, l in seen]
            if memory.update((x, y, yaw), seen, k * dt):
                planner.set_obstacles([(cx, cy, 0.1) for cx, cy in memory.confirmed()])
                need_plan = True
        if need_plan or k % 10 == 0:
            path = planner.plan((x, y), goal[:2])
            assert path is not None
            need_plan = False
        c = follower.step((x, y, yaw), path, goal, dt)
        if c.arrived:
            return True, (x, y, yaw), min_clear, k * dt
        vx = c.front - (slip if abs(c.front) + abs(c.left) > 1e-6 else 0.0)
        cs, sn = math.cos(yaw), math.sin(yaw)
        x += (cs * vx - sn * c.left) * dt
        y += (sn * vx + cs * c.left) * dt
        yaw = normalize_angle(yaw + c.turn * dt)
        for cx, cy, r in CONES:
            min_clear = min(min_clear, math.hypot(x - cx, y - cy) - r - 0.2)
    return False, (x, y, yaw), min_clear, max_time


def test_ready_and_playing():
    ok, pose, _, t = simulate((-3.0, -3.0, R(90)), (-1.5, 0.0, 0.0))
    assert ok and pose[0] < 0 and abs(t - 12.8) < 0.5      # RIGHT 진영으로 안 넘어감
    ok, _, clear, _ = simulate((-1.5, 0.0, 0.0), (3.0, 0.0, 0.0))
    assert ok and clear > 0.0                              # 꼬깔에 한 번도 안 닿음


def test_pid_beats_slip():
    ok, pose, _, _ = simulate((-3.0, 0.0, 0.0), (-1.0, 0.0, 0.0), slip=0.1, max_time=30.0)
    assert not ok and abs(math.hypot(pose[0] + 1.0, pose[1]) - 0.125) < 0.01   # P만: 12.5cm 앞에서 멈춤
    pid = ControllerParams(ki_move=0.5, kd_move=0.1, ki_turn=0.3, kd_turn=0.1)
    ok, *_ = simulate((-3.0, 0.0, 0.0), (-1.0, 0.0, 0.0), params=pid, slip=0.1, max_time=30.0)
    assert ok                                              # PID: 도착


def test_camera_only():
    for start, goal in [((-3.0, -3.0, R(90)), (-1.5, 0.0, 0.0)),
                        ((-1.5, 0.0, 0.0), (3.0, 0.0, 0.0)),
                        ((3.0, 0.0, R(180)), (-1.5, 0.0, 0.0))]:
        ok, _, clear, _ = simulate(start, goal, vision=True)
        assert ok and clear > 0.0
