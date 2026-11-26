"""앞걸음 위주로 걷는지 확인 (휴머노이드는 옆걸음·뒷걸음이 느리고 잘 넘어짐)"""
import math

from robit_navigation.astar import GridPlanner
from robit_navigation.nav_core import ControllerParams, PathFollower, normalize_angle

R = math.radians


def drive(start, goal, max_time=60.0):
    """반환: 도착 여부, 마지막 위치, 명령 기록 [(남은 거리, 앞, 옆, 회전), ...]"""
    p = GridPlanner(-4.5, 4.5, -3.0, 3.0, robot_radius=0.15, margin=0.15)
    params = ControllerParams()
    f = PathFollower(params, 5)
    x, y, yaw = start
    log, path = [], None
    for k in range(int(max_time / 0.1)):
        if k % 10 == 0:
            path = p.plan((x, y), goal[:2])
        c = f.step((x, y, yaw), path, goal, 0.1)
        if c.arrived:
            return True, (x, y, yaw), log, params
        log.append((math.hypot(goal[0] - x, goal[1] - y), c.front, c.left, c.turn))
        cs, sn = math.cos(yaw), math.sin(yaw)
        x += (cs * c.front - sn * c.left) * 0.1
        y += (sn * c.front + cs * c.left) * 0.1
        yaw = normalize_angle(yaw + c.turn * 0.1)
    return False, (x, y, yaw), log, params


def test_goal_behind_turns_first():
    # 목표가 등 뒤: 뒤로 걷지 않고 먼저 제자리에서 돌기
    ok, pose, log, p = drive((0.0, 0.0, 0.0), (-2.0, 0.0, R(180)))
    assert ok
    _, front, left, turn = log[0]
    assert front == 0.0 and left == 0.0 and abs(turn) > 0.5        # 첫 명령 = 제자리 회전
    assert all(fr >= -p.max_back - 1e-9 for _, fr, _, _ in log)    # 뒷걸음은 제한 안
    assert abs(normalize_angle(pose[2] - R(180))) < p.tol_yaw


def test_far_is_mostly_forward():
    # 멀리 갈 때: 옆걸음은 max_side 까지만, 대부분 앞걸음
    ok, _, log, p = drive((-3.0, -2.0, R(90)), (2.0, 2.0, 0.0))
    assert ok
    far = [(fr, le) for d, fr, le, _ in log if d > p.near_dist + 0.05]
    assert all(abs(le) <= p.max_side + 1e-9 for _, le in far)
    forward = sum(fr for fr, _ in far)
    side = sum(abs(le) for _, le in far)
    assert side < 0.1 * forward                                      # 옆으로 간 거리 < 앞으로 간 거리의 10%


def test_near_uses_side_step():
    # 바로 옆 25cm 목표: 돌지 않고 옆걸음으로 맞춤 (마지막 방향 = 지금 방향)
    ok, pose, log, p = drive((0.0, 0.0, 0.0), (0.0, 0.25, 0.0))
    assert ok
    assert max(abs(le) for _, _, le, _ in log) > p.max_side          # 가까우면 옆걸음 씀
    assert all(abs(le) <= p.max_side_near + 1e-9 for _, _, le, _ in log)
    assert abs(pose[2]) < p.tol_yaw


def test_arrive_latch():
    # 도착 래치: 한 번 도착하면 위치가 조금 흔들려도(허용 범위 2배 안) 다시 안 움직임
    from robit_navigation.nav_core import GoalController
    p = ControllerParams(tol_dist=0.15, tol_yaw=math.radians(15), latch=2.0)
    g = GoalController(p)
    goal = (1.0, 0.0, 0.0)
    assert g.step((0.95, 0.0, 0.0), goal, 0.1).arrived                 # 5cm: 도착
    c = g.step((0.78, 0.05, math.radians(20)), goal, 0.1)              # 22cm, 20도 흔들림 (허용 범위 밖, 2배 안)
    assert c.arrived and c.front == c.left == c.turn == 0.0            # 그래도 계속 도착, 안 움직임
    c = g.step((0.6, 0.0, 0.0), goal, 0.1)                              # 40cm: 2배(30cm) 넘게 밀림
    assert not c.arrived and c.front > 0.0                              # 다시 걸어감
    # 래치를 끄면(0) 허용 범위만 벗어나도 다시 움직임
    g2 = GoalController(ControllerParams(tol_dist=0.15, tol_yaw=math.radians(15), latch=0.0))
    assert g2.step((0.95, 0.0, 0.0), goal, 0.1).arrived
    assert not g2.step((0.78, 0.0, 0.0), goal, 0.1).arrived
    # 새 목표(reset)면 래치 풀림
    g.step((0.95, 0.0, 0.0), goal, 0.1)
    g.reset()
    assert not g.step((0.78, 0.0, 0.0), goal, 0.1).arrived
