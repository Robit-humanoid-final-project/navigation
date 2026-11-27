"""계산 부분 확인: 손으로 푼 답, 좌표 변환, PID, 꼬깔 기억 (실행: python3 -m pytest test)"""
import math

from robit_navigation.nav_core import Pid, field_to_robot, normalize_angle, path_remaining, read_walking_parm, robot_to_field
from robit_navigation.obstacle_memory import ObstacleMemory, ObstacleMemoryParams
from robit_navigation.astar import GridPlanner

R = math.radians


def close(a, b, tol=1e-6):
    return abs(a - b) < tol


def test_field_to_robot():
    f, l = field_to_robot((0, 0, R(180)), 0, 2)
    assert close(f, 0) and close(l, -2)          # 오른쪽 2m
    f, l = field_to_robot((1, 1, 0), 3, 2)
    assert close(f, 2) and close(l, 1)           # 앞 2m, 왼쪽 1m
    f, l = field_to_robot((0, 0, R(-90)), 1, 0)
    assert close(f, 0) and close(l, 1)           # 왼쪽 1m


def test_robot_to_field():
    x, y = robot_to_field((1, 1, R(90)), 2, 0)
    assert close(x, 1) and close(y, 3)           # 위를 보는 로봇의 앞 2m = 필드 (1, 3)
    x, y = robot_to_field((0, 0, R(180)), 0, 2)
    assert close(x, 0) and close(y, -2)          # 왼쪽을 보는 로봇의 왼쪽 2m = 필드 (0, -2)
    robot = (-1.0, 0.5, R(37))
    x, y = robot_to_field(robot, *field_to_robot(robot, 2.0, -1.0))
    assert close(x, 2.0) and close(y, -1.0)      # 두 변환은 서로 반대


def test_normalize_angle():
    assert close(normalize_angle(R(160) - R(-160)), R(-40))   # 오른쪽 40도
    assert close(normalize_angle(R(-170) - R(170)), R(20))    # 왼쪽 20도


def test_pid():
    p_only = Pid(0.8, 0.0, 0.0, 0.1, 0.5)
    assert close(p_only.update(0.1, 0.1, True, 0.3), 0.08)    # P: 0.8 x 0.1
    assert close(p_only.update(3.0, 0.1, True, 0.3), 0.3)     # 최대값으로 자름
    pi = Pid(0.8, 0.5, 0.0, 0.1, 0.5)
    for _ in range(100):
        u = pi.update(0.1, 0.1, True, 0.3)
    assert 0.08 < u <= 0.08 + 0.1 + 1e-9       # I가 쌓이지만 i_limit(0.1)을 넘지 않음
    pd = Pid(0.8, 0.0, 0.5, 0.0, 1.0)
    pd.update(1.0, 0.1, False, 10.0)
    assert pd.update(0.9, 0.1, False, 10.0) < 0.8 * 0.9   # 다가가는 중이면 D가 브레이크


def test_obstacle_memory():
    mem = ObstacleMemory()
    r = (0, 0, 0)
    assert not mem.update_field(r, [(1.0, 0.0)], 0.0) and mem.confirmed() == []   # 한 번은 인정 안 함
    assert mem.update_field(r, [(1.02, 0.01)], 0.1) and len(mem.confirmed()) == 1  # 두 번 보면 인정
    mem.update_field((0, 0, R(180)), [], 10.0)
    assert len(mem.confirmed()) == 1            # 뒤로 돌아서 안 보여도 기억
    mem.update_field(r, [], 10.0)
    assert mem.confirmed() == []                # 보여야 하는데 계속 안 보이면 지움


def test_path_remaining():
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]       # ㄱ자 경로, 전체 2m
    assert close(path_remaining(path, (0.0, 0.0)), 2.0)
    assert close(path_remaining(path, (1.0, 0.0)), 1.0)      # 모서리: 남은 1m
    assert close(path_remaining(path, (1.0, 1.0)), 0.0)      # 끝


def test_obstacle_memory_field_frame():
    # Localization(world_model)처럼 필드 기준으로 받아도 같은 꼬깔로 기억
    mem = ObstacleMemory()
    r = (0, 0, 0)
    mem.update_field(r, [(1.0, 0.0)], 0.0)
    assert mem.update_field(r, [(1.02, -0.01)], 0.1) and len(mem.confirmed()) == 1
    mem.update_field((0, 0, R(180)), [], 10.0)
    assert len(mem.confirmed()) == 1            # 안 보이는 쪽이면 계속 기억


def test_obstacle_memory_ghosts():
    # 유령 꼬깔 막기: 필드 밖에 보인 꼬깔은 버리고, 카메라 밖이어도 아주 오래 안 보이면 지움
    from robit_navigation.obstacle_memory import ObstacleMemoryParams
    mem = ObstacleMemory(ObstacleMemoryParams(max_unseen_time=10.0, field_half_x=4.5, field_half_y=3.0))
    r = (0, 0, 0)
    mem.update_field(r, [(1.0, 0.0), (4.8, 0.0)], 0.0)          # (4.8, 0) = 필드 밖
    mem.update_field(r, [(1.0, 0.0), (4.8, 0.0)], 0.1)
    assert mem.confirmed() == [(1.0, 0.0)]                      # 필드 밖 꼬깔은 기억 안 함
    back = (0, 0, R(180))                                       # 반대쪽 보기 = 꼬깔이 카메라 밖
    mem.update_field(back, [], 5.0)
    assert len(mem.confirmed()) == 1                            # 10초 안 = 아직 기억
    mem.update_field(back, [], 10.5)
    assert mem.confirmed() == []                                # 10초 넘게 안 보임 = 지움
    mem.update_field(r, [(1.0, 0.0)], 11.0)
    mem.update_field(r, [(1.0, 0.0)], 11.1)
    assert len(mem.confirmed()) == 1                            # 다시 보이면 다시 기억
    mem.clear()
    assert mem.confirmed() == []                                # 지우기 버튼


def test_to_walk():
    from robit_navigation.nav_core import to_walk
    m = (0.3, 0.15, 1.0, 40.0, 20.0, 10.0)      # max_move, max_side, max_turn, ik 최대값
    assert to_walk(0.3, 0.0, 0.0, *m) == (40.0, 0.0, 0.0, 1.0)        # 최대 앞 = x 최대값
    assert to_walk(0.15, 0.0, 0.0, *m) == (20.0, 0.0, 0.0, 1.0)       # 절반 = 절반
    assert to_walk(0.0, -0.15, 0.5, *m) == (0.0, -20.0, 5.0, 1.0)     # 오른쪽 옆걸음, 왼쪽 반 회전
    assert to_walk(9.0, 0.0, 0.0, *m)[0] == 40.0                      # 최대값 넘지 않음
    assert to_walk(0.0, 0.0, 0.0, *m) == (0.0, 0.0, 0.0, 0.0)         # 멈춤 = flag 0
    assert to_walk(0.0, 0.0, 1.0, 0.3, 0.15, 1.0, 40.0, 20.0, -10.0)[2] == -10.0   # 음수 = 방향 반대


def test_cone_follows_new_position():
    # 고정 안 함: 내 위치가 틀렸을 때 잘못 찍힌 꼬깔도, 다시 제대로 보이면 새 위치로 바뀜
    mem = ObstacleMemory(ObstacleMemoryParams(merge_dist=0.5, forget_time=2.0))
    r = (0.0, 0.0, 0.0)
    for k in range(5):
        mem.update_field(r, [(2.0, 0.35)], 0.1 * k)          # 방향이 틀려서 35cm 옆에 찍힘
    for k in range(30):
        mem.update_field(r, [(2.0, 0.0)], 1.0 + 0.1 * k)     # 위치가 다시 맞음
    (x, y), = mem.confirmed()
    assert abs(x - 2.0) < 0.03 and abs(y) < 0.03


def test_keep_same_side():
    # 꼬깔이 조금 흔들려도 한 번 정한 쪽(왼쪽/오른쪽)으로 계속 감
    pl = GridPlanner(-4.5, 4.5, -3.0, 3.0, resolution=0.1, robot_radius=0.15, margin=0.08, boundary_margin=0.2)
    start, goal = (-1.5, 0.0), (1.5, 0.0)

    def side(path):
        mid = [y for x, y in path if abs(x) < 0.3]
        return 'L' if sum(mid) / len(mid) > 0 else 'R'

    pl.set_obstacles([(0.0, 0.05, 0.1)])
    path = pl.plan(start, goal)
    first = side(path)
    flips_before = 0
    for cy in (-0.1, 0.1, -0.08, 0.06, -0.1):                # 꼬깔이 좌우로 흔들림
        pl.set_obstacles([(0.0, cy, 0.1)])
        pl.set_prefer(None)
        flips_before += side(pl.plan(start, goal)) != first  # 예전 방식: 쪽이 바뀜
        pl.set_prefer(path, 0.5)
        path = pl.plan(start, goal)
        assert side(path) == first                            # 새 방식: 같은 쪽 유지
    assert flips_before > 0
    # 지금 쪽이 꼬깔 벽으로 완전히 막히면 다른 쪽으로 바꿈
    sign = 1.0 if first == 'L' else -1.0
    pl.set_obstacles([(0.0, 0.0, 0.1)] + [(0.0, sign * 0.45 * k, 0.1) for k in range(1, 7)])
    pl.set_prefer(path, 0.5)
    assert side(pl.plan(start, goal)) != first


def test_read_walking_parm(tmp_path):
    # 보행 담당 yaml (11/26) 그대로
    f = tmp_path / 'walking_parm.yaml'
    f.write_text("""ros__parameters:
    # Navigation
    Nav_test_X: 30.00
    Nav_test_side_R: -10.00
    Nav_test_side_L: 10.00
    Nav_test_Yaw_R: -5.00
    Nav_test_Yaw_L: 5.00
    Ta_test_X_F: 5.00
""")
    assert read_walking_parm(str(f)) == {'ik_max_x': 30.0, 'ik_max_y': 10.0, 'ik_max_yaw': 5.0}
    # 노드 이름 밑에 있어도 찾음
    f.write_text("/**:\n  ros__parameters:\n    Nav_test_X: 20\n    Nav_test_side_L: 8\n    Nav_test_Yaw_L: 4\n")
    assert read_walking_parm(str(f)) == {'ik_max_x': 20.0, 'ik_max_y': 8.0, 'ik_max_yaw': 4.0}
    assert read_walking_parm(str(tmp_path / 'none.yaml')) is None      # 파일 없음


def test_cones_every_3_seconds():
    # Localization이 꼬깔을 3초에 한 번 보내고 사이에는 빈 값: 노드 기본값이면 계속 있어야 함 (안 깜빡임)
    mem = ObstacleMemory(ObstacleMemoryParams(min_hits=1, forget_time=4.0, smoothing=1.0))
    for k in range(100):                                       # 10초 (0.1초마다)
        cones = [(1.5, 0.1 if (k // 30) % 2 else 0.0)] if k % 30 == 0 else []
        mem.update_field((0.0, 0.0, 0.0), cones, 0.1 * k)
        assert len(mem.confirmed()) == 1
    assert mem.confirmed()[0] == (1.5, 0.1)                    # 마지막에 받은 위치 그대로
    # 꼬깔이 진짜 없어지면(더 안 옴) 4초 뒤 지움
    for k in range(100, 160):
        mem.update_field((0.0, 0.0, 0.0), [], 0.1 * k)
    assert mem.confirmed() == []
