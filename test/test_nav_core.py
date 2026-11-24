"""계산 부분 확인: 손으로 푼 답, 좌표 변환, PID, 꼬깔 기억 (실행: python3 -m pytest test)"""
import math

from robit_navigation.nav_core import Pid, field_to_robot, normalize_angle, path_remaining, robot_to_field
from robit_navigation.obstacle_memory import ObstacleMemory

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
    assert not mem.update(r, [(1.0, 0.0)], 0.0) and mem.confirmed() == []   # 한 번은 인정 안 함
    assert mem.update(r, [(1.02, 0.01)], 0.1) and len(mem.confirmed()) == 1  # 두 번 보면 인정
    mem.update((0, 0, R(180)), [], 10.0)
    assert len(mem.confirmed()) == 1            # 뒤로 돌아서 안 보여도 기억
    mem.update(r, [], 10.0)
    assert mem.confirmed() == []                # 보여야 하는데 계속 안 보이면 지움

    mem2 = ObstacleMemory()
    mem2.update((0, 0, R(90)), [(1.0, 0.0)], 0.0)    # 위를 보면서 앞 1m
    mem2.update((0, 0, 0), [(0.0, 1.0)], 0.1)        # 오른쪽(+x)을 보면서 왼쪽 1m
    c = mem2.confirmed()
    assert len(c) == 1 and close(c[0][0], 0.0, 0.01) and close(c[0][1], 1.0, 0.01)   # 같은 꼬깔


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
