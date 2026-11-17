"""
내비게이션 계산 부분 (ROS2와 상관없는 순수 계산)
- 좌표 변환, 각도 정규화, PID 제어, 도착 판단, Carrot(당근) 따라가기
- 단위: 거리 m, 각도 rad, 시간 s
- 좌표 약속 (ROS 표준, 팀 합의 전 임시): 로봇 기준 +x = 앞, +y = 왼쪽, 반시계 회전 = +
"""
import math
from dataclasses import dataclass


def normalize_angle(angle):
    """각도를 -π ~ +π 로 맞추기 (가장 짧게 도는 쪽)"""
    return math.atan2(math.sin(angle), math.cos(angle))


def field_to_robot(robot, goal_x, goal_y):
    """필드 좌표의 목표를 로봇 기준(앞, 왼쪽)으로 바꾸기. robot = (x, y, yaw)"""
    rx, ry, ryaw = robot
    dx = goal_x - rx                           # 1단계: 빼기
    dy = goal_y - ry
    c, s = math.cos(ryaw), math.sin(ryaw)
    front = c * dx + s * dy                    # 2단계: 회전 (앞 + / 뒤 -)
    left = -s * dx + c * dy                    #              (왼쪽 + / 오른쪽 -)
    return front, left


def robot_to_field(robot, front, left):
    """반대 방향: 로봇 기준(앞, 왼쪽)을 필드 좌표로 바꾸기 (Vision이 주는 꼬깔 위치 변환용)"""
    rx, ry, ryaw = robot
    c, s = math.cos(ryaw), math.sin(ryaw)
    return rx + c * front - s * left, ry + s * front + c * left


def clip(value, limit):
    """최대값 제한: -limit ~ +limit"""
    return max(-limit, min(limit, value))


# ---------------- PID ----------------
class Pid:
    """u = Kp*e + Ki*(오차를 쌓은 값) + Kd*(오차 변화 속도)"""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, i_limit=0.0, d_alpha=0.5):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit = i_limit      # I가 낼 수 있는 최대 명령 (적분 제한)
        self.d_alpha = d_alpha      # D 부드럽게: 0~1, 작을수록 더 부드럽지만 반응이 느림
        self.reset()

    def reset(self):
        """새 목표가 오거나 오차의 뜻이 바뀌면 쌓인 값과 이전 오차를 지운다"""
        self.integral = 0.0         # 오차를 쌓은 값 (∫e dt)
        self.prev_error = None      # 이전 오차 (D 계산용)
        self.d_filtered = 0.0       # 부드럽게 만든 오차 변화 속도

    def update(self, error, dt, use_i, limit):
        """
        error: 오차 (목표 - 현재), dt: 지난 계산 후 흐른 시간 (s)
        use_i: I를 쓸지 (False면 쌓인 값을 0으로), limit: 최대 명령
        """
        if dt <= 0.0:
            dt = 1e-3

        # P: 지금 오차
        p = self.kp * error

        # D: 오차가 얼마나 빨리 변하나 (위치값이 흔들리면 튀므로 부드럽게)
        if self.prev_error is not None:
            raw = (error - self.prev_error) / dt
            self.d_filtered = self.d_alpha * raw + (1.0 - self.d_alpha) * self.d_filtered
        self.prev_error = error
        d = self.kd * self.d_filtered

        # I: 오차를 계속 쌓기 (외란 때문에 남는 오차 없애기)
        if not use_i or self.ki <= 0.0:
            self.integral = 0.0
        else:
            nxt = self.integral + error * dt
            # 명령이 이미 최대값이면 더 쌓지 않음 (적분이 너무 커지는 것 방지)
            if abs(p + self.ki * nxt + d) < limit:
                self.integral = nxt
            # I가 낼 수 있는 명령에 상한 (적분 제한)
            self.integral = clip(self.integral, self.i_limit / self.ki)
        i = self.ki * self.integral

        return clip(p + i + d, limit)


@dataclass
class ControllerParams:
    """튜닝 값"""
    # 이동 (앞뒤, 좌우)
    kp_move: float = 0.8
    ki_move: float = 0.0
    kd_move: float = 0.0
    i_limit_move: float = 0.1      # I가 낼 수 있는 최대 이동 속도 (m/s)
    # 회전
    kp_turn: float = 1.5
    ki_turn: float = 0.0
    kd_turn: float = 0.0
    i_limit_turn: float = 0.3      # I가 낼 수 있는 최대 회전 속도 (rad/s)
    # 공통
    d_alpha: float = 0.5           # D 부드럽게 (0~1)
    max_move: float = 0.3          # 최대 이동 속도 (m/s)
    max_turn: float = 1.0          # 최대 회전 속도 (rad/s)
    tol_dist: float = 0.10         # 허용 거리 (m)
    tol_yaw: float = math.radians(10.0)   # 허용 각도 (rad)

    def move_pid(self):
        return Pid(self.kp_move, self.ki_move, self.kd_move, self.i_limit_move, self.d_alpha)

    def turn_pid(self):
        return Pid(self.kp_turn, self.ki_turn, self.kd_turn, self.i_limit_turn, self.d_alpha)


@dataclass
class Command:
    """걸음 명령 + 상태"""
    front: float = 0.0     # 앞 속도 (m/s), 앞 + / 뒤 -
    left: float = 0.0      # 왼쪽 속도 (m/s), 왼쪽 + / 오른쪽 -
    turn: float = 0.0      # 회전 속도 (rad/s), 반시계 +
    arrived: bool = False  # 도착했는지
    remain: float = 0.0    # 남은 거리 (m)
    carrot: tuple = None   # 이번에 향한 임시 목표 (RViz 표시용)


class GoalController:
    """목표 위치까지 PID로 이동하고 도착을 판단"""

    def __init__(self, params=None):
        self.p = params or ControllerParams()
        self.front_pid = self.p.move_pid()
        self.left_pid = self.p.move_pid()
        self.turn_pid = self.p.turn_pid()
        self.mode = None    # 'APPROACH' 다가가는 중 / 'ALIGN' 제자리에서 방향 맞추는 중

    def reset(self):
        self.front_pid.reset()
        self.left_pid.reset()
        self.turn_pid.reset()
        self.mode = None

    def step(self, robot, goal, dt):
        """robot, goal: (x, y, yaw)  [m, m, rad]"""
        gx, gy, gyaw = goal
        cmd = Command(carrot=(gx, gy))

        # 목표를 로봇 기준으로
        front, left = field_to_robot(robot, gx, gy)
        dist = math.hypot(front, left)
        cmd.remain = dist

        # 최종 방향까지 돌아야 할 각도 (목표 - 현재)
        final_turn = normalize_angle(gyaw - robot[2])

        # 도착 판단: 거리와 방향 둘 다 허용 범위 안
        if dist < self.p.tol_dist and abs(final_turn) < self.p.tol_yaw:
            cmd.arrived = True        # 정지 명령 (front, left, turn 모두 0)
            self.reset()
            return cmd

        if dist >= self.p.tol_dist:
            # 아직 멀다: 목표 쪽으로 몸을 돌리면서 걸어간다
            if self.mode != 'APPROACH':
                self.turn_pid.reset()     # 회전 오차의 뜻이 바뀌므로 초기화
                self.mode = 'APPROACH'
            heading_err = math.atan2(left, front)     # 목표가 몇 도 방향인가
            # 이동: PID (I로 미끄러짐 때문에 남는 거리까지 없앰)
            cmd.front = self.front_pid.update(front, dt, True, self.p.max_move)
            cmd.left = self.left_pid.update(left, dt, True, self.p.max_move)
            # 회전: 목표 쪽 바라보기 (P + D, 가까워지면 방향이 크게 흔들리므로 I는 안 씀)
            cmd.turn = self.turn_pid.update(heading_err, dt, False, self.p.max_turn)
        else:
            # 위치는 도착: 제자리에서 최종 방향만 맞춘다
            if self.mode != 'ALIGN':
                self.reset()
                self.mode = 'ALIGN'
            # 회전: PID (I로 남는 각도까지 없앰)
            cmd.turn = self.turn_pid.update(final_turn, dt, True, self.p.max_turn)
        return cmd


class PathFollower:
    """
    경로 위 조금 앞선 점(Carrot, 당근)을 따라가기
    - 가장 가까운 경로 점을 찾고, 거기서 carrot_points개 앞의 점을 임시 목표로 삼는다
    - 당근이 경로 끝에 닿으면 GoalController로 최종 목표에 정확히 도착하고 방향을 맞춘다
    """

    def __init__(self, params=None, carrot_points=5):
        self.p = params or ControllerParams()
        self.final = GoalController(self.p)      # 경로 끝: 최종 목표에 정확히 도착
        self.front_pid = self.p.move_pid()       # 경로 중간: 당근 따라가기 (I 안 씀)
        self.left_pid = self.p.move_pid()
        self.turn_pid = self.p.turn_pid()
        self.carrot_points = carrot_points       # 몇 개 앞의 점을 당근으로 쓸지 (m가 아니라 개수)
        self.on_carrot = False

    def reset(self):
        self.final.reset()
        self.front_pid.reset()
        self.left_pid.reset()
        self.turn_pid.reset()
        self.on_carrot = False

    def step(self, robot, path, goal, dt):
        """robot, goal: (x, y, yaw)   path: [(x, y), ...]"""
        rx, ry, _ = robot
        n = len(path)

        # 1. 가장 가까운 경로 점 찾기
        nearest = min(range(n), key=lambda k: math.hypot(path[k][0] - rx, path[k][1] - ry))

        # 남은 경로 길이
        remain = math.hypot(path[nearest][0] - rx, path[nearest][1] - ry)
        for k in range(nearest, n - 1):
            remain += math.hypot(path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1])

        # 2. 몇 개 앞의 점 = 당근
        c = nearest + self.carrot_points
        if c >= n - 1:
            # 당근이 경로 끝에 닿음 -> 최종 목표로 정확히 이동 + 방향 맞추기 (PID)
            if self.on_carrot:
                self.final.reset()
                self.on_carrot = False
            cmd = self.final.step(robot, goal, dt)
            cmd.remain = remain
            return cmd

        # 3. 당근을 로봇 기준으로 바꿔서 따라가기 (P + D)
        #    당근은 항상 조금 앞에 있어서 오차가 0이 되지 않으므로 I는 쓰지 않음
        if not self.on_carrot:
            self.front_pid.reset()
            self.left_pid.reset()
            self.turn_pid.reset()
            self.on_carrot = True
        front, left = field_to_robot(robot, path[c][0], path[c][1])
        return Command(
            front=self.front_pid.update(front, dt, False, self.p.max_move),
            left=self.left_pid.update(left, dt, False, self.p.max_move),
            turn=self.turn_pid.update(math.atan2(left, front), dt, False, self.p.max_turn),
            remain=remain,
            carrot=tuple(path[c]),
        )
