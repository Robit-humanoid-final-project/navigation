"""
내비게이션 노드 (내 담당 부분)

받는 토픽 (팀 메시지 humanoid_interfaces):
  nav_command    humanoid_interfaces/NavCommand   <- Taskplanner
                 nav_enable        : true일 때만 걸음. false = 멈추고 IDLE (Vision이 공 앞을 맡을 때 등)
                 x, y, theta       : 목표 (필드 기준, m, rad)
                 pos_tolerance     : 이 거리 안이면 도착 (m), 0 이하면 tol_dist 사용
                 theta_tolerance   : 이 각도 안이면 도착 (rad), 0 이하면 tol_yaw_deg 사용
                 forbid_right_half : true = RIGHT 진영 금지 (Ready). 로봇 중심이 가운데선에서 center_margin 왼쪽까지만
                 avoid_ball        : true = 공을 장애물로 피함 (공 주변 0.37m 안으로 안 들어감)
  world_model    humanoid_interfaces/WorldModel   <- Localization (모두 필드 기준)
                 robot_pose        : 내 위치 x, y [m], theta [rad]
                 ball_pose, ball_detected : 공 위치 (안 보이면 마지막으로 본 위치를 씀)
                 obstacles         : 지금 보이는 꼬깔 -> 꼬깔 기억(obstacle_memory)으로 기억해서 씀
                 is_localized      : 아직 Localization에서 구현 안 됨 (항상 false) -> require_localized 가
                                     true일 때만 봄 (위치를 못 믿으면 NO_POSE로 멈춤)
  teammate_pose  geometry_msgs/Pose2D             우리 팀 다른 로봇 위치 (필드 기준) <- UDP (메시지 미정)
                 꼬깔처럼 피함. 1초 넘게 안 오면 무시. 양보(yield_to_teammate)·안전 정지 때는 멈춤

보내는 토픽:
  nav_feedback   humanoid_interfaces/NavFeedback  -> Taskplanner
                 status            : 0 대기(IDLE) / 1 이동 중(MOVING, 다른 로봇 기다리는 중 포함)
                                     / 2 도착(ARRIVED) / 3 길 없음(BLOCKED, 위치 모름)
                 eta_seconds       : 도착 예상 시간 (s) = 남은 경로 길이 / max_move. 도착 0, 길 없음 -1
  master2ik      humanoid_interfaces/Master2IkMsg  -> 보행 (ik_walk)
                 x_length, y_length, yaw = 내 속도를 비율로 바꾼 값 (to_walk)
                   예) 앞 속도가 max_move 면 x_length = ik_max_x, 절반이면 ik_max_x 의 절반
                 flag = 1 걷기, 0 멈춤
                 (ik_max_x, ik_max_y, ik_max_yaw 는 임시 값: 보행 담당 답 오면 바꾸기)
  nav_cmd        geometry_msgs/Twist   같은 명령을 m/s, rad/s 로 (가짜 로봇·UI용)
                 linear.x 앞, linear.y 왼쪽 [m/s], angular.z 회전 [rad/s]
  master2ik, nav_cmd 둘 다: IDLE이 되면 0을 한 번만 보내고 그다음엔 안 보냄 (Vision 명령과 안 섞이게)
  nav_path       nav_msgs/Path                    A-star 경로 (화면 표시용)
  nav_debug      visualization_msgs/MarkerArray   목표, 꼬깔·공 위험 범위, 당근, 상태 글자 (화면 표시용)

공 바로 앞에서 골대 방향 맞추기·킥 위치 맞추기는 Vision이 맡음 (그동안 Taskplanner가 nav_enable = false)

이동, 회전 모두 PID 제어 (ki, kd = 0 이면 P만)
튜닝 값은 실행 중에도 바꿀 수 있음 (빌드 다시 안 해도 됨):
  ros2 param set /robot1/navigation_node ki_move 0.5
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point, Pose2D, PoseStamped, Twist
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from humanoid_interfaces.msg import Master2IkMsg, NavCommand, NavFeedback, WorldModel

from robit_navigation.astar import GridPlanner
from robit_navigation.teammate import (SpeedEstimator, look_ahead_point, plan_with_teammate,
                                      should_yield, too_close)
from robit_navigation.nav_core import ControllerParams, PathFollower, path_remaining, to_walk
from robit_navigation.obstacle_memory import ObstacleMemory, ObstacleMemoryParams

FRAME = 'map'
# 상태 글자 -> NavFeedback.status 숫자 (팀 약속: 0 idle, 1 moving, 2 reached, 3 no path)
STATUS_CODE = {'IDLE': 0, 'MOVING': 1, 'WAITING': 1, 'ARRIVED': 2, 'BLOCKED': 3, 'NO_POSE': 3}

# 실행 중에 바꿀 수 있는 값 (이름: 기본값)
TUNABLE = {
    # PID: ki, kd 를 0으로 두면 P만, kd만 올리면 PD, 둘 다 올리면 PID
    'kp_move': 0.8, 'ki_move': 0.0, 'kd_move': 0.0,
    'i_limit_move': 0.1,       # I가 낼 수 있는 최대 이동 속도 (m/s)
    'kp_turn': 1.5, 'ki_turn': 0.0, 'kd_turn': 0.0,
    'i_limit_turn': 0.3,       # I가 낼 수 있는 최대 회전 속도 (rad/s)
    'd_alpha': 0.5,            # D 부드럽게 (0~1, 작을수록 부드러움)
    'max_move': 0.3,           # m/s
    'max_turn': 1.0,           # rad/s
    'tol_dist': 0.10,          # 기본 허용 거리 (m), NavCommand.pos_tolerance 가 0 이하일 때
    'tol_yaw_deg': 10.0,       # 기본 허용 각도 (도), NavCommand.theta_tolerance 가 0 이하일 때
    # 앞걸음 위주 (휴머노이드는 옆걸음·뒷걸음이 느리고 잘 넘어짐)
    'turn_in_place_deg': 30.0, # 가려는 쪽이 이 각도보다 벗어나면 제자리 회전
    'walk_again_deg': 15.0,    # 이 각도보다 작아지면 다시 걷기
    'near_dist': 0.3,          # 목표가 이 거리 안이면 마지막 방향 보면서 옆걸음으로 맞춤 (m)
    'max_side': 0.05,          # 멀 때 옆걸음 최대 (m/s)
    'max_side_near': 0.15,     # 가까울 때 옆걸음 최대 (m/s)
    'max_back': 0.05,          # 뒷걸음 최대 (m/s)
    'carrot_points': 5,        # 당근: 몇 개 앞 점
    'robot_radius': 0.15,      # 로봇 25 x 17cm를 덮는 원 반지름 (m)
    'margin': 0.15,            # 추가 여유 (m)
    'center_margin': 0.2,      # RIGHT 진영 금지일 때 로봇 중심이 가운데선에서 떨어져야 하는 거리 (m)
    'boundary_margin': 0.2,    # 로봇 중심이 필드 선에서 떨어져야 하는 거리 (m), 필드 밖 = 30초 페널티
    'cone_radius': 0.1,        # 꼬깔 14 x 14cm를 덮는 원 반지름 (m)
    'ball_radius': 0.07,       # 공 반지름 (지름 14cm) (m)
    'replan_period': 1.0,      # 몇 초마다 다시 계획
    'require_localized': False,  # True: world_model.is_localized 가 false면 멈춤 (Localization 구현 후 켜기)
    # 우리 팀 다른 로봇 피하기
    'teammate_radius': 0.15,   # 다른 로봇 몸통 반지름 (m)
    'yield_to_teammate': False,   # True: 다른 로봇이 가까이서 움직이면 멈춰서 양보 (한 로봇만 True)
    'yield_dist': 0.9,         # 이 거리 안에서 양보 (m)
    'stop_dist': 0.45,         # 다른 로봇이 바로 앞 이 거리 안이면 안전 정지 (m)
    # 보행(master2ik)으로 보낼 최대값 (임시 값, 보행 담당 답 오면 바꾸기). 음수로 넣으면 방향 반대
    'ik_max_x': 25.0,          # 앞 속도가 max_move 일 때 x_length
    'ik_max_y': 25.0,          # 옆 속도가 max_side_near 일 때 y_length
    'ik_max_yaw': 10.0,        # 회전 속도가 max_turn 일 때 yaw
}
# 시작할 때만 읽는 값
STARTUP_ONLY = {
    'rate': 10.0,              # 1초에 몇 번 계산
    'field_length': 9.0,       # m
    'field_width': 6.0,
    'grid_resolution': 0.1,    # 격자 한 칸 (m)
    # 꼬깔 기억 (카메라 시야와 맞추기)
    'view_angle_deg': 60.0,    # 카메라 시야 전체 각도
    'view_range': 3.0,         # 카메라가 꼬깔을 볼 수 있는 거리 (m)
    'merge_dist': 0.3,         # 이 거리 안이면 같은 꼬깔
    'min_hits': 2,             # 몇 번 봐야 진짜 꼬깔로 인정
    'forget_time': 2.0,        # 보여야 할 곳에서 이만큼 안 보이면 지움 (s)
}


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        for name, default in {**TUNABLE, **STARTUP_ONLY}.items():
            self.declare_parameter(name, default)
        p = lambda name: self.get_parameter(name).value

        self.rate = p('rate')
        self.field_l, self.field_w = p('field_length'), p('field_width')
        self.res = p('grid_resolution')
        self.memory = ObstacleMemory(ObstacleMemoryParams(
            merge_dist=p('merge_dist'), min_hits=p('min_hits'), forget_time=p('forget_time'),
            view_half_angle=math.radians(p('view_angle_deg')) / 2.0, view_range=p('view_range')))

        self.robot = None          # 아직 위치를 못 받았으면 None
        self.goal = None           # 아직 목표를 못 받았으면 None
        self.obstacles = []        # 계획에 쓰는 꼬깔 [(x, y, 반지름), ...] (필드 기준)
        self.left_only = False     # True: RIGHT 진영 금지 (Ready), NavCommand.forbid_right_half
        self.avoid_ball = True     # True: 공을 장애물로 피함, NavCommand.avoid_ball
        self.localized = True      # world_model.is_localized
        self.tol = (None, None)    # 이번 목표의 허용 거리·각도 (None = 기본값)
        self.ball = None           # 공 위치 (x, y) (필드 기준), 모르면 None
        self.teammate = None       # 다른 로봇 위치 (x, y) (필드 기준), 모르면 None
        self.teammate_time = None  # 다른 로봇 위치를 마지막으로 받은 시간
        self.teammate_planned = None   # 경로 계획할 때 쓴 다른 로봇 위치
        self.teammate_speed = SpeedEstimator()
        self.path = None
        self.need_replan = False
        self.last_plan_time = None
        self.last_step_time = None
        self.last_status = ''
        self.remain = 0.0

        self.apply_parameters()
        # 실행 중에 값이 바뀌면: 먼저 검사하고(on_set), 통과하면 적용(post_set)
        self.add_on_set_parameters_callback(self.check_parameters)
        self.add_post_set_parameters_callback(self.on_parameters_changed)

        # 받기
        self.create_subscription(NavCommand, 'nav_command', self.on_command, 10)
        self.create_subscription(WorldModel, 'world_model', self.on_world, 10)
        self.create_subscription(Pose2D, 'teammate_pose', self.on_teammate, 10)
        # 보내기
        self.walk_pub = self.create_publisher(Master2IkMsg, 'master2ik', 10)
        self.cmd_pub = self.create_publisher(Twist, 'nav_cmd', 10)
        self.feedback_pub = self.create_publisher(NavFeedback, 'nav_feedback', 10)
        self.path_pub = self.create_publisher(Path, 'nav_path', 10)
        self.debug_pub = self.create_publisher(MarkerArray, 'nav_debug', 10)
        # 반복 (기본 0.1초마다)
        self.create_timer(1.0 / self.rate, self.loop)

        self.get_logger().info('내비게이션 노드 시작: 위치와 목표를 기다리는 중')

    # ---------------- 튜닝 값 ----------------
    def check_parameters(self, params):
        """잘못된 값이 들어오면 거부"""
        for prm in params:
            n, v = prm.name, prm.value
            if n in STARTUP_ONLY:
                return SetParametersResult(
                    successful=False, reason=f'{n} 은(는) 시작할 때만 정할 수 있음 (launch 파일에서 바꾸기)')
            if n == 'carrot_points' and v < 1:
                return SetParametersResult(successful=False, reason='carrot_points 는 1 이상')
            if n == 'd_alpha' and not 0.0 < v <= 1.0:
                return SetParametersResult(successful=False, reason='d_alpha 는 0보다 크고 1 이하')
            if isinstance(v, (int, float)) and v < 0 and not n.startswith('ik_max'):
                return SetParametersResult(successful=False, reason=f'{n} 는 0 이상')
        return SetParametersResult(successful=True)

    def on_parameters_changed(self, params):
        self.apply_parameters()
        self.need_replan = True
        self.get_logger().info('튜닝 값 변경 적용')

    def apply_parameters(self):
        """현재 파라미터 값으로 계산 도구를 다시 만든다"""
        p = lambda name: float(self.get_parameter(name).value)
        cp = ControllerParams(
            kp_move=p('kp_move'), ki_move=p('ki_move'), kd_move=p('kd_move'),
            i_limit_move=p('i_limit_move'),
            kp_turn=p('kp_turn'), ki_turn=p('ki_turn'), kd_turn=p('kd_turn'),
            i_limit_turn=p('i_limit_turn'),
            d_alpha=p('d_alpha'), max_move=p('max_move'), max_turn=p('max_turn'),
            tol_dist=p('tol_dist'), tol_yaw=math.radians(p('tol_yaw_deg')),
            turn_in_place=math.radians(p('turn_in_place_deg')),
            walk_again=math.radians(p('walk_again_deg')),
            near_dist=p('near_dist'), max_side=p('max_side'),
            max_side_near=p('max_side_near'), max_back=p('max_back'))
        self.cone_radius = p('cone_radius')
        self.replan_period = p('replan_period')
        self.require_localized = bool(self.get_parameter('require_localized').value)
        self.ball_radius = p('ball_radius')
        self.teammate_radius = p('teammate_radius')
        self.yield_to_teammate = bool(self.get_parameter('yield_to_teammate').value)
        self.yield_dist = p('yield_dist')
        self.stop_dist = p('stop_dist')
        self.ik_max = (p('ik_max_x'), p('ik_max_y'), p('ik_max_yaw'))
        self.obstacles = [(x, y, self.cone_radius) for x, y, _ in self.obstacles]
        self.default_tol = (cp.tol_dist, cp.tol_yaw)
        self.follower = PathFollower(cp, int(self.get_parameter('carrot_points').value))
        self.set_tolerance(*self.tol) if hasattr(self, 'tol') else None
        hl, hw = self.field_l / 2.0, self.field_w / 2.0
        self.planner = GridPlanner(-hl, hl, -hw, hw, resolution=self.res,
                                   robot_radius=p('robot_radius'), margin=p('margin'),
                                   boundary_margin=p('boundary_margin'))
        self.planner.set_max_x(-p('center_margin') if self.left_only else None)

    def set_tolerance(self, dist, yaw):
        """이번 목표의 허용 오차 (None 이면 기본값 tol_dist, tol_yaw_deg)"""
        self.follower.p.tol_dist = dist if dist else self.default_tol[0]
        self.follower.p.tol_yaw = yaw if yaw else self.default_tol[1]

    # ---------------- 받기 ----------------
    def on_command(self, msg):
        """
        Taskplanner의 NavCommand. Taskplanner가 같은 명령을 계속 보내도 되게,
        목표가 바뀌었을 때만 새 목표로 처리 (PID에 쌓인 값을 지우고 다시 계획)
        """
        # RIGHT 진영 금지 (Ready)
        if msg.forbid_right_half != self.left_only:
            self.left_only = msg.forbid_right_half
            margin = float(self.get_parameter('center_margin').value)
            self.planner.set_max_x(-margin if self.left_only else None)
            self.need_replan = True
            self.get_logger().info('RIGHT 진영 금지 (Ready)' if self.left_only else 'RIGHT 진영 금지 해제')
        # 공 피하기
        if msg.avoid_ball != self.avoid_ball:
            self.avoid_ball = msg.avoid_ball
            self.need_replan = True
        # 걷기 끄기 = 목표 취소 (멈추고 IDLE)
        if not msg.nav_enable:
            self.cancel()
            return
        # 허용 오차 (0 이하면 기본값)
        tol = (msg.pos_tolerance if msg.pos_tolerance > 0.0 else None,
               msg.theta_tolerance if msg.theta_tolerance > 0.0 else None)
        if tol != self.tol:
            self.tol = tol
            self.set_tolerance(*tol)
        # 목표: 2cm 또는 2도 넘게 바뀌었을 때만 새 목표
        goal = (msg.x, msg.y, msg.theta)
        if self.goal is not None and math.hypot(goal[0] - self.goal[0], goal[1] - self.goal[1]) < 0.02 \
                and abs(math.remainder(goal[2] - self.goal[2], 2 * math.pi)) < math.radians(2.0):
            return
        self.goal = goal
        self.need_replan = True
        self.follower.reset()    # 새 목표: PID에 쌓인 값(I)과 이전 오차(D)를 지움
        self.get_logger().info(f'새 목표: ({goal[0]:.2f}, {goal[1]:.2f}), {math.degrees(goal[2]):.0f}도')

    def cancel(self):
        """목표 취소: 목표와 경로를 지우고 멈춤 (다음 반복에서 IDLE, 정지 명령 한 번)"""
        if self.goal is None:
            return
        self.goal = None
        self.path = None
        self.follower.reset()
        self.publish_path()             # 화면의 경로도 지움
        self.get_logger().info('목표 취소 (nav_enable = false)')

    def on_world(self, msg):
        """Localization의 WorldModel: 내 위치, 공, 지금 보이는 꼬깔 (모두 필드 기준)"""
        self.robot = (msg.robot_pose.x, msg.robot_pose.y, msg.robot_pose.theta)
        self.localized = msg.is_localized
        # 공: 보일 때만 갱신 (안 보이면 마지막으로 본 위치). 5cm 넘게 움직였으면 다시 계획
        if msg.ball_detected:
            ball = (msg.ball_pose.x, msg.ball_pose.y)
            if self.ball is None or math.hypot(ball[0] - self.ball[0], ball[1] - self.ball[1]) > 0.05:
                self.need_replan = True
            self.ball = ball
        # 꼬깔: 지금 보이는 것만 오니까 기억해서 씀
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.memory.update_field(self.robot, [(p.x, p.y) for p in msg.obstacles], now):
            self.need_replan = True     # 꼬깔이 새로 보이거나 사라지면 바로 다시 계획
        self.obstacles = [(x, y, self.cone_radius) for x, y in self.memory.confirmed()]

    def on_teammate(self, msg):
        self.teammate = (msg.x, msg.y)
        self.teammate_time = self.get_clock().now()
        self.teammate_speed.update(msg.x, msg.y, self.teammate_time.nanoseconds * 1e-9)
        # 다른 로봇이 계획할 때보다 15cm 넘게 움직였으면 바로 다시 계획
        if (self.teammate_planned is None
                or math.hypot(msg.x - self.teammate_planned[0], msg.y - self.teammate_planned[1]) > 0.15):
            self.need_replan = True

    def teammate_now(self):
        """믿을 만한 다른 로봇 위치 (1초 넘게 안 왔으면 None)"""
        if self.teammate is None:
            return None
        age = (self.get_clock().now() - self.teammate_time).nanoseconds * 1e-9
        return self.teammate if age < 1.0 else None

    # ---------------- 경로 계획 ----------------
    def replan(self):
        ball = self.ball if self.avoid_ball else None
        mate = self.teammate_now()
        self.teammate_planned = mate
        # 다른 로봇 = 움직이는 꼬깔 (다른 로봇 때문에 길이 없으면 빼고 다시: 곧 비킬 테니까)
        self.path = plan_with_teammate(self.planner, self.robot[:2], self.goal[:2], self.obstacles,
                                       mate, self.teammate_radius,
                                       ball, self.ball_radius)
        self.last_plan_time = self.get_clock().now()
        self.need_replan = False
        if self.path is None:
            # 같은 경고가 계속 쌓이지 않게 5초에 한 번만 출력
            self.get_logger().warn(
                '경로를 찾지 못함 (목표가 꼬깔·공·다른 로봇 범위 안, 필드 선 근처이거나 길이 막힘)', throttle_duration_sec=5.0)
        self.publish_path()

    def time_to_replan(self):
        if self.need_replan or self.last_plan_time is None:
            return True
        elapsed = (self.get_clock().now() - self.last_plan_time).nanoseconds * 1e-9
        return elapsed > self.replan_period

    # ---------------- 반복 ----------------
    def loop(self):
        cmd = Twist()   # 기본값 = 정지

        if self.robot is None or (self.require_localized and not self.localized):
            self.publish(cmd, 'NO_POSE')       # 위치를 모르거나 못 믿으면 걷지 않는다
            return
        if self.goal is None:
            self.publish(cmd, 'IDLE')
            self.publish_debug(None)
            return
        if self.time_to_replan():
            self.replan()
        if self.path is None:
            self.publish(cmd, 'BLOCKED')       # 갈 길이 없으면 멈추고 알린다
            self.publish_debug(None)
            return

        # 지난 계산 후 흐른 시간 (PID의 I, D 계산에 필요)
        t = self.get_clock().now()
        dt = (t - self.last_step_time).nanoseconds * 1e-9 if self.last_step_time else 0.0
        if dt <= 0.0 or dt > 0.5:
            dt = 1.0 / self.rate   # 처음이거나 너무 오래 멈췄으면 기본 주기로
        self.last_step_time = t

        # 다른 로봇: 바로 앞에 붙었으면 안전 정지, 양보하는 로봇이면 기다림
        mate = self.teammate_now()
        if mate is not None and len(self.path) >= 2:
            toward = look_ahead_point(self.path, self.robot, self.follower.carrot_points)
            if (too_close(self.robot, toward, mate, self.stop_dist) or
                    (self.yield_to_teammate and
                     should_yield(self.robot, toward, mate, self.teammate_speed.speed, self.yield_dist))):
                self.remain = path_remaining(self.path, self.robot)
                self.publish(cmd, 'WAITING')
                self.publish_debug(None)
                return

        c = self.follower.step(self.robot, self.path, self.goal, dt)
        cmd.linear.x, cmd.linear.y, cmd.angular.z = c.front, c.left, c.turn
        self.remain = c.remain
        self.publish(cmd, 'ARRIVED' if c.arrived else 'MOVING')
        self.publish_debug(None if c.arrived else c.carrot)

    def publish(self, cmd, status):
        # 목표가 없을 때(IDLE)는 정지 명령을 한 번만 보내고 조용히 있음
        # (Vision이 공 앞에서 보행 명령을 보낼 때 우리 "0"과 섞이지 않게)
        if not (status == 'IDLE' and self.last_status == 'IDLE'):
            self.cmd_pub.publish(cmd)
            fp = self.follower.p
            x, y, yaw, flag = to_walk(cmd.linear.x, cmd.linear.y, cmd.angular.z,
                                      fp.max_move, fp.max_side_near, fp.max_turn, *self.ik_max)
            self.walk_pub.publish(Master2IkMsg(x_length=x, y_length=y, yaw=yaw, flag=flag))
        # 도착 예상 시간: 가는 중(기다리는 중 포함)이면 남은 경로 길이 / 최대 앞걸음 속도
        #                  도착 0, 대기 0, 길 없음·위치 모름 -1
        if status in ('MOVING', 'WAITING'):
            eta = self.remain / max(self.follower.p.max_move, 1e-3)
        elif status in ('ARRIVED', 'IDLE'):
            eta = 0.0
        else:
            eta = -1.0
        self.feedback_pub.publish(NavFeedback(status=STATUS_CODE[status], eta_seconds=float(eta)))
        if status != self.last_status:      # 상태가 바뀔 때만 화면에 출력
            self.get_logger().info(f'상태: {status}')
            self.last_status = status

    # ---------------- 화면 표시 (UI) ----------------
    def publish_path(self):
        msg = Path()
        msg.header.frame_id = FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in (self.path or []):
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def make_marker(self, ns, mid, mtype, r, g, b, a=1.0):
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        return m

    def publish_debug(self, carrot):
        arr = MarkerArray()
        clear = Marker()
        clear.header.frame_id = FRAME
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        # 기억한 꼬깔의 위험 범위 (연한 주황 원판) = 로봇 중심이 들어가면 안 되는 곳
        for k, (ox, oy, r) in enumerate(self.obstacles):
            m = self.make_marker('inflated', k, Marker.CYLINDER, 1.0, 0.5, 0.0, 0.25)
            m.pose.position.x, m.pose.position.y = float(ox), float(oy)
            m.scale.x = m.scale.y = 2.0 * self.planner.inflated_radius(r)
            m.scale.z = 0.01
            arr.markers.append(m)

        # 로봇 중심이 다닐 수 있는 범위 (필드 선 안쪽 boundary_margin, 주황 점선 대신 얇은 선)
        #   LEFT 진영만 다닐 때는 오른쪽 끝이 가운데선 왼쪽 금지선 (빨간색)
        b = self.planner.boundary_margin
        if b > 0.0 or self.planner.max_x is not None:
            hl, hw = self.field_l / 2.0 - b, self.field_w / 2.0 - b
            right = hl if self.planner.max_x is None else min(hl, self.planner.max_x)
            red = self.planner.max_x is not None
            m = self.make_marker('boundary', 0, Marker.LINE_STRIP, 1.0, 0.2 if red else 0.6, 0.1, 0.8)
            m.scale.x = 0.03 if red else 0.02
            m.points = [Point(x=x, y=y) for x, y in
                        [(-hl, -hw), (right, -hw), (right, hw), (-hl, hw), (-hl, -hw)]]
            arr.markers.append(m)

        # 공의 위험 범위 (연한 하얀 원판) = 로봇 중심이 들어가면 공을 밀게 되는 곳
        if self.ball is not None and self.avoid_ball:
            m = self.make_marker('ball_zone', 0, Marker.CYLINDER, 1.0, 1.0, 1.0, 0.2)
            m.pose.position.x, m.pose.position.y = float(self.ball[0]), float(self.ball[1])
            m.scale.x = m.scale.y = 2.0 * self.planner.inflated_radius(self.ball_radius)
            m.scale.z = 0.01
            arr.markers.append(m)

        # 다른 로봇의 위험 범위 (연한 보라 원판)
        mate = self.teammate_now()
        if mate is not None:
            m = self.make_marker('teammate', 0, Marker.CYLINDER, 0.6, 0.3, 0.9, 0.25)
            m.pose.position.x, m.pose.position.y = float(mate[0]), float(mate[1])
            m.scale.x = m.scale.y = 2.0 * self.planner.inflated_radius(self.teammate_radius)
            m.scale.z = 0.01
            arr.markers.append(m)

        # 당근 (노란 공)
        if carrot is not None:
            m = self.make_marker('carrot', 0, Marker.SPHERE, 1.0, 0.9, 0.0)
            m.pose.position.x, m.pose.position.y = float(carrot[0]), float(carrot[1])
            m.scale.x = m.scale.y = m.scale.z = 0.12
            arr.markers.append(m)

        # 목표 (빨간 화살표 = 도착 후 바라볼 방향)
        if self.goal is not None:
            m = self.make_marker('goal', 0, Marker.ARROW, 0.9, 0.1, 0.1)
            m.pose.position.x, m.pose.position.y = float(self.goal[0]), float(self.goal[1])
            m.pose.orientation.z = math.sin(self.goal[2] / 2.0)
            m.pose.orientation.w = math.cos(self.goal[2] / 2.0)
            m.scale.x, m.scale.y, m.scale.z = 0.4, 0.12, 0.12
            arr.markers.append(m)

        # 상태 글자 (로봇 위에 표시): 예) MOVING 2.35m
        if self.robot is not None:
            s = self.last_status
            r = 0.3 if s == 'ARRIVED' else 1.0     # 이동 중 흰색, 도착 초록, 막힘 빨강
            g = 0.3 if s == 'BLOCKED' else 1.0
            b = 1.0 if s in ('MOVING', 'IDLE', 'WAITING') else 0.3
            m = self.make_marker('status', 0, Marker.TEXT_VIEW_FACING, r, g, b)
            m.pose.position.x = float(self.robot[0])
            m.pose.position.y = float(self.robot[1]) + 0.45
            m.pose.position.z = 0.5
            m.scale.z = 0.25    # 글자 높이 (m)
            m.text = f'MOVING {self.remain:.2f}m' if s == 'MOVING' else s
            arr.markers.append(m)

        self.debug_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                    # Ctrl+C 로 종료
    except Exception:
        if rclpy.ok():
            raise               # 실행 중에 난 진짜 오류는 그대로 보여줌
        # 종료하는 순간에 생긴 오류는 무시
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
