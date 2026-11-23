"""
내비게이션 UI (연습·디버깅용, Localization UI와 같은 배치)

  왼쪽 위   필드: 로봇, 꼬깔, 공, A-star 경로, 위험 범위, 카메라 시야, 당근, 목표
  오른쪽 위  로봇 기준 화면: 로봇이 바라보는 방향이 위쪽, 걸음 명령 화살표, 상태
  왼쪽 아래  조작할 로봇 고르기 (Robot 1 / Robot 2)
            로봇 / 목표 / 공 위치 입력 + Set, Go, Reset 버튼
  오른쪽 아래 튜닝 탭 (PID, 경로, 가짜 로봇) + Text (기록)

필드 위 마우스:
  왼쪽 클릭 후 끌기          = 목표 (끈 방향 = 도착 후 바라볼 방향)
  오른쪽 클릭               = 공 옮기기
  Ctrl + 왼쪽 클릭 후 끌기   = 로봇 옮기기

로봇마다 (robot1, robot2):
  받는 토픽:  /robotN/robot_pose, /robotN/nav_path, /robotN/nav_feedback (NavFeedback), /robotN/nav_cmd,
              /robotN/nav_markers (가짜 로봇 그림), /robotN/nav_debug (내비게이션 그림)
  보내는 토픽: /robotN/nav_command (NavCommand, Taskplanner 대신: 목표, Stop, Ready, 공 피하기),
              /robotN/initialpose (로봇 옮기기)
같이 쓰는 토픽: /ball_pose (받음), /clicked_point (공 옮기기, 보냄)
공 앞으로 버튼: 고른 로봇에게 "공 앞" 목표를 보냄
  공 앞 = 공에서 상대 골대(오른쪽, x = +4.5) 반대쪽으로 "공 앞 거리"만큼 뒤, 골대 쪽 바라보기
  (그 안쪽에서 골대 방향 맞추기·킥은 Vision이 맡음)
튜닝 값은 ROS2 파라미터로 바꿈 (ros2 param set 과 같음)
"""
import math
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from geometry_msgs.msg import Point, PointStamped, Pose2D, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from humanoid_interfaces.msg import NavCommand, NavFeedback
from visualization_msgs.msg import Marker, MarkerArray

from python_qt_binding.QtCore import QPointF, QRectF, Qt, QTimer
from python_qt_binding.QtGui import QBrush, QColor, QFont, QPainter, QPen, QPolygonF
from python_qt_binding.QtWidgets import (
    QApplication, QCheckBox, QDoubleSpinBox, QFrame, QGridLayout, QHBoxLayout, QLabel,
    QMainWindow, QPlainTextEdit, QPushButton, QSpinBox, QTabWidget, QVBoxLayout, QWidget)

from robit_navigation.ball_approach import ball_front_goal
from robit_navigation.nav_core import field_to_robot

FRAME = 'map'
FIELD_L, FIELD_W = 9.0, 6.0      # 필드 크기 (m, 예시 값)
BORDER = 0.7                     # 필드 선 바깥 여백 (m)
# 로봇별 색 (필드 그림, 경로, 지나온 길)
ROBOT_COLORS = ['#1e63d6', '#9c27b0', '#00897b', '#f4511e']
TRAIL_COLORS = [(20, 40, 120, 200), (90, 20, 110, 200), (0, 80, 70, 200), (140, 50, 0, 200)]
PATH_COLORS = [(40, 110, 255), (200, 80, 230), (0, 180, 160), (255, 120, 40)]

STATUS_COLOR = {'IDLE': '#ffffff', 'MOVING': '#ffe14d', 'ARRIVED': '#5dff7a', 'WAITING': '#4dd0ff',
                'BLOCKED': '#ff5a5a', 'NO_POSE': '#ff9a3c'}

# 튜닝 탭: (파라미터 이름, 화면 이름, 최소, 최대, 한 칸, 소수 자리)
PID_PARAMS = [
    ('kp_move', 'Kp 이동', 0, 5, 0.1, 2), ('ki_move', 'Ki 이동', 0, 5, 0.05, 2),
    ('kd_move', 'Kd 이동', 0, 5, 0.05, 2), ('kp_turn', 'Kp 회전', 0, 5, 0.1, 2),
    ('ki_turn', 'Ki 회전', 0, 5, 0.05, 2), ('kd_turn', 'Kd 회전', 0, 5, 0.05, 2),
    ('max_move', '최대 앞걸음 m/s', 0, 1, 0.05, 2), ('max_turn', '최대 회전 rad/s', 0, 3, 0.1, 2),
    ('turn_in_place_deg', '제자리 회전 각도', 0, 180, 5, 1), ('walk_again_deg', '다시 걷기 각도', 0, 180, 5, 1),
    ('near_dist', '옆걸음 거리 m', 0, 2, 0.05, 2), ('max_side', '옆걸음 최대(멀 때)', 0, 1, 0.01, 2),
    ('max_side_near', '옆걸음 최대(가까울 때)', 0, 1, 0.01, 2), ('max_back', '뒷걸음 최대', 0, 1, 0.01, 2),
]
PATH_PARAMS = [
    ('robot_radius', '로봇 반지름 m', 0, 1, 0.01, 2), ('margin', '여유 m', 0, 1, 0.01, 2),
    ('cone_radius', '꼬깔 반지름 m', 0, 1, 0.01, 2), ('ball_radius', '공 반지름 m', 0, 1, 0.01, 3),
    ('tol_dist', '허용 거리 m', 0, 1, 0.01, 2),
    ('tol_yaw_deg', '허용 각도 도', 0, 90, 1, 1), ('carrot_points', '당근 (점 개수)', 1, 30, 1, 0),
    ('boundary_margin', '선 안쪽 여유 m', 0, 1, 0.05, 2), ('center_margin', '가운데선 여유 m', 0, 1, 0.05, 2),
]

STYLE = """
QWidget { background: #55575b; color: #f0f0f0; font-size: 13px; }
QPushButton { background: #2e3337; border: none; border-radius: 8px; padding: 8px 14px; color: #ffffff; }
QPushButton:hover { background: #3c4247; }
QPushButton:pressed { background: #23272a; }
QLabel#tag { border: 1px dashed #e8e8e8; padding: 3px 6px; }
QDoubleSpinBox, QSpinBox { background: #ffffff; color: #111111; padding: 2px; min-width: 64px; }
QPlainTextEdit { background: #ffffff; color: #111111; font-family: monospace; font-size: 12px; }
QTabWidget::pane { border: 1px solid #e8e8e8; }
QTabBar::tab { background: #55575b; border: 1px solid #e8e8e8; padding: 4px 10px; }
QTabBar::tab:selected { background: #6a6d72; }
QFrame#logo { border: 2px solid #e8e8e8; }
"""


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def qcolor(c, alpha_scale=1.0):
    return QColor(int(c.r * 255), int(c.g * 255), int(c.b * 255), int(min(1.0, c.a * alpha_scale) * 255))


# ======================================================================
class NavGuiNode(Node):
    """ROS2 부분: 로봇마다 토픽을 받아서 저장해 두고, 버튼을 누르면 고른 로봇에게 토픽·파라미터를 보낸다"""

    def __init__(self):
        super().__init__('nav_gui_node')
        # 로봇 이름 목록과 Reset 위치 (launch에서 줌)
        self.declare_parameter('robots', ['robot1', 'robot2'])
        self.declare_parameter('starts_x', [-2.25, -2.25])
        self.declare_parameter('starts_y', [-3.1, 3.1])
        self.declare_parameter('starts_yaw_deg', [90.0, -90.0])
        # 준비 위치 (Ready 때 스스로 걸어갈 곳, LEFT 진영이면 자유)
        self.declare_parameter('readys_x', [-0.8, -0.8])
        self.declare_parameter('readys_y', [-1.2, 1.2])
        self.declare_parameter('readys_yaw_deg', [0.0, 0.0])
        p = lambda n: list(self.get_parameter(n).value)
        self.names = p('robots')
        self.starts = {n: (x, y, a) for n, x, y, a in
                       zip(self.names, p('starts_x'), p('starts_y'), p('starts_yaw_deg'))}
        self.readys = {n: (x, y, a) for n, x, y, a in
                       zip(self.names, p('readys_x'), p('readys_y'), p('readys_yaw_deg'))}

        self.data = {n: {'robot': None, 'path': [], 'status': '-', 'cmd': (0.0, 0.0, 0.0), 'eta': -1.0}
                     for n in self.names}
        self.selected = self.names[0]   # 지금 조작하는 로봇
        self.ball = None           # (x, y)
        self.markers = {}          # (로봇, 토픽, ns, id) -> Marker
        self.log = None            # 화면 기록 함수 (창에서 연결)
        self.nav_pub, self.init_pub, self.params = {}, {}, {}
        # 로봇마다 Taskplanner 대신 보내는 NavCommand 내용 (바뀔 때마다 다시 보냄)
        self.command = {n: NavCommand(nav_enable=False, avoid_ball=True) for n in self.names}

        for n in self.names:
            t = f'/{n}/' if n else '/'
            self.create_subscription(Pose2D, t + 'robot_pose', lambda m, n=n: self.on_pose(n, m), 10)
            self.create_subscription(Path, t + 'nav_path', lambda m, n=n: self.on_path(n, m), 10)
            self.create_subscription(NavFeedback, t + 'nav_feedback', lambda m, n=n: self.on_feedback(n, m), 10)
            self.create_subscription(Twist, t + 'nav_cmd', lambda m, n=n: self.on_cmd(n, m), 10)
            self.create_subscription(MarkerArray, t + 'nav_markers',
                                     lambda m, n=n: self.on_markers(n, 'fake', m), 10)
            self.create_subscription(MarkerArray, t + 'nav_debug',
                                     lambda m, n=n: self.on_markers(n, 'nav', m), 10)
            self.nav_pub[n] = self.create_publisher(NavCommand, t + 'nav_command', 10)
            self.init_pub[n] = self.create_publisher(PoseWithCovarianceStamped, t + 'initialpose', 10)
            for kind in ('navigation_node',):
                self.params[(n, kind)] = AsyncParameterClient(self, t + kind)

        self.create_subscription(PointStamped, '/ball_pose', self.on_ball, 10)
        self.ball_pub = self.create_publisher(PointStamped, '/clicked_point', 10)


    # 고른 로봇의 값 (로봇 기준 화면에서 씀)
    @property
    def robot(self):
        return self.data[self.selected]['robot']

    @property
    def path(self):
        return self.data[self.selected]['path']

    @property
    def status(self):
        return self.data[self.selected]['status']

    @property
    def cmd(self):
        return self.data[self.selected]['cmd']

    # ---------- 받기 ----------
    def on_pose(self, n, m):
        self.data[n]['robot'] = (m.x, m.y, m.theta)

    def on_ball(self, m):
        self.ball = (m.point.x, m.point.y)

    def on_path(self, n, m):
        self.data[n]['path'] = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    def on_feedback(self, n, m):
        """NavFeedback: 상태 숫자 -> 글자, 도착 예상 시간"""
        text = {0: 'IDLE', 1: 'MOVING', 2: 'ARRIVED', 3: 'BLOCKED'}.get(m.status, str(m.status))
        if text != self.data[n]['status'] and self.log:
            self.log(f'{n} 상태: {text}')
        self.data[n]['status'] = text
        self.data[n]['eta'] = m.eta_seconds

    def on_cmd(self, n, m):
        self.data[n]['cmd'] = (m.linear.x, m.linear.y, m.angular.z)

    def on_markers(self, n, topic, arr):
        for mk in arr.markers:
            if mk.action == Marker.DELETEALL:
                self.markers = {k: v for k, v in self.markers.items() if k[:2] != (n, topic)}
            elif mk.action == Marker.DELETE:
                self.markers.pop((n, topic, mk.ns, mk.id), None)
            else:
                self.markers[(n, topic, mk.ns, mk.id)] = mk

    # ---------- 보내기 (고른 로봇에게) ----------
    def publish_command(self, n):
        self.nav_pub[n].publish(self.command[n])

    def send_goal(self, x, y, yaw, n=None):
        """목표 보내기: nav_enable = true + 목표 (허용 오차 0 = 내비게이션 기본값)"""
        n = n or self.selected
        c = self.command[n]
        c.nav_enable, c.x, c.y, c.theta = True, float(x), float(y), float(yaw)
        self.publish_command(n)
        self.log(f'{n} 목표 보냄: ({x:.2f}, {y:.2f}), {math.degrees(yaw):.0f}도')

    def send_left_only(self, on):
        """Ready: 두 로봇 모두 RIGHT 진영 금지 (게임 상태라서 같이)"""
        for n in self.names:
            self.command[n].forbid_right_half = bool(on)
            self.publish_command(n)
        self.log('Ready: RIGHT 진영 금지' if on else 'Playing: RIGHT 진영 금지 해제')

    def send_avoid_ball(self, on, n=None):
        n = n or self.selected
        self.command[n].avoid_ball = bool(on)
        self.publish_command(n)
        self.log(f'{n} 공 피하기 {"켬" if on else "끔"}')

    def send_cancel(self, n=None):
        """멈추기: nav_enable = false"""
        n = n or self.selected
        self.command[n].nav_enable = False
        self.publish_command(n)
        self.log(f'{n} 멈춤 (nav_enable = false)')

    def send_robot(self, x, y, yaw, n=None):
        n = n or self.selected
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.init_pub[n].publish(msg)
        self.log(f'{n} 옮김: ({x:.2f}, {y:.2f}), {math.degrees(yaw):.0f}도')

    def send_ball(self, x, y):
        msg = PointStamped()
        msg.header.frame_id = FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point = Point(x=float(x), y=float(y))
        self.ball_pub.publish(msg)
        self.log(f'공 옮김: ({x:.2f}, {y:.2f})')

    # ---------- 파라미터 (고른 로봇의 노드) ----------
    def set_params(self, kind, values):
        """values: {이름: 값}. ros2 param set 과 같은 일"""
        client = self.params[(self.selected, kind)]
        if not client.services_are_ready():
            self.log(f'{self.selected}/{kind} 이(가) 실행 중이 아님')
            return
        names, who = list(values), self.selected
        future = client.set_parameters([Parameter(n, value=values[n]) for n in names])

        def done(f):
            res = f.result()
            if res is None:
                self.log('파라미터 변경 실패 (응답 없음)')
                return
            bad = [f'{n}: {r.reason}' for n, r in zip(names, res.results) if not r.successful]
            self.log(f'{who} 적용 완료: ' + ', '.join(f'{n}={values[n]}' for n in names) if not bad
                     else f'{who} 거부됨 -> ' + '; '.join(bad))
        future.add_done_callback(done)

    def get_params(self, kind, names, on_values):
        """고른 로봇의 지금 값을 읽어서 on_values({이름: 값}) 호출"""
        client = self.params[(self.selected, kind)]
        if not client.services_are_ready():
            return
        future = client.get_parameters(names)

        def done(f):
            res = f.result()
            if res is not None:
                on_values({n: parameter_value_to_python(v) for n, v in zip(names, res.values)})
        future.add_done_callback(done)


# ======================================================================
class FieldView(QWidget):
    """필드 그림 (필드 좌표: 오른쪽 +x, 위쪽 +y)"""

    def __init__(self, node, window):
        super().__init__()
        self.node, self.win = node, window
        self.setMinimumSize(560, 400)
        self.drag = None   # ('goal' 또는 'robot', 시작점, 지금 점)

    # 필드 m <-> 화면 픽셀
    def scale(self):
        return min(self.width() / (FIELD_L + 2 * BORDER), self.height() / (FIELD_W + 2 * BORDER))

    def to_px(self, x, y):
        s = self.scale()
        return QPointF(self.width() / 2 + x * s, self.height() / 2 - y * s)

    def to_field(self, p):
        s = self.scale()
        return (p.x() - self.width() / 2) / s, (self.height() / 2 - p.y()) / s

    # ---------- 마우스 ----------
    def mousePressEvent(self, e):
        x, y = self.to_field(e.pos())
        if e.button() == Qt.RightButton:
            self.win.set_ball_inputs(x, y)
            self.node.send_ball(x, y)
        elif e.button() == Qt.LeftButton:
            mode = 'robot' if e.modifiers() & Qt.ControlModifier else 'goal'
            self.drag = (mode, (x, y), (x, y))

    def mouseMoveEvent(self, e):
        if self.drag:
            self.drag = (self.drag[0], self.drag[1], self.to_field(e.pos()))

    def mouseReleaseEvent(self, e):
        if not self.drag or e.button() != Qt.LeftButton:
            return
        mode, (x0, y0), (x1, y1) = self.drag
        self.drag = None
        if math.hypot(x1 - x0, y1 - y0) > 0.05:
            yaw = math.atan2(y1 - y0, x1 - x0)          # 끈 방향
        else:
            yaw = 0.0
        if mode == 'goal':
            self.win.set_goal_inputs(x0, y0, yaw)
            self.node.send_goal(x0, y0, yaw)
        else:
            self.win.set_robot_inputs(x0, y0, yaw)
            self.node.send_robot(x0, y0, yaw)

    # ---------- 그리기 ----------
    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        s = self.scale()
        p.fillRect(self.rect(), QColor('#4ab53c'))
        self.draw_field(p, s)
        self.draw_markers(p, s)

        # 로봇마다: A-star 경로, 로봇 (실제 크기 25 x 17cm 네모, 앞면 25cm)
        for k, n in enumerate(self.node.names):
            d = self.node.data[n]
            if len(d['path']) >= 2:
                p.setPen(QPen(QColor(*PATH_COLORS[k % 4]), max(2.0, 0.03 * s)))
                p.setBrush(Qt.NoBrush)
                p.drawPolyline(QPolygonF([self.to_px(x, y) for x, y in d['path']]))
        for k, n in enumerate(self.node.names):
            if self.node.data[n]['robot'] is not None:
                self.draw_robot(p, *self.node.data[n]['robot'], s, k, n == self.node.selected)

        # 끄는 중인 목표/로봇 화살표 미리보기
        if self.drag:
            mode, (x0, y0), (x1, y1) = self.drag
            col = QColor(230, 30, 30) if mode == 'goal' else QColor(30, 200, 60)
            p.setPen(QPen(col, 3))
            p.drawLine(self.to_px(x0, y0), self.to_px(x1, y1))
            p.setBrush(col)
            p.drawEllipse(self.to_px(x0, y0), 5, 5)

        # 도움말
        p.setPen(QColor(255, 255, 255, 200))
        p.setFont(QFont('Sans', 9))
        p.drawText(8, self.height() - 8,
                   '왼쪽 끌기: 목표   오른쪽 클릭: 공   Ctrl+왼쪽 끌기: 로봇')
        p.end()

    def draw_field(self, p, s):
        hl, hw = FIELD_L / 2, FIELD_W / 2
        line = QPen(QColor('#ffffff'), max(2.0, 0.05 * s))
        p.setPen(line)
        p.setBrush(Qt.NoBrush)

        def rect(x0, y0, x1, y1):
            a, b = self.to_px(x0, y1), self.to_px(x1, y0)
            p.drawRect(QRectF(a, b))

        rect(-hl, -hw, hl, hw)                                   # 바깥선
        p.drawLine(self.to_px(0, -hw), self.to_px(0, hw))        # 센터라인
        r = 0.75 * s                                             # 센터서클 (지름 1.5m)
        p.drawEllipse(self.to_px(0, 0), r, r)
        for side in (-1, 1):
            gx = side * hl
            rect(min(gx, gx - side * 1.0), -1.5, max(gx, gx - side * 1.0), 1.5)   # 골 에어리어 1 x 3m
            rect(min(gx, gx - side * 2.0), -2.5, max(gx, gx - side * 2.0), 2.5)   # 페널티 에어리어 2 x 5m
            rect(min(gx, gx + side * 0.6), -1.3, max(gx, gx + side * 0.6), 1.3)   # 골대 (바깥쪽)
            p.setBrush(QColor('#ffffff'))
            p.drawEllipse(self.to_px(side * (hl - 1.5), 0), 0.06 * s, 0.06 * s)   # 페널티 마크
            p.setBrush(Qt.NoBrush)

    def draw_markers(self, p, s):
        def order(mk):
            # 연한 원판(위험 범위) -> 선 -> 물체 -> 화살표 -> 글자 순서로 그림
            rank = {Marker.CYLINDER: 2, Marker.LINE_STRIP: 1, Marker.LINE_LIST: 1,
                    Marker.SPHERE: 3, Marker.ARROW: 4, Marker.TEXT_VIEW_FACING: 5}.get(mk.type, 9)
            return (0 if mk.color.a < 0.5 and mk.type == Marker.CYLINDER else 1, rank)

        index = {n: k for k, n in enumerate(self.node.names)}
        for (who, topic, ns, mid), mk in sorted(self.node.markers.items(), key=lambda kv: order(kv[1])):
            if topic == 'fake' and ns == 'nav' and mid in (0, 1, 5):
                continue        # 가짜 로봇의 필드선·로봇 화살표·바닥판은 이 화면에서 직접 그림
            if topic == 'fake' and mid >= 10 and who != self.node.names[0]:
                continue        # 꼬깔은 가짜 로봇마다 같으니 한 번만 그림
            if topic == 'nav' and who != self.node.selected:
                continue        # 내비게이션 그림(위험 범위, 목표, 당근)은 고른 로봇 것만
            pos = mk.pose.position
            c = self.to_px(pos.x, pos.y)
            if mk.type in (Marker.CYLINDER, Marker.SPHERE):
                r = mk.scale.x / 2 * s
                p.setPen(Qt.NoPen if mk.color.a < 0.5 else QPen(QColor(0, 0, 0, 120), 1))
                p.setBrush(qcolor(mk.color, 1.6 if mk.color.a < 0.5 else 1.0))
                p.drawEllipse(c, max(r, 2.0), max(r, 2.0))
            elif mk.type in (Marker.LINE_STRIP, Marker.LINE_LIST):
                col = qcolor(mk.color)
                if topic == 'fake' and mid == 2:
                    col = QColor(*TRAIL_COLORS[index[who] % 4])   # 지나온 길: 초록 필드에서 잘 보이게 진하게
                p.setPen(QPen(col, max(1.5, mk.scale.x * s)))
                pts = [self.to_px(q.x, q.y) for q in mk.points]
                if mk.type == Marker.LINE_STRIP and len(pts) >= 2:
                    p.drawPolyline(QPolygonF(pts))
                else:
                    for a, b in zip(pts[0::2], pts[1::2]):
                        p.drawLine(a, b)
            elif mk.type == Marker.ARROW:
                self.draw_arrow(p, pos.x, pos.y, yaw_of(mk.pose.orientation),
                                mk.scale.x, qcolor(mk.color), s)
            elif mk.type == Marker.TEXT_VIEW_FACING:
                p.setPen(qcolor(mk.color))
                p.setFont(QFont('Sans', 10, QFont.Bold))
                p.drawText(c - QPointF(30, 0), mk.text)

    def draw_robot(self, p, x, y, yaw, s, k=0, selected=True):
        c, sn = math.cos(yaw), math.sin(yaw)

        def corner(f, l):     # 로봇 기준 (앞, 왼쪽) -> 화면
            return self.to_px(x + c * f - sn * l, y + sn * f + c * l)

        body = QPolygonF([corner(0.085, 0.125), corner(0.085, -0.125),
                          corner(-0.085, -0.125), corner(-0.085, 0.125)])
        edge = QColor('#ffe14d') if selected else QColor('#ffffff')   # 고른 로봇은 노란 테두리
        p.setPen(QPen(edge, 3 if selected else 2))
        p.setBrush(QColor(ROBOT_COLORS[k % 4]))
        p.drawPolygon(body)
        p.setPen(QPen(QColor('#ffffff'), 3))
        p.drawLine(self.to_px(x, y), corner(0.3, 0.0))    # 바라보는 방향
        p.setPen(QColor('#ffffff'))
        p.setFont(QFont('Sans', 9, QFont.Bold))
        p.drawText(self.to_px(x, y) + QPointF(-4, 22), str(k + 1))   # 로봇 번호

    def draw_arrow(self, p, x, y, yaw, length, col, s):
        tip = (x + length * math.cos(yaw), y + length * math.sin(yaw))
        left = (tip[0] - 0.15 * math.cos(yaw - 0.5), tip[1] - 0.15 * math.sin(yaw - 0.5))
        right = (tip[0] - 0.15 * math.cos(yaw + 0.5), tip[1] - 0.15 * math.sin(yaw + 0.5))
        p.setPen(QPen(col, max(3.0, 0.06 * s)))
        p.drawLine(self.to_px(x, y), self.to_px(*tip))
        p.setBrush(col)
        p.drawPolygon(QPolygonF([self.to_px(*tip), self.to_px(*left), self.to_px(*right)]))


# ======================================================================
class RobotView(QWidget):
    """로봇 기준 화면: 로봇이 가운데 아래, 바라보는 방향이 위쪽. 걸음 명령을 화살표로"""

    RANGE = 3.0    # 위쪽으로 보이는 거리 (m)

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setMinimumSize(380, 300)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor('#000000'))
        w, h = self.width(), self.height()
        s = h * 0.8 / self.RANGE
        ox, oy = w / 2, h * 0.85               # 로봇 위치 (화면)

        def px(front, left):                   # 로봇 기준 (앞, 왼쪽) -> 화면
            return QPointF(ox - left * s, oy - front * s)

        # 거리 눈금 (1m, 2m, 3m)
        p.setPen(QPen(QColor(70, 70, 70), 1))
        p.setBrush(Qt.NoBrush)
        p.setFont(QFont('Sans', 8))
        for d in (1, 2, 3):
            p.drawEllipse(QPointF(ox, oy), d * s, d * s)
            p.drawText(QPointF(ox + 4, oy - d * s + 12), f'{d}m')

        robot = self.node.robot
        if robot is not None:
            # 카메라 시야 (60도)
            p.setPen(QPen(QColor(90, 150, 255, 160), 1))
            for a in (-30, 30):
                r = math.radians(a)
                p.drawLine(px(0, 0), px(3 * math.cos(r), 3 * math.sin(r)))
            # 내비게이션이 아는 위험 범위, 공, 당근, 목표 (필드 -> 로봇 기준)
            for (who, topic, ns, _), mk in self.node.markers.items():
                if who != self.node.selected or topic != 'nav' or \
                        ns not in ('inflated', 'ball_zone', 'teammate', 'carrot', 'goal'):
                    continue
                f, l = field_to_robot(robot, mk.pose.position.x, mk.pose.position.y)
                r = mk.scale.x / 2 * s
                if ns in ('inflated', 'ball_zone', 'teammate'):
                    p.setPen(Qt.NoPen)
                    p.setBrush(qcolor(mk.color, 1.6))
                    p.drawEllipse(px(f, l), r, r)
                elif ns == 'carrot':
                    p.setPen(Qt.NoPen)
                    p.setBrush(QColor(255, 230, 0))
                    p.drawEllipse(px(f, l), 6, 6)
                else:
                    p.setPen(QPen(QColor(230, 40, 40), 2))
                    p.setBrush(Qt.NoBrush)
                    p.drawEllipse(px(f, l), 8, 8)
            if self.node.ball is not None:
                f, l = field_to_robot(robot, *self.node.ball)
                p.setPen(Qt.NoPen)
                p.setBrush(QColor('#ffffff'))
                p.drawEllipse(px(f, l), max(4.0, 0.07 * s), max(4.0, 0.07 * s))
            if len(self.node.path) >= 2:
                p.setPen(QPen(QColor(40, 110, 255), 2))
                p.setBrush(Qt.NoBrush)
                p.drawPolyline(QPolygonF([px(*field_to_robot(robot, x, y)) for x, y in self.node.path]))

        # 로봇 (25 x 17cm 네모, 앞면 25cm)
        p.setPen(QPen(QColor(60, 220, 90), 2))
        p.setBrush(QColor(60, 220, 90, 90))
        p.drawRect(QRectF(px(0.085, 0.125), px(-0.085, -0.125)))

        # 걸음 명령 화살표 (속도 1 m/s = 2m 길이로 크게)
        front, left, turn = self.node.cmd
        if abs(front) + abs(left) > 1e-3:
            p.setPen(QPen(QColor(255, 220, 0), 4))
            p.drawLine(px(0, 0), px(front * 2.0, left * 2.0))

        # 글자: 상태, 명령
        st = self.node.status
        p.setPen(QColor(STATUS_COLOR.get(st, '#ffffff')))
        p.setFont(QFont('Sans', 18, QFont.Bold))
        p.drawText(14, 32, st)
        p.setPen(QColor('#dddddd'))
        p.setFont(QFont('Sans', 10))
        p.drawText(14, 56, f'앞 {front:+.2f} m/s   왼쪽 {left:+.2f} m/s   회전 {turn:+.2f} rad/s')
        if robot is not None:
            p.drawText(14, 76, f'로봇 ({robot[0]:+.2f}, {robot[1]:+.2f})  {math.degrees(robot[2]):+.0f}도')
        eta = self.node.data[self.node.selected]['eta']
        p.drawText(14, 96, f'도착 예상 {eta:.1f} 초' if eta >= 0 else '도착 예상 -')
        p.setPen(QColor('#888888'))
        p.drawText(w - 170, 20, f'{self.node.selected} 기준 화면')
        p.end()


# ======================================================================
class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        node.log = self.log
        self.setWindowTitle('Navigation')
        self.setStyleSheet(STYLE)

        self.field = FieldView(node, self)
        self.robot_view = RobotView(node)
        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setMinimumWidth(240)

        top = QHBoxLayout()
        top.addWidget(self.field, 3)
        top.addWidget(self.robot_view, 2)
        bottom = QHBoxLayout()
        bottom.addLayout(self.make_inputs(), 3)
        bottom.addWidget(self.make_tabs(), 2)
        text_box = QVBoxLayout()
        text_box.addWidget(QLabel('Text'))
        text_box.addWidget(self.text)
        bottom.addLayout(text_box, 1)

        root = QVBoxLayout()
        root.addLayout(top, 3)
        root.addLayout(bottom, 2)
        body = QWidget()
        body.setLayout(root)
        self.setCentralWidget(body)
        self.resize(1500, 880)

        self.redraw = QTimer(self)
        self.redraw.timeout.connect(self.refresh)
        self.redraw.start(50)           # 1초에 20번 다시 그림
        self.log('Navigation UI 시작')
        # 로봇마다 입력칸 값을 따로 기억 (로봇을 바꿔도 입력한 값이 안 지워지게)
        #   Robot 줄 = 그 로봇 시작 위치, Goal 줄 = 예시 목표, Ready 줄 = 준비 위치
        self.saved = {}
        for k, n in enumerate(self.node.names):
            st = self.node.starts.get(n, (-3.0, -3.0, 90.0))
            rd = self.node.readys.get(n, (-0.8, 0.0, 0.0))
            self.saved[n] = {'robot': st, 'goal': (-1.5, 1.0 - 2.0 * k, 0.0), 'ready': rd}
        self.ready_wait = None     # Ready 중 아직 준비 위치에 도착 안 한 로봇들
        self.ready_sent = 0.0
        self.shown = None          # 지금 입력칸에 보이는 로봇
        self.select(self.node.selected)
        QTimer.singleShot(1500, self.read_all)   # 노드가 켜지면 지금 값 읽어오기

    # ---------- 왼쪽 아래: 위치 입력 ----------
    def spin(self, lo, hi, step, dec, val):
        b = QDoubleSpinBox()
        b.setRange(lo, hi)
        b.setSingleStep(step)
        b.setDecimals(dec)
        b.setValue(val)
        return b

    def tag(self, text):
        t = QLabel(text)
        t.setObjectName('tag')
        t.setAlignment(Qt.AlignCenter)
        return t

    def make_inputs(self):
        # 조작할 로봇 고르기
        pick = QHBoxLayout()
        pick.addWidget(self.tag('조작할 로봇'))
        self.pick_buttons = {}
        for k, n in enumerate(self.node.names):
            b = QPushButton(f'Robot {k + 1}')
            b.setCheckable(True)
            b.clicked.connect(lambda _, n=n: self.select(n))
            self.pick_buttons[n] = b
            pick.addWidget(b)
        pick.addSpacing(20)
        setting = QPushButton('모두 세팅 위치로')   # 규정: 경기 전 LEFT 진영 터치라인 세팅 위치에 놓기
        setting.clicked.connect(self.on_all_setting)
        pick.addWidget(setting)
        self.ready_btn = QPushButton('Ready')
        self.ready_btn.setCheckable(True)
        self.ready_btn.toggled.connect(self.on_ready)
        pick.addWidget(self.ready_btn)
        pick.addStretch()

        g = QGridLayout()
        g.setHorizontalSpacing(10)
        hl, hw = FIELD_L / 2, FIELD_W / 2
        st = self.node.starts.get(self.node.selected, (-3.0, -3.0, 90.0))
        self.rx, self.ry, self.ryaw = self.spin(-hl - 0.5, hl + 0.5, 0.1, 2, st[0]), self.spin(-hw - 0.5, hw + 0.5, 0.1, 2, st[1]), self.spin(-180, 180, 5, 0, st[2])
        self.gx, self.gy, self.gyaw = self.spin(-hl, hl, 0.1, 2, -1.5), self.spin(-hw, hw, 0.1, 2, 0.0), self.spin(-180, 180, 5, 0, 0)
        self.bx, self.by = self.spin(-hl, hl, 0.1, 2, 0.0), self.spin(-hw, hw, 0.1, 2, 1.0)
        self.qx, self.qy, self.qyaw = self.spin(-hl, 0.0, 0.1, 2, -0.8), self.spin(-hw, hw, 0.1, 2, 0.0), self.spin(-180, 180, 5, 0, 0)

        rows = [('Robot', (self.rx, self.ry, self.ryaw), 'Set', self.on_set_robot),
                ('Goal', (self.gx, self.gy, self.gyaw), 'Go', self.on_go),
                ('Ball', (self.bx, self.by, None), 'Set', self.on_set_ball)]
        for c, h in enumerate(['', 'X (m)', 'Y (m)', '각도 (도)']):
            if h:
                g.addWidget(QLabel(h), 0, c, Qt.AlignCenter)
        rows.append(('Ready 위치', (self.qx, self.qy, self.qyaw), None, None))   # 준비 위치 (LEFT 진영)
        for r, (name, boxes, btn, fn) in enumerate(rows, start=1):
            g.addWidget(self.tag(name), r, 0)
            for c, b in enumerate(boxes, start=1):
                if b is not None:
                    g.addWidget(b, r, c)
            if btn:
                button = QPushButton(btn)
                button.clicked.connect(fn)
                g.addWidget(button, r, 4)

        reset = QPushButton('Reset')          # 목표 취소 + 시작 위치로
        reset.clicked.connect(self.on_reset)
        g.addWidget(reset, 1, 5)
        stop = QPushButton('Stop')            # 목표 취소 (그 자리에 멈춤)
        stop.clicked.connect(self.node.send_cancel)
        g.addWidget(stop, 2, 5)

        front = QHBoxLayout()                 # 공 앞으로 가기 (그 안쪽은 Vision이 맡음)
        front.addWidget(self.tag('공 앞 거리 (m)'))
        self.front_dist = self.spin(0.3, 2.0, 0.05, 2, 0.5)
        front.addWidget(self.front_dist)
        go_front = QPushButton('공 앞으로')
        go_front.setStyleSheet('background: #2e7d32;')
        go_front.clicked.connect(self.on_ball_front)
        front.addWidget(go_front)
        self.avoid = QCheckBox('공 피하기')      # NavCommand.avoid_ball (고른 로봇)
        self.avoid.setChecked(True)
        self.avoid.toggled.connect(lambda on: self.node.send_avoid_ball(on))
        front.addWidget(self.avoid)
        front.addStretch()

        logo = QFrame()
        logo.setObjectName('logo')
        lay = QHBoxLayout(logo)
        pin = QLabel('➤')
        pin.setStyleSheet('color: #e53935; font-size: 30px; border: none;')
        name = QLabel('Navigation')
        name.setStyleSheet('font-size: 28px; border: none;')
        lay.addWidget(pin)
        lay.addWidget(name)
        lay.addStretch()

        box = QVBoxLayout()
        box.addLayout(pick)
        box.addLayout(g)
        box.addLayout(front)
        box.addStretch()
        box.addWidget(logo, 0, Qt.AlignLeft)
        return box

    # ---------- 오른쪽 아래: 튜닝 탭 ----------
    def make_tab(self, params, node_name):
        w = QWidget()
        g = QGridLayout(w)
        boxes = {}
        for k, (name, label, lo, hi, step, dec) in enumerate(params):
            if dec == 0:
                b = QSpinBox()
                b.setRange(int(lo), int(hi))
            else:
                b = QDoubleSpinBox()
                b.setRange(lo, hi)
                b.setSingleStep(step)
                b.setDecimals(dec)
            boxes[name] = b
            g.addWidget(self.tag(label), k % 5, (k // 5) * 2)   # 한 줄에 5개씩
            g.addWidget(b, k % 5, (k // 5) * 2 + 1)
        apply_btn, read_btn = QPushButton('Apply'), QPushButton('Read')
        g.addWidget(apply_btn, 0, 6)
        g.addWidget(read_btn, 1, 6)

        def apply():
            vals = {n: (int(b.value()) if isinstance(b, QSpinBox) else round(float(b.value()), 4))
                    for n, b in boxes.items()}
            self.node.set_params(node_name, vals)

        apply_btn.clicked.connect(apply)
        read_btn.clicked.connect(lambda: self.read(node_name, boxes))
        return w, (node_name, boxes)

    def make_tabs(self):
        tabs = QTabWidget()
        pid, a = self.make_tab(PID_PARAMS, 'navigation_node')
        path, b = self.make_tab(PATH_PARAMS, 'navigation_node')
        self.tab_info = [a, b]
        tabs.addTab(pid, 'PID')
        tabs.addTab(path, 'Path')
        return tabs

    def read(self, node_name, boxes):
        names = list(boxes)

        def fill(vals):
            for n, v in vals.items():
                if v is not None:
                    boxes[n].setValue(int(v) if isinstance(boxes[n], QSpinBox) else float(v))
        self.node.get_params(node_name, names, fill)

    def read_all(self):
        for info in self.tab_info:
            self.read(*info)

    # ---------- 버튼 ----------
    def on_set_robot(self):
        self.node.send_robot(self.rx.value(), self.ry.value(), math.radians(self.ryaw.value()))

    def on_go(self):
        self.node.send_goal(self.gx.value(), self.gy.value(), math.radians(self.gyaw.value()))

    def on_set_ball(self):
        self.node.send_ball(self.bx.value(), self.by.value())

    def on_ball_front(self):
        """고른 로봇을 공 앞으로: 공에서 상대 골대 반대쪽으로 '공 앞 거리' 뒤, 골대 쪽 바라보기"""
        if self.node.ball is None:
            self.log('공 위치를 아직 모름')
            return
        x, y, yaw = ball_front_goal(self.node.ball, (FIELD_L / 2.0, 0.0), self.front_dist.value())
        self.set_goal_inputs(x, y, yaw)
        self.node.send_goal(x, y, yaw)

    def on_all_setting(self):
        """두 로봇 모두 목표 취소 + 세팅 위치에 놓기 (경기 시작 전)"""
        for n in self.node.names:
            st = self.node.starts.get(n, (-3.0, -3.0, 90.0))
            self.node.send_cancel(n)
            self.node.send_robot(st[0], st[1], math.radians(st[2]), n)
        self.set_robot_inputs(*self.node.starts[self.node.selected][:2],
                              math.radians(self.node.starts[self.node.selected][2]))

    def on_ready(self, on):
        """
        Ready 켜기: RIGHT 진영 금지 + 두 로봇을 각자 준비 위치로 보냄 (규정: 스스로 걸어가서 정지)
        둘 다 도착하면 Text에 '준비 완료'. 끄기 = Playing (RIGHT 진영 제한 해제)
        """
        self.ready_btn.setStyleSheet('background: #c62828; border: 3px solid #ffe14d;' if on else '')
        self.ready_btn.setText('Ready 켜짐 (끄면 Playing)' if on else 'Ready')
        self.node.send_left_only(on)
        if not on:
            self.ready_wait = None
            return
        self.save_inputs()
        for n in self.node.names:
            x, y, a = self.saved[n]['ready']
            self.node.send_goal(x, y, math.radians(a), n)
        self.ready_wait = set(self.node.names)
        self.ready_sent = time.monotonic()

    def check_ready(self):
        """Ready 중: 두 로봇이 준비 위치에 도착했는지 (보낸 직후의 예전 상태는 무시)"""
        if not self.ready_wait or time.monotonic() - self.ready_sent < 1.0:
            return
        for n in list(self.ready_wait):
            st = self.node.data[n]['status']
            if st == 'ARRIVED':
                self.ready_wait.discard(n)
            elif st == 'BLOCKED':
                self.log(f'{n} 준비 위치로 못 감 (BLOCKED): 준비 위치가 LEFT 진영 안쪽인지 확인')
                self.ready_wait.discard(n)
        if not self.ready_wait:
            self.ready_wait = None
            self.log('준비 완료: 두 로봇 모두 준비 위치에서 정지')

    def on_reset(self):
        self.node.send_cancel()               # 예전 목표로 다시 출발하지 않게 먼저 취소
        st = self.node.starts.get(self.node.selected, (-3.0, -3.0, 90.0))
        self.set_robot_inputs(st[0], st[1], math.radians(st[2]))
        self.on_set_robot()

    def select(self, n):
        """조작할 로봇 바꾸기: 버튼 색, 입력칸, 튜닝 값을 그 로봇 것으로"""
        # 지금 보이는 로봇의 입력칸 값을 저장하고, 고른 로봇의 저장된 값을 불러옴
        self.save_inputs()
        (rx, ry, ra), (gx, gy, ga) = self.saved[n]['robot'], self.saved[n]['goal']
        qx, qy, qa = self.saved[n]['ready']
        self.rx.setValue(rx), self.ry.setValue(ry), self.ryaw.setValue(ra)
        self.gx.setValue(gx), self.gy.setValue(gy), self.gyaw.setValue(ga)
        self.qx.setValue(qx), self.qy.setValue(qy), self.qyaw.setValue(qa)
        self.shown = n
        self.node.selected = n
        self.avoid.blockSignals(True)
        self.avoid.setChecked(self.node.command[n].avoid_ball)
        self.avoid.blockSignals(False)
        for name, b in self.pick_buttons.items():
            b.setChecked(name == n)
            k = self.node.names.index(name)
            b.setStyleSheet(f'background: {ROBOT_COLORS[k % 4]}; border: 3px solid #ffe14d;'
                            if name == n else '')
        self.read_all()
        self.log(f'조작할 로봇: {n}')

    def save_inputs(self):
        """지금 보이는 로봇의 입력칸 값을 저장"""
        if self.shown is not None:
            self.saved[self.shown] = {
                'robot': (self.rx.value(), self.ry.value(), self.ryaw.value()),
                'goal': (self.gx.value(), self.gy.value(), self.gyaw.value()),
                'ready': (self.qx.value(), self.qy.value(), self.qyaw.value())}

    def set_robot_inputs(self, x, y, yaw):
        self.rx.setValue(x), self.ry.setValue(y), self.ryaw.setValue(math.degrees(yaw))

    def set_goal_inputs(self, x, y, yaw):
        self.gx.setValue(x), self.gy.setValue(y), self.gyaw.setValue(math.degrees(yaw))

    def set_ball_inputs(self, x, y):
        self.bx.setValue(x), self.by.setValue(y)

    # ---------- 반복 ----------
    def refresh(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)    # ROS2 메시지 처리
        for _ in range(20):                            # 쌓인 메시지도 처리
            rclpy.spin_once(self.node, timeout_sec=0.0)
        self.check_ready()
        self.field.update()
        self.robot_view.update()

    def log(self, text):
        self.text.appendPlainText(text)


def main(args=None):
    rclpy.init(args=args)
    node = NavGuiNode()
    app = QApplication(sys.argv)
    win = MainWindow(node)
    win.show()
    signal.signal(signal.SIGINT, lambda *_: app.quit())   # Ctrl+C 로 종료
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
