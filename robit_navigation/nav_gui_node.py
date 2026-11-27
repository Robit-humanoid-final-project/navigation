"""
내비게이션 UI (실제 로봇용, Localization UI와 같은 배치)

  왼쪽 위   필드: 로봇, 꼬깔, 공, A-star 경로, 위험 범위, 당근, 목표 (Localization 맵 그림 위에)
  오른쪽 위  로봇 기준 화면: 로봇이 바라보는 방향이 위쪽, 상태, 보행 명령(master2ik) 값
  왼쪽 아래  Ready / Playing, 목표 입력 + Go, Stop, 준비 위치, 공 앞으로, 꼬깔 기억 지우기
  오른쪽 아래 튜닝 탭 (PID, 경로) + Text (기록)

필드 위 마우스:
  왼쪽 클릭          = 목표 (도착 후 가던 방향 그대로 바라봄)
  왼쪽 클릭 후 끌기   = 목표 + 도착 후 바라볼 방향 (끈 방향)

받는 토픽:  world_model (Localization: 로봇·공·꼬깔), nav_path, nav_feedback, nav_debug (내비게이션 그림),
            master2ik (보행으로 실제 나가는 값, 화면 표시용)
보내는 토픽: nav_command (Taskplanner 대신: 목표, Stop, Ready. 공 피하기는 항상 켬),
            nav_clear_cones (꼬깔 기억 지우기),
            master2ik flag 0 (Stop 누를 때·UI 닫을 때만, 보행 멈춤)
            -> 내비게이션은 nav_enable = false면 보행 명령을 안 보내니까 (팀 약속) Taskplanner 대신 UI가 멈춤

주의: Taskplanner와 같이 돌릴 때는 UI 버튼을 누르지 말기 (UI도 nav_command를 보내서 명령이 섞임)
공 앞으로 버튼: 공에서 상대 골대(오른쪽, x = +4.5) 반대쪽으로 "공 앞 거리"만큼 뒤, 골대 쪽 바라보기
튜닝 값은 ROS2 파라미터로 바꿈 (ros2 param set 과 같음)
"""
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import Empty
from nav_msgs.msg import Path
from humanoid_interfaces.msg import Master2IkMsg, NavCommand, NavFeedback, WorldModel
from visualization_msgs.msg import Marker, MarkerArray

from python_qt_binding.QtCore import QPointF, QRectF, Qt, QTimer
from python_qt_binding.QtGui import QColor, QFont, QPainter, QPen, QPixmap, QPolygonF
from python_qt_binding.QtWidgets import (
    QApplication, QDoubleSpinBox, QFrame, QGridLayout, QHBoxLayout, QLabel,
    QMainWindow, QPlainTextEdit, QPushButton, QSpinBox, QTabWidget, QVBoxLayout, QWidget)

from robit_navigation.ball_approach import ball_front_goal
from robit_navigation.nav_core import field_to_robot

# 필드 크기 (m): Localization 맵(stadium_info.hpp)과 같은 값
FIELD_L, FIELD_W = 9.0, 6.0      # A, B: 필드 길이, 폭
GOAL_DEPTH, GOAL_WIDTH = 0.5, 1.8          # C, D: 골대 깊이, 폭
GOAL_AREA_L, GOAL_AREA_W = 1.0, 3.0        # E, F: 골 에어리어
PENALTY_AREA_L, PENALTY_AREA_W = 2.0, 4.0  # G, H: 페널티 에어리어
PENALTY_MARK = 1.5                         # I: 골라인에서 페널티 마크까지
CENTER_CIRCLE = 1.5                        # J: 센터서클 지름
BORDER = 1.0                               # K: 필드 선 바깥 여백
# 필드 그림: Localization 맵 이미지 (1100 x 800 px = 11 x 8m, 1m = 100px, 가운데 = 원점)
FIELD_MAP = 'field_map.png'
CONE_DRAW_R = 0.1                # 지금 보이는 꼬깔 그리는 반지름 (m)
ROBOT_COLOR = '#1e63d6'

STATUS_COLOR = {'IDLE': '#ffffff', 'MOVING': '#ffe14d', 'ARRIVED': '#5dff7a', 'WAITING': '#4dd0ff',
                'BLOCKED': '#ff5a5a', 'NO_POSE': '#ff9a3c'}

# 튜닝 탭: (파라미터 이름, 화면 이름, 최소, 최대, 한 칸, 소수 자리)
PID_PARAMS = [
    ('kp_move', 'Kp 이동', 0, 5, 0.1, 2), ('ki_move', 'Ki 이동', 0, 5, 0.05, 2),
    ('kd_move', 'Kd 이동', 0, 5, 0.05, 2), ('kp_turn', 'Kp 회전', 0, 5, 0.1, 2),
    ('ki_turn', 'Ki 회전', 0, 5, 0.05, 2), ('kd_turn', 'Kd 회전', 0, 5, 0.05, 2),
    ('max_move', '최대 앞걸음 m/s', 0, 1, 0.05, 2), ('max_turn', '최대 회전 rad/s', 0, 3, 0.1, 2),
    ('turn_in_place_deg', '제자리 회전 각도', 0, 180, 5, 1), ('walk_again_deg', '다시 걷기 각도', 0, 180, 5, 1),
    ('slow_down_deg', '안 줄이는 각도', 0, 90, 1, 1), ('cmd_smoothing_tau', '부드럽게 s', 0, 2, 0.05, 2),
    ('near_dist', '옆걸음 거리 m', 0, 2, 0.05, 2), ('max_side', '옆걸음 최대(멀 때)', 0, 1, 0.01, 2),
    ('max_side_near', '옆걸음 최대(가까울 때)', 0, 1, 0.01, 2), ('max_back', '뒷걸음 최대', 0, 1, 0.01, 2),
]
PATH_PARAMS = [
    ('robot_radius', '로봇 반지름 m', 0, 1, 0.01, 2), ('margin', '여유 m', 0, 1, 0.01, 2),
    ('cone_radius', '꼬깔 반지름 m', 0, 1, 0.01, 2), ('ball_radius', '공 반지름 m', 0, 1, 0.01, 3),
    ('tol_dist', '허용 거리 m', 0, 1, 0.01, 2),
    ('tol_yaw_deg', '허용 각도 도', 0, 90, 1, 1), ('carrot_points', '당근 (점 개수)', 1, 30, 1, 0),
    ('boundary_margin', '선 안쪽 여유 m', 0, 1, 0.05, 2), ('center_margin', '가운데선 여유 m', 0, 1, 0.05, 2),
    ('arrive_latch', '도착 래치 (배)', 0, 5, 0.5, 1),
    ('keep_side_cost', '같은 쪽 유지 (배)', 0, 3, 0.1, 1),
]

READY_ON = 'background: #c62828; border: 3px solid #ffe14d;'    # Ready 켜짐 (빨강)
PLAY_ON = 'background: #2e7d32; border: 3px solid #ffe14d;'     # Playing 켜짐 (초록)
STYLE = """
QWidget { background: #55575b; color: #f0f0f0; font-size: 13px; }
QPushButton { background: #2e3337; border: none; border-radius: 8px; padding: 8px 14px; color: #ffffff; }
QPushButton:hover { background: #3c4247; }
QPushButton:pressed { background: #23272a; }
QPushButton:disabled { background: #46494d; color: #8a8d91; }
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
    """ROS2 부분: 토픽을 받아서 저장해 두고, 버튼을 누르면 내비게이션에 토픽·파라미터를 보낸다"""

    def __init__(self):
        super().__init__('nav_gui_node')
        # 로봇 이름 (화면 표시용)과 준비 위치 (Ready 때 스스로 걸어갈 곳, LEFT 진영이면 자유), launch에서 줌
        self.declare_parameter('robot', 'robot1')
        self.declare_parameter('ready_x', -0.8)
        self.declare_parameter('ready_y', -1.2)
        self.declare_parameter('ready_yaw_deg', 0.0)
        p = lambda n: self.get_parameter(n).value
        self.name = p('robot')
        self.ready = (float(p('ready_x')), float(p('ready_y')), float(p('ready_yaw_deg')))

        self.robot = None          # 로봇 위치 (x, y, yaw), world_model
        self.ball = None           # 공 위치 (x, y), world_model (안 보이면 마지막 위치)
        self.seen_cones = []       # 지금 보이는 꼬깔 [(x, y)], world_model
        self.path = []             # A-star 경로
        self.status = '-'
        self.eta = -1.0
        self.walk = (0.0, 0.0, 0.0, 0.0)   # 보행으로 나가는 값 (x_length, y_length, yaw, flag)
        self.markers = {}          # (ns, id) -> Marker (내비게이션 그림)
        self.log = None            # 화면 기록 함수 (창에서 연결)
        # Taskplanner 대신 보내는 NavCommand 내용 (바뀔 때마다 다시 보냄)
        self.command = NavCommand(nav_enable=False, avoid_ball=True)

        self.create_subscription(WorldModel, 'world_model', self.on_world, 10)
        self.create_subscription(Path, 'nav_path', self.on_path, 10)
        self.create_subscription(NavFeedback, 'nav_feedback', self.on_feedback, 10)
        self.create_subscription(MarkerArray, 'nav_debug', self.on_markers, 10)
        self.create_subscription(Master2IkMsg, 'master2ik', self.on_walk, 10)
        self.nav_pub = self.create_publisher(NavCommand, 'nav_command', 10)
        self.clear_pub = self.create_publisher(Empty, 'nav_clear_cones', 10)
        self.walk_pub = self.create_publisher(Master2IkMsg, 'master2ik', 10)   # Stop·닫기 때 보행 멈춤만
        self.params = AsyncParameterClient(self, 'navigation_node')

    # ---------- 받기 ----------
    def on_world(self, m):
        """Localization 의 world_model (필드 기준)"""
        self.robot = (m.robot_pose.x, m.robot_pose.y, m.robot_pose.theta)
        if m.ball_detected:
            self.ball = (m.ball_pose.x, m.ball_pose.y)
        self.seen_cones = [(o.x, o.y) for o in m.obstacles]

    def on_path(self, m):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    def on_feedback(self, m):
        """NavFeedback: 상태 숫자 -> 글자, 도착 예상 시간"""
        text = {0: 'IDLE', 1: 'MOVING', 2: 'ARRIVED', 3: 'BLOCKED'}.get(m.status, str(m.status))
        if m.status == 3 and self.robot is None:
            text = 'NO_POSE'    # 3 = 길 없음 또는 위치 모름 -> 위치를 한 번도 못 받았으면 위치 모름
        if text != self.status and self.log:
            self.log(f'상태: {text}')
        self.status = text
        self.eta = m.eta_seconds

    def on_walk(self, m):
        self.walk = (m.x_length, m.y_length, m.yaw, m.flag)

    def on_markers(self, arr):
        for mk in arr.markers:
            if mk.action == Marker.DELETEALL:
                self.markers = {}
            elif mk.action == Marker.DELETE:
                self.markers.pop((mk.ns, mk.id), None)
            else:
                self.markers[(mk.ns, mk.id)] = mk

    # ---------- 보내기 ----------
    def publish_command(self):
        self.nav_pub.publish(self.command)

    def send_goal(self, x, y, yaw):
        """목표 보내기: nav_enable = true + 목표 (허용 오차 0 = 내비게이션 기본값)"""
        c = self.command
        c.nav_enable, c.x, c.y, c.theta = True, float(x), float(y), float(yaw)
        self.publish_command()
        self.log(f'목표 보냄: ({x:.2f}, {y:.2f}), {math.degrees(yaw):.0f}도')

    def send_left_only(self, on):
        """Ready: RIGHT 진영 금지"""
        self.command.forbid_right_half = bool(on)
        self.publish_command()
        self.log('Ready: RIGHT 진영 금지' if on else 'Playing: RIGHT 진영 금지 해제')

    def send_cancel(self):
        """멈추기: nav_enable = false + 보행 멈춤"""
        self.command.nav_enable = False
        self.publish_command()
        self.stop_walk()
        self.log('멈춤 (nav_enable = false, 보행 flag 0)')

    def stop_walk(self):
        """보행 멈춤 (master2ik flag 0): 내비게이션은 nav_enable = false면 안 보내니까 Taskplanner 대신 UI가"""
        self.walk_pub.publish(Master2IkMsg(x_length=0.0, y_length=0.0, yaw=0.0, flag=0.0))

    def clear_cones(self):
        """꼬깔 기억 지우기"""
        self.clear_pub.publish(Empty())
        self.log('꼬깔 기억 지움 (지금 보이는 것부터 다시 기억)')

    def stop_all(self):
        """UI를 닫을 때: nav_enable = false + 보행 멈춤 (로봇이 마지막 명령으로 계속 걷지 않게)"""
        self.command.nav_enable = False
        self.publish_command()
        self.stop_walk()

    # ---------- 파라미터 (내비게이션 노드) ----------
    def set_params(self, values):
        """values: {이름: 값}. ros2 param set 과 같은 일"""
        if not self.params.services_are_ready():
            self.log('navigation_node 이(가) 실행 중이 아님')
            return
        names = list(values)
        future = self.params.set_parameters([Parameter(n, value=values[n]) for n in names])

        def done(f):
            res = f.result()
            if res is None:
                self.log('파라미터 변경 실패 (응답 없음)')
                return
            bad = [f'{n}: {r.reason}' for n, r in zip(names, res.results) if not r.successful]
            self.log('적용 완료: ' + ', '.join(f'{n}={values[n]}' for n in names) if not bad
                     else '거부됨 -> ' + '; '.join(bad))
        future.add_done_callback(done)

    def get_params(self, names, on_values):
        """지금 값을 읽어서 on_values({이름: 값}) 호출"""
        if not self.params.services_are_ready():
            return
        future = self.params.get_parameters(names)

        def done(f):
            res = f.result()
            if res is not None:
                on_values({n: parameter_value_to_python(v) for n, v in zip(names, res.values)})
        future.add_done_callback(done)


# ======================================================================
def load_field_map():
    """설치된 패키지의 Localization 맵 그림 (없으면 None -> 선으로 그림)"""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('robit_navigation'), 'resource', FIELD_MAP)
    except Exception:
        return None
    img = QPixmap(path)
    return None if img.isNull() else img


class FieldView(QWidget):
    """필드 그림 (필드 좌표: 오른쪽 +x, 위쪽 +y)"""

    def __init__(self, node, window):
        super().__init__()
        self.node, self.win = node, window
        self.setMinimumSize(550, 400)
        self.drag = None   # (시작점, 지금 점)
        self.map_img = load_field_map()

    # 필드 m <-> 화면 픽셀
    def scale(self):
        return min(self.width() / (FIELD_L + 2 * BORDER), self.height() / (FIELD_W + 2 * BORDER))

    def to_px(self, x, y):
        s = self.scale()
        return QPointF(self.width() / 2 + x * s, self.height() / 2 - y * s)

    def to_field(self, p):
        s = self.scale()
        return (p.x() - self.width() / 2) / s, (self.height() / 2 - p.y()) / s

    # ---------- 마우스: 왼쪽 클릭·끌기 = 목표 ----------
    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            pt = self.to_field(e.pos())
            self.drag = (pt, pt)

    def mouseMoveEvent(self, e):
        if self.drag:
            self.drag = (self.drag[0], self.to_field(e.pos()))

    def mouseReleaseEvent(self, e):
        if not self.drag or e.button() != Qt.LeftButton:
            return
        (x0, y0), (x1, y1) = self.drag
        self.drag = None
        if math.hypot(x1 - x0, y1 - y0) > 0.15:
            yaw = math.atan2(y1 - y0, x1 - x0)          # 끈 방향
        elif self.node.robot is not None:
            rx, ry, _ = self.node.robot                 # 클릭만: 로봇 -> 목표 방향 (도착해서 안 돌아도 됨)
            yaw = math.atan2(y0 - ry, x0 - rx)
        else:
            yaw = 0.0
        self.win.set_goal_inputs(x0, y0, yaw)
        self.node.send_goal(x0, y0, yaw)

    # ---------- 그리기 ----------
    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        s = self.scale()
        p.fillRect(self.rect(), QColor('#2c323d'))
        if self.map_img is not None:
            # Localization 맵 그림을 필드 좌표에 맞춰 그림 (그림 전체 = 필드 + 바깥 여백 1m)
            top_left = self.to_px(-(FIELD_L / 2 + BORDER), FIELD_W / 2 + BORDER)
            p.drawPixmap(QRectF(top_left.x(), top_left.y(), (FIELD_L + 2 * BORDER) * s,
                                (FIELD_W + 2 * BORDER) * s), self.map_img, QRectF(self.map_img.rect()))
        else:
            self.draw_field(p, s)                                # 그림이 없으면 선으로 그림
        self.draw_markers(p, s)

        # 지금 보이는 꼬깔(주황 점)과 공(흰색) = world_model 값
        p.setPen(QPen(QColor(0, 0, 0, 120), 1))
        p.setBrush(QColor(255, 120, 0))
        for cx, cy in self.node.seen_cones:
            p.drawEllipse(self.to_px(cx, cy), max(CONE_DRAW_R * s, 3.0), max(CONE_DRAW_R * s, 3.0))
        if self.node.ball is not None:
            p.setBrush(QColor(255, 255, 255))
            p.drawEllipse(self.to_px(*self.node.ball), max(0.07 * s, 3.0), max(0.07 * s, 3.0))

        # A-star 경로, 로봇 (실제 크기 25 x 17cm 네모, 앞면 25cm)
        if len(self.node.path) >= 2:
            p.setPen(QPen(QColor(40, 110, 255), max(2.0, 0.03 * s)))
            p.setBrush(Qt.NoBrush)
            p.drawPolyline(QPolygonF([self.to_px(x, y) for x, y in self.node.path]))
        if self.node.robot is not None:
            self.draw_robot(p, *self.node.robot, s)

        # 끄는 중인 목표 화살표 미리보기
        if self.drag:
            (x0, y0), (x1, y1) = self.drag
            col = QColor(230, 30, 30)
            p.setPen(QPen(col, 3))
            p.drawLine(self.to_px(x0, y0), self.to_px(x1, y1))
            p.setBrush(col)
            p.drawEllipse(self.to_px(x0, y0), 5, 5)

        # 도움말
        p.setPen(QColor(255, 255, 255, 200))
        p.setFont(QFont('Sans', 9))
        p.drawText(8, self.height() - 8, '클릭: 목표   끌기: 목표 + 바라볼 방향')
        # 지금 게임 상태 (왼쪽 위)
        ready = self.win.ready_btn.isChecked()
        p.setPen(QColor('#ff5a5a') if ready else QColor('#ffffff'))
        p.setFont(QFont('Sans', 14, QFont.Bold))
        p.drawText(10, 26, 'READY (RIGHT 진영 금지)' if ready else 'PLAYING')
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
        r = CENTER_CIRCLE / 2 * s                                # 센터서클
        p.drawEllipse(self.to_px(0, 0), r, r)

        def box(depth, width, outside=False):
            """골라인에 붙은 네모 (depth: 골라인에서 안쪽(또는 바깥쪽) 길이, width: 폭)"""
            for side in (-1, 1):
                gx = side * hl
                ex = gx + side * depth if outside else gx - side * depth
                rect(min(gx, ex), -width / 2, max(gx, ex), width / 2)

        box(GOAL_AREA_L, GOAL_AREA_W)                            # 골 에어리어
        box(PENALTY_AREA_L, PENALTY_AREA_W)                      # 페널티 에어리어
        box(GOAL_DEPTH, GOAL_WIDTH, outside=True)                # 골대 (필드 바깥쪽)
        for side in (-1, 1):
            p.setBrush(QColor('#ffffff'))
            p.drawEllipse(self.to_px(side * (hl - PENALTY_MARK), 0), 0.06 * s, 0.06 * s)   # 페널티 마크
            p.setBrush(Qt.NoBrush)

    def draw_markers(self, p, s):
        """내비게이션 그림 (nav_debug): 꼬깔·공 위험 범위, 다닐 수 있는 범위, 당근, 목표, 상태 글자"""
        def order(mk):
            # 연한 원판(위험 범위) -> 선 -> 물체 -> 화살표 -> 글자 순서로 그림
            rank = {Marker.CYLINDER: 2, Marker.LINE_STRIP: 1, Marker.LINE_LIST: 1,
                    Marker.SPHERE: 3, Marker.ARROW: 4, Marker.TEXT_VIEW_FACING: 5}.get(mk.type, 9)
            return (0 if mk.color.a < 0.5 and mk.type == Marker.CYLINDER else 1, rank)

        for (ns, _), mk in sorted(self.node.markers.items(), key=lambda kv: order(kv[1])):
            pos = mk.pose.position
            c = self.to_px(pos.x, pos.y)
            if mk.type in (Marker.CYLINDER, Marker.SPHERE):
                r = mk.scale.x / 2 * s
                p.setPen(Qt.NoPen if mk.color.a < 0.5 else QPen(QColor(0, 0, 0, 120), 1))
                p.setBrush(qcolor(mk.color, 1.6 if mk.color.a < 0.5 else 1.0))
                p.drawEllipse(c, max(r, 2.0), max(r, 2.0))
            elif mk.type in (Marker.LINE_STRIP, Marker.LINE_LIST):
                p.setPen(QPen(qcolor(mk.color), max(1.5, mk.scale.x * s)))
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

    def draw_robot(self, p, x, y, yaw, s):
        c, sn = math.cos(yaw), math.sin(yaw)

        def corner(f, l):     # 로봇 기준 (앞, 왼쪽) -> 화면
            return self.to_px(x + c * f - sn * l, y + sn * f + c * l)

        body = QPolygonF([corner(0.085, 0.125), corner(0.085, -0.125),
                          corner(-0.085, -0.125), corner(-0.085, 0.125)])
        p.setPen(QPen(QColor('#ffe14d'), 3))
        p.setBrush(QColor(ROBOT_COLOR))
        p.drawPolygon(body)
        p.setPen(QPen(QColor('#ffffff'), 3))
        p.drawLine(self.to_px(x, y), corner(0.3, 0.0))    # 바라보는 방향

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
    """로봇 기준 화면: 로봇이 가운데 아래, 바라보는 방향이 위쪽"""

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
            # 내비게이션이 아는 위험 범위, 당근, 목표 (필드 -> 로봇 기준)
            for (ns, _), mk in self.node.markers.items():
                if ns not in ('inflated', 'ball_zone', 'teammate', 'carrot', 'goal'):
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

        # 보행 명령 화살표 (master2ik x, y 값, 25 = 1m 길이)
        wx, wy, wyaw, flag = self.node.walk
        if flag > 0.5 and abs(wx) + abs(wy) > 1e-3:
            p.setPen(QPen(QColor(255, 220, 0), 4))
            p.drawLine(px(0, 0), px(wx / 25.0, wy / 25.0))

        # 글자: 상태, 보행 명령, 위치, 도착 예상
        st = self.node.status
        p.setPen(QColor(STATUS_COLOR.get(st, '#ffffff')))
        p.setFont(QFont('Sans', 18, QFont.Bold))
        p.drawText(14, 32, st)
        p.setPen(QColor('#dddddd'))
        p.setFont(QFont('Sans', 10))
        p.drawText(14, 56, f'보행  x {wx:+.1f}   y {wy:+.1f}   yaw {wyaw:+.1f}   flag {flag:.0f}')
        if robot is not None:
            p.drawText(14, 76, f'로봇 ({robot[0]:+.2f}, {robot[1]:+.2f})  {math.degrees(robot[2]):+.0f}도')
        eta = self.node.eta
        p.drawText(14, 96, f'도착 예상 {eta:.1f} 초' if eta >= 0 else '도착 예상 -')
        p.setPen(QColor('#888888'))
        p.drawText(w - 170, 20, f'{self.node.name} 기준 화면')
        p.end()


# ======================================================================
class MainWindow(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        node.log = self.log
        self.setWindowTitle(f'Navigation ({node.name})')
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
        self.ready_wait = False         # Ready 중 아직 준비 위치에 도착 안 함
        self.ready_sent = 0.0
        self.log(f'Navigation UI 시작 ({node.name})')
        QTimer.singleShot(1500, self.read_all)   # 노드가 켜지면 지금 값 읽어오기

    # ---------- 왼쪽 아래: 버튼, 입력 ----------
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
        # 게임 상태: Ready / Playing
        state = QHBoxLayout()
        state.addWidget(self.tag(self.node.name))
        self.ready_btn = QPushButton('Ready')        # 준비 위치로 + RIGHT 진영 금지
        self.ready_btn.setCheckable(True)
        self.ready_btn.toggled.connect(self.on_ready)
        state.addWidget(self.ready_btn)
        self.play_btn = QPushButton('Playing')      # 바로 경기 상태 (RIGHT 진영 제한 없음), 처음 켜면 이 상태
        self.play_btn.setCheckable(True)
        self.play_btn.setChecked(True)
        self.play_btn.setStyleSheet(PLAY_ON)
        self.play_btn.clicked.connect(lambda: self.ready_btn.setChecked(False) if self.ready_btn.isChecked()
                                      else self.play_btn.setChecked(True))
        state.addWidget(self.play_btn)
        state.addStretch()

        # 목표, 준비 위치
        g = QGridLayout()
        g.setHorizontalSpacing(10)
        hl, hw = FIELD_L / 2, FIELD_W / 2
        rx, ry, ryaw = self.node.ready
        self.gx, self.gy, self.gyaw = self.spin(-hl, hl, 0.1, 2, 0.0), self.spin(-hw, hw, 0.1, 2, 0.0), self.spin(-180, 180, 5, 0, 0)
        self.qx, self.qy, self.qyaw = self.spin(-hl, 0.0, 0.1, 2, rx), self.spin(-hw, hw, 0.1, 2, ry), self.spin(-180, 180, 5, 0, ryaw)
        for c, h in enumerate(['', 'X (m)', 'Y (m)', '각도 (도)']):
            if h:
                g.addWidget(QLabel(h), 0, c, Qt.AlignCenter)
        for r, (name, boxes) in enumerate([('Goal', (self.gx, self.gy, self.gyaw)),
                                           ('Ready 위치', (self.qx, self.qy, self.qyaw))], start=1):
            g.addWidget(self.tag(name), r, 0)
            for c, b in enumerate(boxes, start=1):
                g.addWidget(b, r, c)
        go = QPushButton('Go')
        go.clicked.connect(self.on_go)
        g.addWidget(go, 1, 4)
        stop = QPushButton('Stop')            # 목표 취소 + 보행 멈춤
        stop.clicked.connect(self.node.send_cancel)
        g.addWidget(stop, 1, 5)

        front = QHBoxLayout()                 # 공 앞으로 가기 (그 안쪽 킥 정렬은 Taskplanner가 맡음)
        front.addWidget(self.tag('공 앞 거리 (m)'))
        self.front_dist = self.spin(0.3, 2.0, 0.05, 2, 0.5)
        front.addWidget(self.front_dist)
        go_front = QPushButton('공 앞으로')
        go_front.setStyleSheet('background: #2e7d32;')
        go_front.clicked.connect(self.on_ball_front)
        front.addWidget(go_front)
        clear = QPushButton('꼬깔 기억 지우기')   # 처음부터 다시 (내 위치가 크게 틀렸다가 다시 잡혔을 때)
        clear.clicked.connect(self.node.clear_cones)
        front.addWidget(clear)
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
        box.addLayout(state)
        box.addLayout(g)
        box.addLayout(front)
        box.addStretch()
        box.addWidget(logo, 0, Qt.AlignLeft)
        return box

    # ---------- 오른쪽 아래: 튜닝 탭 ----------
    def make_tab(self, params):
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
            self.node.set_params(vals)

        apply_btn.clicked.connect(apply)
        read_btn.clicked.connect(lambda: self.read(boxes))
        return w, boxes

    def make_tabs(self):
        tabs = QTabWidget()
        pid, a = self.make_tab(PID_PARAMS)
        path, b = self.make_tab(PATH_PARAMS)
        self.tab_boxes = [a, b]
        tabs.addTab(pid, 'PID')
        tabs.addTab(path, 'Path')
        return tabs

    def read(self, boxes):
        def fill(vals):
            for n, v in vals.items():
                if v is not None:
                    boxes[n].setValue(int(v) if isinstance(boxes[n], QSpinBox) else float(v))
        self.node.get_params(list(boxes), fill)

    def read_all(self):
        for boxes in self.tab_boxes:
            self.read(boxes)

    # ---------- 버튼 ----------
    def on_go(self):
        self.node.send_goal(self.gx.value(), self.gy.value(), math.radians(self.gyaw.value()))

    def on_ball_front(self):
        """공 앞으로: 공에서 상대 골대 반대쪽으로 '공 앞 거리' 뒤, 골대 쪽 바라보기"""
        if self.node.ball is None:
            self.log('공 위치를 아직 모름')
            return
        x, y, yaw = ball_front_goal(self.node.ball, (FIELD_L / 2.0, 0.0), self.front_dist.value())
        self.set_goal_inputs(x, y, yaw)
        self.node.send_goal(x, y, yaw)

    def on_ready(self, on):
        """
        Ready 켜기: RIGHT 진영 금지 + 준비 위치로 보냄 (규정: 스스로 걸어가서 정지)
        도착하면 Text에 '준비 완료'. Playing = RIGHT 진영 제한 해제
        """
        self.ready_btn.setStyleSheet(READY_ON if on else '')
        self.play_btn.setChecked(not on)
        self.play_btn.setStyleSheet('' if on else PLAY_ON)
        self.node.send_left_only(on)
        if not on:
            self.ready_wait = False
            return
        self.node.send_goal(self.qx.value(), self.qy.value(), math.radians(self.qyaw.value()))
        self.ready_wait = True
        self.ready_sent = time.monotonic()

    def check_ready(self):
        """Ready 중: 준비 위치에 도착했는지 (보낸 직후의 예전 상태는 무시)"""
        if not self.ready_wait or time.monotonic() - self.ready_sent < 1.0:
            return
        st = self.node.status
        if st == 'ARRIVED':
            self.ready_wait = False
            self.log('준비 완료: 준비 위치에서 정지')
        elif st == 'NO_POSE':
            self.ready_wait = False
            self.log('준비 위치로 못 감 (NO_POSE): 위치(world_model)가 안 옴, Localization 확인')
        elif st == 'BLOCKED':
            self.ready_wait = False
            self.log('준비 위치로 못 감 (BLOCKED): 준비 위치가 LEFT 진영 안쪽인지 확인')

    def set_goal_inputs(self, x, y, yaw):
        self.gx.setValue(x), self.gy.setValue(y), self.gyaw.setValue(math.degrees(yaw))

    # ---------- 반복 ----------
    def refresh(self):
        for _ in range(20):                            # ROS2 메시지 처리 (쌓인 것까지)
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
        node.stop_all()          # 닫을 때 로봇 멈추기
        time.sleep(0.2)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
