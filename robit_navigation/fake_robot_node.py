"""
가짜 로봇 노드 (연습용, 내 담당 아님)
- 실제로는 Walking(걷기) + Localization(위치 추정) + Vision(꼬깔 찾기)이 하는 일을 흉내만 낸다.
- /nav_cmd 를 받아서 그만큼 움직였다고 치고, /robot_pose 로 새 위치를 보낸다.
- 가짜 Vision: 카메라 시야(앞쪽 부채꼴) 안의 꼬깔만, 로봇 기준(앞, 왼쪽)으로, 약간의 오차를 섞어서 보낸다.
- 꼬깔에 닿으면 경고를 띄운다.
- 공: 로봇이 닿으면 공이 밀려나고 경고를 띄운다. RViz2 "Publish Point" 버튼으로 공을 옮길 수 있다.
- RViz2 "2D Pose Estimate" 버튼으로 찍은 곳으로 로봇을 옮긴다 (Reset 역할).
- slip 파라미터로 걸을 때 뒤로 미끄러지는 외란을 흉내 낼 수 있다 (PID 실험용).
- RViz2 화면에 필드, 로봇, 지나온 길, 꼬깔, 카메라 시야를 그린다.

받는 토픽:  /nav_cmd (Twist), /initialpose (PoseWithCovarianceStamped, RViz2 버튼),
            /clicked_point (PointStamped, RViz2 "Publish Point" 버튼 -> 공 옮기기)
보내는 토픽: /robot_pose (Pose2D), /vision_obstacles (PoseArray, 로봇 기준),
            /ball_pose (PointStamped, 필드 기준), /nav_markers (MarkerArray)
로봇 2대일 때: 공은 has_ball = True 인 가짜 로봇 하나만 가지고 있고, 두 로봇 모두 공을 밀 수 있다.
            /teammate_pose (Pose2D) 로 다른 로봇 위치를 받아서 부딪히면 경고한다.
"""
import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import (Point, PointStamped, Pose, Pose2D, PoseArray,
                               PoseWithCovarianceStamped, Twist)
from visualization_msgs.msg import Marker, MarkerArray

FRAME = 'map'               # 필드 좌표계 이름 (RViz2의 Fixed Frame과 같아야 함)
ROBOT_FRAME = 'base_footprint'   # 로봇 기준 좌표계 이름 (Vision 토픽 문서와 같게)


class FakeRobotNode(Node):
    def __init__(self):
        super().__init__('fake_robot_node')

        self.declare_parameter('start_x', -3.0)
        self.declare_parameter('start_y', -3.0)
        self.declare_parameter('start_yaw_deg', 90.0)
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('field_length', 9.0)    # 필드 크기는 예시 값
        self.declare_parameter('field_width', 6.0)
        # 가짜 꼬깔 위치 (필드 기준, m). cones_x[k], cones_y[k]가 한 꼬깔
        self.declare_parameter('cones_x', [-1.0, 1.0, 2.2])
        self.declare_parameter('cones_y', [-1.2, 0.0, -0.5])
        self.declare_parameter('cone_radius', 0.1)
        self.declare_parameter('robot_radius', 0.15)
        # 가짜 공 (필드 기준, m)
        self.declare_parameter('ball_x', 0.0)
        self.declare_parameter('ball_y', 1.0)
        self.declare_parameter('ball_radius', 0.07)
        self.declare_parameter('has_ball', True)   # 로봇 2대일 때 공은 한 쪽만 가짐
        # 가짜 Vision 카메라
        self.declare_parameter('view_angle_deg', 60.0)   # 카메라 시야 전체 각도
        self.declare_parameter('view_range', 3.0)        # 꼬깔을 볼 수 있는 거리 (m)
        self.declare_parameter('vision_noise', 0.03)     # 위치 오차 (m)
        self.declare_parameter('vision_rate', 10.0)      # 1초에 몇 번 보낼지
        # 미끄러짐(외란): 걷는 동안 로봇이 뒤로 밀리는 속도 (m/s). 0이면 미끄러짐 없음
        # 실행 중에도 변경 가능: ros2 param set /fake_robot_node slip 0.1
        self.declare_parameter('slip', 0.0)

        p = lambda name: self.get_parameter(name).value
        self.x, self.y = p('start_x'), p('start_y')
        self.yaw = math.radians(p('start_yaw_deg'))
        self.dt = 1.0 / p('rate')
        self.field_l, self.field_w = p('field_length'), p('field_width')
        self.cones = list(zip(p('cones_x'), p('cones_y')))
        self.cone_r, self.robot_r = p('cone_radius'), p('robot_radius')
        self.view_half = math.radians(p('view_angle_deg')) / 2.0
        self.view_range = p('view_range')
        self.ball = [p('ball_x'), p('ball_y')]
        self.ball_r = p('ball_radius')
        self.has_ball = p('has_ball')
        self.teammate = None       # 다른 로봇 위치 (x, y)
        self.bumping = False
        self.outside = False
        self.pushing = False

        self.cmd = Twist()
        self.trail = []
        self.touching = False

        self.create_subscription(Twist, 'nav_cmd', self.on_cmd, 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_initialpose, 10)
        self.create_subscription(Pose2D, 'teammate_pose', self.on_teammate, 10)
        if self.has_ball:
            self.create_subscription(PointStamped, 'clicked_point', self.on_clicked_point, 10)
            self.ball_pub = self.create_publisher(PointStamped, 'ball_pose', 10)
        self.pose_pub = self.create_publisher(Pose2D, 'robot_pose', 10)
        self.vision_pub = self.create_publisher(PoseArray, 'vision_obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'nav_markers', 10)
        self.create_timer(self.dt, self.loop)
        self.create_timer(1.0 / p('vision_rate'), self.publish_vision)

        self.get_logger().info(
            f'가짜 로봇 시작: ({self.x:.1f}, {self.y:.1f}), {p("start_yaw_deg"):.0f}도')

    def on_cmd(self, msg):
        self.cmd = msg

    def on_initialpose(self, msg):
        """RViz2 "2D Pose Estimate": 찍은 위치·방향으로 로봇을 옮기고 지나온 길을 지운다"""
        q = msg.pose.pose.orientation
        self.x, self.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.cmd = Twist()
        self.trail = []
        self.get_logger().info(
            f'로봇 위치 옮김: ({self.x:.2f}, {self.y:.2f}), {math.degrees(self.yaw):.0f}도')

    def on_teammate(self, msg):
        self.teammate = (msg.x, msg.y)

    def on_clicked_point(self, msg):
        """RViz2 "Publish Point": 찍은 곳으로 공을 옮긴다"""
        self.ball = [msg.point.x, msg.point.y]
        self.get_logger().info(f'공 위치 옮김: ({self.ball[0]:.2f}, {self.ball[1]:.2f})')

    def loop(self):
        # 걷는 중이면 미끄러짐만큼 앞으로 덜 감 (외란)
        walking = abs(self.cmd.linear.x) + abs(self.cmd.linear.y) > 1e-6
        vx = self.cmd.linear.x - (self.get_parameter('slip').value if walking else 0.0)

        # 로봇 기준 속도 명령을 필드 기준 이동으로 바꿔서 위치를 더한다 (좌표 변환의 반대 방향)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * vx - s * self.cmd.linear.y) * self.dt
        self.y += (s * vx + c * self.cmd.linear.y) * self.dt
        self.yaw += self.cmd.angular.z * self.dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))   # -π ~ π 유지

        self.pose_pub.publish(Pose2D(x=self.x, y=self.y, theta=self.yaw))

        # 꼬깔에 닿았는지 검사 (규칙: 닿으면 세팅 위치로)
        touching = any(math.hypot(self.x - cx, self.y - cy) < self.cone_r + self.robot_r
                       for cx, cy in self.cones)
        if touching and not self.touching:
            self.get_logger().warn('꼬깔에 닿음!')
        self.touching = touching

        # 필드 밖으로 나갔는지 검사 (로봇 중심이 선 밖 = 몸 절반 이상 나감, 규칙: 30초 페널티)
        out = abs(self.x) > self.field_l / 2.0 or abs(self.y) > self.field_w / 2.0
        if out and not self.outside:
            self.get_logger().warn('필드 밖으로 나감!')
        self.outside = out

        # 다른 로봇과 부딪혔는지 검사
        bumping = (self.teammate is not None and
                   math.hypot(self.x - self.teammate[0], self.y - self.teammate[1]) < 2.0 * self.robot_r)
        if bumping and not self.bumping:
            self.get_logger().warn('다른 로봇과 부딪힘!')
        self.bumping = bumping

        if self.has_ball:
            self.update_ball()

        # 지나온 길 저장 (2cm 이상 움직였을 때만)
        if not self.trail or math.hypot(self.x - self.trail[-1][0], self.y - self.trail[-1][1]) > 0.02:
            self.trail.append((self.x, self.y))
            self.trail = self.trail[-2000:]
        self.publish_markers()

    def update_ball(self):
        """공에 닿은 로봇(나, 다른 로봇)이 있으면 공을 로봇 몸 바깥으로 밀어낸다"""
        robots = [('', self.x, self.y, self.yaw)]
        if self.teammate is not None:
            robots.append(('다른 로봇이 ', self.teammate[0], self.teammate[1], 0.0))
        pushing = False
        for who, rx, ry, ryaw in robots:
            dx, dy = self.ball[0] - rx, self.ball[1] - ry
            d, touch_d = math.hypot(dx, dy), self.robot_r + self.ball_r
            if d < touch_d:
                if d < 1e-6:
                    dx, dy, d = math.cos(ryaw), math.sin(ryaw), 1.0
                self.ball = [rx + dx / d * touch_d, ry + dy / d * touch_d]
                if not self.pushing:
                    self.get_logger().warn(f'{who}공을 밀었음!')
                pushing = True
        self.pushing = pushing
        ball = PointStamped()
        ball.header.frame_id = FRAME
        ball.header.stamp = self.get_clock().now().to_msg()
        ball.point.x, ball.point.y = float(self.ball[0]), float(self.ball[1])
        self.ball_pub.publish(ball)

    def publish_vision(self):
        """가짜 Vision: 시야 안 꼬깔만 로봇 기준(앞, 왼쪽)으로 + 오차"""
        msg = PoseArray()
        msg.header.frame_id = ROBOT_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()
        noise = self.get_parameter('vision_noise').value
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        for cx, cy in self.cones:
            dx, dy = cx - self.x, cy - self.y
            front, left = c * dx + s * dy, -s * dx + c * dy
            if math.hypot(front, left) < self.view_range and abs(math.atan2(left, front)) < self.view_half:
                ps = Pose()
                ps.position.x = front + random.uniform(-noise, noise)
                ps.position.y = left + random.uniform(-noise, noise)
                ps.orientation.w = 1.0
                msg.poses.append(ps)
        self.vision_pub.publish(msg)

    # ---------------- RViz2 화면 그리기 ----------------
    def make_marker(self, mid, mtype, r, g, b, a=1.0):
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = 'nav', mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        return m

    def publish_markers(self):
        arr = MarkerArray()
        hl, hw = self.field_l / 2.0, self.field_w / 2.0

        # 0: 필드 바깥선 + 센터라인
        field = self.make_marker(0, Marker.LINE_LIST, 1.0, 1.0, 1.0)
        field.scale.x = 0.03
        corners = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
        for i in range(4):
            a, b = corners[i], corners[(i + 1) % 4]
            field.points += [Point(x=a[0], y=a[1]), Point(x=b[0], y=b[1])]
        field.points += [Point(x=0.0, y=-hw), Point(x=0.0, y=hw)]
        arr.markers.append(field)

        # 1: 로봇 (초록 화살표 = 바라보는 방향)
        robot = self.make_marker(1, Marker.ARROW, 0.1, 0.8, 0.2)
        robot.pose.position.x, robot.pose.position.y = self.x, self.y
        robot.pose.orientation.z = math.sin(self.yaw / 2.0)
        robot.pose.orientation.w = math.cos(self.yaw / 2.0)
        robot.scale.x, robot.scale.y, robot.scale.z = 0.4, 0.12, 0.12
        arr.markers.append(robot)

        # 2: 지나온 길
        if len(self.trail) >= 2:
            trail = self.make_marker(2, Marker.LINE_STRIP, 0.1, 0.8, 0.2, 0.6)
            trail.scale.x = 0.02
            trail.points = [Point(x=px, y=py) for px, py in self.trail]
            arr.markers.append(trail)

        # 3: 카메라 시야 (연한 파란 부채꼴 테두리)
        fov = self.make_marker(3, Marker.LINE_STRIP, 0.4, 0.7, 1.0, 0.5)
        fov.scale.x = 0.015
        pts = [Point(x=self.x, y=self.y)]
        for k in range(11):
            a = self.yaw - self.view_half + 2.0 * self.view_half * k / 10
            pts.append(Point(x=self.x + self.view_range * math.cos(a),
                             y=self.y + self.view_range * math.sin(a)))
        pts.append(Point(x=self.x, y=self.y))
        fov.points = pts
        arr.markers.append(fov)

        # 10~: 실제 꼬깔 (주황 원기둥)
        for k, (cx, cy) in enumerate(self.cones):
            m = self.make_marker(10 + k, Marker.CYLINDER, 1.0, 0.4, 0.0)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = cx, cy, 0.15
            m.scale.x = m.scale.y = 2.0 * self.cone_r
            m.scale.z = 0.3
            arr.markers.append(m)

        # 5: 필드 바닥판 (RViz2 "Publish Point"는 클릭한 곳에 물체가 있어야 점이 찍힘)
        floor = self.make_marker(5, Marker.CUBE, 0.12, 0.28, 0.12)
        floor.pose.position.z = -0.02
        floor.scale.x, floor.scale.y, floor.scale.z = self.field_l + 1.0, self.field_w + 1.0, 0.01
        arr.markers.append(floor)

        # 4: 공 (하얀 공)
        if self.has_ball:
            m = self.make_marker(4, Marker.SPHERE, 1.0, 1.0, 1.0)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = self.ball[0], self.ball[1], self.ball_r
            m.scale.x = m.scale.y = m.scale.z = 2.0 * self.ball_r
            arr.markers.append(m)

        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotNode()
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
