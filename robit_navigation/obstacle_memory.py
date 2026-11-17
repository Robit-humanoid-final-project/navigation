"""
꼬깔 기억하기 (ROS2와 상관없는 순수 계산)

Vision은 지금 카메라에 보이는 꼬깔만 "로봇 기준(앞, 왼쪽)"으로 알려준다.
그대로 쓰면 로봇이 고개를 돌리거나 꼬깔을 지나치는 순간 꼬깔이 사라져서
A-star가 그쪽으로 길을 낼 수 있다. 그래서:
  1. 받은 꼬깔을 필드 기준으로 바꿔서 기억한다 (꼬깔은 경기 중 움직이지 않음)
  2. 같은 꼬깔을 다시 보면 위치를 조금씩 보정한다 (Vision 오차 줄이기)
  3. 두 번 이상 본 꼬깔만 진짜로 인정한다 (한 번 잘못 본 것 무시)
  4. 카메라에 보여야 할 곳인데 한동안 안 보이면 지운다 (잘못 본 것 / 치워진 꼬깔)
"""
import math
from dataclasses import dataclass

from robit_navigation.nav_core import field_to_robot, robot_to_field


@dataclass
class ObstacleMemoryParams:
    merge_dist: float = 0.3                 # 이 거리 안이면 같은 꼬깔로 봄 (m)
    min_hits: int = 2                       # 몇 번 봐야 진짜 꼬깔로 인정할지
    forget_time: float = 2.0                # 보여야 할 곳에서 이만큼 안 보이면 지움 (s)
    view_half_angle: float = math.radians(30.0)   # 카메라 시야 절반 (전체 60도)
    view_range: float = 3.0                 # 카메라가 꼬깔을 볼 수 있는 거리 (m)
    smoothing: float = 0.3                  # 다시 볼 때 위치를 얼마나 새 값 쪽으로 옮길지 (0~1)


class ObstacleMemory:
    def __init__(self, params=None):
        self.p = params or ObstacleMemoryParams()
        self.tracks = []    # [{'x', 'y', 'hits', 'last_seen'}, ...]  필드 기준

    def clear(self):
        self.tracks = []

    def in_view(self, robot, fx, fy):
        """이 점(필드 기준)이 지금 카메라 시야 안인지"""
        front, left = field_to_robot(robot, fx, fy)
        return (math.hypot(front, left) < self.p.view_range
                and abs(math.atan2(left, front)) < self.p.view_half_angle)

    def confirmed(self):
        """진짜로 인정된 꼬깔 [(x, y), ...] (필드 기준)"""
        return [(t['x'], t['y']) for t in self.tracks if t['hits'] >= self.p.min_hits]

    def update(self, robot, seen, now):
        """
        robot: 지금 내 위치 (x, y, yaw), seen: 이번에 보인 꼬깔 [(앞, 왼쪽), ...] (로봇 기준)
        now: 지금 시간 (s)
        반환: 진짜로 인정된 꼬깔 목록이 바뀌었으면 True (경로를 다시 짜야 함)
        """
        before = self.confirmed()
        updated = set()

        for front, left in seen:
            # 1. 로봇 기준 -> 필드 기준
            fx, fy = robot_to_field(robot, front, left)

            # 2. 기억 중인 꼬깔 중 가장 가까운 것 찾기
            best, best_d = None, self.p.merge_dist
            for k, t in enumerate(self.tracks):
                d = math.hypot(t['x'] - fx, t['y'] - fy)
                if d < best_d:
                    best, best_d = k, d

            if best is not None:
                # 같은 꼬깔: 위치를 새 값 쪽으로 조금 옮겨서 오차를 줄임
                t = self.tracks[best]
                t['x'] += self.p.smoothing * (fx - t['x'])
                t['y'] += self.p.smoothing * (fy - t['y'])
                t['hits'] += 1
                t['last_seen'] = now
                updated.add(best)
            else:
                # 처음 보는 꼬깔: 새로 기억 (아직 1번만 봐서 진짜로는 인정 안 함)
                self.tracks.append({'x': fx, 'y': fy, 'hits': 1, 'last_seen': now})
                updated.add(len(self.tracks) - 1)

        # 3. 카메라에 보여야 할 곳인데 한동안 안 보인 꼬깔은 지우기
        self.tracks = [
            t for k, t in enumerate(self.tracks)
            if k in updated
            or not self.in_view(robot, t['x'], t['y'])
            or now - t['last_seen'] <= self.p.forget_time
        ]

        # 4. 진짜 꼬깔 목록이 바뀌었는지 (개수가 달라졌거나 10cm 넘게 움직였으면)
        after = self.confirmed()
        if len(before) != len(after):
            return True
        return any(math.hypot(a[0] - b[0], a[1] - b[1]) > 0.10 for a, b in zip(before, after))
