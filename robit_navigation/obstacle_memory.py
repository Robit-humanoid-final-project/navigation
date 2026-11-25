"""
꼬깔 기억하기 (ROS2와 상관없는 순수 계산)

Localization(world_model)은 지금 카메라에 보이는 꼬깔만 필드 기준으로 알려준다.
그대로 쓰면 로봇이 고개를 돌리거나 꼬깔을 지나치는 순간 꼬깔이 사라져서
A-star가 그쪽으로 길을 낼 수 있다. 그래서:
  1. 받은 꼬깔을 필드 기준으로 기억한다
  2. 같은 꼬깔을 다시 보면 위치를 조금씩 보정한다 (Vision 오차 줄이기)
  3. 두 번 이상 본 꼬깔만 진짜로 인정한다 (한 번 잘못 본 것 무시)
  4. 카메라에 보여야 할 곳인데 한동안 안 보이면 지운다 (잘못 본 것 / 치워진 꼬깔)
  5. 카메라 밖이어도 아주 오래(max_unseen_time) 다시 안 보이면 지운다
     (내 위치가 잠깐 틀렸을 때 엉뚱한 곳에 찍힌 "유령 꼬깔"이 계속 남지 않게)
  6. 필드 밖에 보인 꼬깔은 버린다 (꼬깔은 필드 안에만 있음)

꼬깔 고정 (꼬깔은 경기 중 안 움직이니까):
  - 로봇이 멈춰 있을 때(still) 본 위치만 모아서
  - lock_hits 번 이상 보였고, 그 위치들이 lock_spread 안에 모여 있으면 평균 위치로 고정
  - 고정된 꼬깔은 안 보여도 안 지우고, 위치도 안 바뀜 (최대 max_locked 개, 규정 = 3개)
  - 걸을 때(카메라 흔들림) 본 것은 후보로만 씀 -> 유령 꼬깔이 고정되지 않게
  - 다시 하려면 clear() (UI "꼬깔 기억 지우기")
"""
import math
from dataclasses import dataclass

from robit_navigation.nav_core import field_to_robot


@dataclass
class ObstacleMemoryParams:
    merge_dist: float = 0.5                 # 이 거리 안이면 같은 꼬깔로 봄 (m)
    min_hits: int = 2                       # 몇 번 봐야 진짜 꼬깔(후보)로 인정할지
    forget_time: float = 2.0                # 보여야 할 곳에서 이만큼 안 보이면 지움 (s)
    view_half_angle: float = math.radians(30.0)   # 카메라 시야 절반 (전체 60도)
    view_range: float = 3.0                 # 카메라가 꼬깔을 볼 수 있는 거리 (m)
    smoothing: float = 0.3                  # 다시 볼 때 위치를 얼마나 새 값 쪽으로 옮길지 (0~1)
    max_unseen_time: float = 10.0           # 카메라 밖이어도 이만큼 다시 안 보이면 지움 (s), 0 = 안 지움
    field_half_x: float = 0.0               # 필드 절반 길이 (m), 이 밖에 보인 꼬깔은 버림, 0 = 검사 안 함
    field_half_y: float = 0.0               # 필드 절반 폭 (m)
    lock_hits: int = 10                     # 멈춰서 이만큼 보이면 고정 (0 = 고정 안 함)
    lock_spread: float = 0.15               # 그 위치들이 평균에서 이 거리 안에 다 모여야 고정 (m)
    max_locked: int = 3                     # 고정할 수 있는 꼬깔 수 (규정: 꼬깔 3개)


class ObstacleMemory:
    def __init__(self, params=None):
        self.p = params or ObstacleMemoryParams()
        # [{'x', 'y', 'hits', 'last_seen', 'locked', 'still': [(x, y), ...]}, ...]  필드 기준
        self.tracks = []

    def clear(self):
        self.tracks = []

    def in_field(self, fx, fy):
        """이 점(필드 기준)이 필드 안인지 (필드 크기를 안 줬으면 항상 True)"""
        return ((self.p.field_half_x <= 0.0 or abs(fx) <= self.p.field_half_x)
                and (self.p.field_half_y <= 0.0 or abs(fy) <= self.p.field_half_y))

    def in_view(self, robot, fx, fy):
        """이 점(필드 기준)이 지금 카메라 시야 안인지"""
        front, left = field_to_robot(robot, fx, fy)
        return (math.hypot(front, left) < self.p.view_range
                and abs(math.atan2(left, front)) < self.p.view_half_angle)

    def confirmed(self):
        """경로 계획에 쓰는 꼬깔 [(x, y), ...] (필드 기준) = 고정된 것 + 인정된 후보"""
        return [(t['x'], t['y']) for t in self.tracks if t['locked'] or t['hits'] >= self.p.min_hits]

    def items(self):
        """화면 표시용 [(x, y, 고정 여부), ...]"""
        return [(t['x'], t['y'], t['locked']) for t in self.tracks
                if t['locked'] or t['hits'] >= self.p.min_hits]

    def num_locked(self):
        return sum(t['locked'] for t in self.tracks)

    def try_lock(self, t):
        """멈춰서 본 위치가 충분히 많고 잘 모여 있으면 평균 위치로 고정"""
        s = t['still']
        if (self.p.lock_hits <= 0 or len(s) < self.p.lock_hits
                or self.num_locked() >= self.p.max_locked):
            return
        mx = sum(x for x, _ in s) / len(s)
        my = sum(y for _, y in s) / len(s)
        if all(math.hypot(x - mx, y - my) <= self.p.lock_spread for x, y in s):
            t['x'], t['y'], t['locked'] = mx, my, True

    def update_field(self, robot, seen, now, still=False):
        """
        필드 기준으로 받은 경우 (Localization world_model)
        robot: 지금 내 위치 (x, y, yaw), seen: 이번에 보인 꼬깔 [(x, y), ...] (필드 기준)
        still: 로봇이 멈춰 있는지 (True일 때 본 위치만 고정에 씀)
        반환: 계획에 쓰는 꼬깔 목록이 바뀌었으면 True (경로를 다시 짜야 함)
        """
        before = self.confirmed()
        updated = set()

        for fx, fy in seen:
            if not self.in_field(fx, fy):
                continue        # 필드 밖 = 잘못 본 것 (또는 내 위치가 틀림)
            # 1. 기억 중인 꼬깔 중 가장 가까운 것 찾기
            best, best_d = None, self.p.merge_dist
            for k, t in enumerate(self.tracks):
                d = math.hypot(t['x'] - fx, t['y'] - fy)
                if d < best_d:
                    best, best_d = k, d

            if best is not None:
                t = self.tracks[best]
                t['hits'] += 1
                t['last_seen'] = now
                updated.add(best)
                if t['locked']:
                    continue    # 고정된 꼬깔: 위치 안 바꿈
                # 같은 꼬깔: 위치를 새 값 쪽으로 조금 옮겨서 오차를 줄임
                t['x'] += self.p.smoothing * (fx - t['x'])
                t['y'] += self.p.smoothing * (fy - t['y'])
                if still:
                    t['still'] = (t['still'] + [(fx, fy)])[-max(self.p.lock_hits, 1):]   # 최근 것만
                    self.try_lock(t)
            else:
                # 처음 보는 꼬깔: 새로 기억 (아직 1번만 봐서 진짜로는 인정 안 함)
                self.tracks.append({'x': fx, 'y': fy, 'hits': 1, 'last_seen': now, 'locked': False,
                                    'still': [(fx, fy)] if still else []})
                updated.add(len(self.tracks) - 1)

        # 2. 지우기 (고정된 꼬깔은 안 지움): 카메라에 보여야 할 곳인데 forget_time 동안 안 보임,
        #           또는 (카메라 밖이어도) max_unseen_time 동안 안 보임
        def keep(k, t):
            if t['locked'] or k in updated:
                return True
            unseen = now - t['last_seen']
            if self.p.max_unseen_time > 0.0 and unseen > self.p.max_unseen_time:
                return False
            return not self.in_view(robot, t['x'], t['y']) or unseen <= self.p.forget_time
        self.tracks = [t for k, t in enumerate(self.tracks) if keep(k, t)]

        # 3. 계획에 쓰는 꼬깔 목록이 바뀌었는지 (개수가 달라졌거나 10cm 넘게 움직였으면)
        after = self.confirmed()
        if len(before) != len(after):
            return True
        return any(math.hypot(a[0] - b[0], a[1] - b[1]) > 0.10 for a, b in zip(before, after))
