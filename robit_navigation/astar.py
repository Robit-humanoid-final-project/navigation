"""
격자 A-star 경로 계획
- ROS2와 상관없는 순수 계산
- 단위: m

순서
  1. 필드를 격자(바둑판)로 나눈다
  2. 꼬깔을 '꼬깔 반지름 + 로봇 반지름 + 여유'만큼 부풀려서 그 안의 칸을 막는다
  3. A-star로 출발 칸 -> 목표 칸 최단 경로를 찾는다 (f = g + h)
  4. 칸 번호를 다시 필드 좌표 점 목록으로 바꿔서 돌려준다
"""
import heapq
import math


class GridPlanner:
    def __init__(self, x_min, x_max, y_min, y_max, resolution=0.1,
                 robot_radius=0.2, margin=0.15):
        self.x_min, self.y_min = x_min, y_min
        self.res = resolution                       # 칸 한 변 길이 (m)
        self.nx = int(round((x_max - x_min) / resolution)) + 1
        self.ny = int(round((y_max - y_min) / resolution)) + 1
        self.robot_radius = robot_radius            # 로봇 몸통 반지름 (m)
        self.margin = margin                        # 추가 여유 (m)
        self.obstacles = []                         # [(x, y, 꼬깔 반지름), ...]
        self.blocked = [[False] * self.ny for _ in range(self.nx)]
        self.last_expanded = 0                      # 마지막 탐색에서 확장한 칸 수 (확인용)

    # ---------------- 좌표 <-> 칸 번호 ----------------
    def to_cell(self, x, y):
        i = int(round((x - self.x_min) / self.res))
        j = int(round((y - self.y_min) / self.res))
        return i, j

    def to_point(self, i, j):
        return self.x_min + i * self.res, self.y_min + j * self.res

    def inside(self, i, j):
        return 0 <= i < self.nx and 0 <= j < self.ny

    # ---------------- 장애물 부풀리기 ----------------
    def inflated_radius(self, cone_radius):
        """R = 꼬깔 반지름 + 로봇 반지름 + 여유"""
        return cone_radius + self.robot_radius + self.margin

    def set_obstacles(self, obstacles):
        """obstacles: [(x, y, 꼬깔 반지름), ...]  필드 기준"""
        self.obstacles = list(obstacles)
        self.blocked = [[False] * self.ny for _ in range(self.nx)]
        for ox, oy, r in self.obstacles:
            R = self.inflated_radius(r)
            ci, cj = self.to_cell(ox, oy)
            k = int(math.ceil(R / self.res)) + 1
            for i in range(ci - k, ci + k + 1):
                for j in range(cj - k, cj + k + 1):
                    if not self.inside(i, j):
                        continue
                    px, py = self.to_point(i, j)
                    if math.hypot(px - ox, py - oy) < R:       # d < R 이면 위험
                        self.blocked[i][j] = True

    def clearance(self, x, y):
        """가장 가까운 위험 범위 테두리까지 거리 (음수 = 범위 안, 작을수록 깊이 들어감)"""
        return min((math.hypot(x - ox, y - oy) - self.inflated_radius(r)
                    for ox, oy, r in self.obstacles), default=float('inf'))

    def is_free_point(self, x, y):
        """점 하나가 안전한지 (부풀린 원 밖인지)"""
        return all(math.hypot(x - ox, y - oy) >= self.inflated_radius(r)
                   for ox, oy, r in self.obstacles)

    # ---------------- A-star ----------------
    def plan(self, start_xy, goal_xy):
        """
        start_xy, goal_xy: (x, y) 필드 기준
        반환: 경로 점 목록 [(x, y), ...]  (못 찾으면 None)
        """
        s = self.to_cell(*start_xy)
        g = self.to_cell(*goal_xy)
        if not self.inside(*s) or not self.inside(*g):
            return None
        if self.blocked[g[0]][g[1]]:
            return None              # 목표가 꼬깔 범위 안 -> 갈 수 없음

        def h(c):                    # h: 목표까지 직선거리 (어림값)
            return math.hypot(c[0] - g[0], c[1] - g[1]) * self.res

        # 8방향 이웃: (i 변화, j 변화, 이동 거리)
        moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                 (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]

        g_cost = {s: 0.0}            # g: 출발부터 실제로 온 거리
        parent = {s: None}           # 각 칸이 "어디서 왔는지"
        open_list = [(h(s), s)]      # 후보 목록 (f가 작은 것부터 꺼냄)
        closed = set()               # 확장 완료한 칸
        self.last_expanded = 0

        while open_list:
            f, cur = heapq.heappop(open_list)       # f가 가장 작은 칸
            if cur in closed:
                continue
            closed.add(cur)
            self.last_expanded += 1

            if cur == g:                            # 목표 도착 -> 거꾸로 따라가며 경로 복원
                cells = []
                while cur is not None:
                    cells.append(cur)
                    cur = parent[cur]
                cells.reverse()
                path = [self.to_point(i, j) for i, j in cells]
                path[0] = tuple(start_xy)           # 첫 점은 실제 로봇 위치
                path[-1] = tuple(goal_xy)           # 마지막 점은 실제 목표
                return path

            for di, dj, cost in moves:
                nb = (cur[0] + di, cur[1] + dj)
                if not self.inside(*nb) or nb in closed:
                    continue
                cur_blocked = self.blocked[cur[0]][cur[1]]
                if self.blocked[nb[0]][nb[1]]:
                    # 안전한 칸에서 막힌 칸으로는 못 들어감
                    if not cur_blocked:
                        continue
                    # 로봇이 이미 막힌 범위 안이면: 위험물에서 멀어지는 쪽으로만 빠져나감
                    # (안 그러면 꼬깔·공을 가로지르는 경로가 나올 수 있음)
                    if self.clearance(*self.to_point(*nb)) <= self.clearance(*self.to_point(*cur)):
                        continue
                # 대각선 이동할 때 막힌 모서리를 스치지 않게
                if di != 0 and dj != 0 and not cur_blocked:
                    if self.blocked[cur[0] + di][cur[1]] or self.blocked[cur[0]][cur[1] + dj]:
                        continue
                new_g = g_cost[cur] + cost * self.res
                if new_g < g_cost.get(nb, float('inf')):
                    g_cost[nb] = new_g
                    parent[nb] = cur
                    heapq.heappush(open_list, (new_g + h(nb), nb))   # f = g + h

        return None                  # 길이 완전히 막힘
