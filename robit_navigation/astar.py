"""
격자 A-star 경로 계획
- ROS2와 상관없는 순수 계산
- 단위: m

순서
  1. 필드를 격자(바둑판)로 나눈다
  2. 꼬깔을 '꼬깔 반지름 + 로봇 반지름 + 여유'만큼 부풀려서 그 안의 칸을 막는다
     필드 선 안쪽 boundary_margin 띠도 막는다 (로봇 몸이 필드 밖으로 안 나가게, 나가면 30초 페널티)
     max_x 를 정하면 그보다 오른쪽도 막는다 (Ready 때 RIGHT 진영에 들어가면 실패)
  3. A-star로 출발 칸 -> 목표 칸 최단 경로를 찾는다 (f = g + h)
  4. 칸 번호를 다시 필드 좌표 점 목록으로 바꿔서 돌려준다
"""
import heapq
import math


class GridPlanner:
    def __init__(self, x_min, x_max, y_min, y_max, resolution=0.1,
                 robot_radius=0.2, margin=0.15, boundary_margin=0.0):
        self.x_min, self.y_min = x_min, y_min
        self.x_max, self.y_max = x_max, y_max
        self.boundary_margin = boundary_margin      # 로봇 중심이 필드 선에서 떨어져야 하는 거리 (m)
        self.max_x = None                           # 로봇 중심이 넘으면 안 되는 x (None = 제한 없음)
        self.res = resolution                       # 칸 한 변 길이 (m)
        self.nx = int(round((x_max - x_min) / resolution)) + 1
        self.ny = int(round((y_max - y_min) / resolution)) + 1
        self.robot_radius = robot_radius            # 로봇 몸통 반지름 (m)
        self.margin = margin                        # 추가 여유 (m)
        self.obstacles = []                         # [(x, y, 꼬깔 반지름), ...]
        self.blocked = self.empty_map()
        self.last_expanded = 0                      # 마지막 탐색에서 확장한 칸 수 (확인용)
        self.prefer, self.prefer_cost = None, 0.0     # 지금 길 근처 칸 (set_prefer)
        self.goal_moved = False                     # 마지막 계획에서 목표가 막혀서 가까운 곳으로 바꿨는지

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

    # ---------------- 필드 테두리 ----------------
    def edge_distance(self, x, y):
        """필드 선까지 가장 가까운 거리 (선 밖이면 음수)"""
        return min(x - self.x_min, self.x_max - x, y - self.y_min, self.y_max - y)

    def zone_distance(self, x, y):
        """막힌 테두리(필드 선 안쪽 띠, max_x 금지선)까지 거리 (음수 = 막힌 곳 안)"""
        d = self.edge_distance(x, y) - self.boundary_margin if self.boundary_margin > 0.0 else float('inf')
        if self.max_x is not None:
            d = min(d, self.max_x - x)
        return d

    def set_max_x(self, max_x):
        """오른쪽 금지선 정하기 (None = 없앰). 지도를 다시 만든다"""
        if max_x != self.max_x:
            self.max_x = max_x
            self.set_obstacles(self.obstacles)

    def empty_map(self):
        """꼬깔 없는 지도: 필드 선 안쪽 띠와 금지선 오른쪽만 막힘"""
        blocked = [[False] * self.ny for _ in range(self.nx)]
        if self.boundary_margin > 0.0 or self.max_x is not None:
            for i in range(self.nx):
                for j in range(self.ny):
                    if self.zone_distance(*self.to_point(i, j)) < 0.0:
                        blocked[i][j] = True
        return blocked

    def set_obstacles(self, obstacles):
        """obstacles: [(x, y, 꼬깔 반지름), ...]  필드 기준"""
        self.obstacles = list(obstacles)
        self.blocked = self.empty_map()
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
        return min([math.hypot(x - ox, y - oy) - self.inflated_radius(r)
                    for ox, oy, r in self.obstacles] + [self.zone_distance(x, y)])

    def is_free_point(self, x, y):
        """점 하나가 안전한지 (부풀린 원 밖이고, 필드 선 안쪽 여유 띠 밖인지)"""
        if self.zone_distance(x, y) < 0.0:
            return False
        return all(math.hypot(x - ox, y - oy) >= self.inflated_radius(r)
                   for ox, oy, r in self.obstacles)

    def segment_free(self, a, b, step=0.05):
        """
        a에서 b까지 직선으로 가도 안전한지 (꼬깔·공·다른 로봇 범위, 필드 선 띠를 안 지나는지)
        - 격자 칸 크기 때문에 생기는 작은 오차(칸 절반)는 봐줌 (margin 안쪽이라 안 부딪힘)
        - 로봇이 이미 그보다 깊이 범위 안이면: 지금보다 더 깊이 들어가지만 않으면 OK (빠져나가는 중)
        """
        limit = min(self.clearance(a[0], a[1]), -0.5 * self.res)
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / step)))
        for k in range(1, n + 1):
            x = a[0] + (b[0] - a[0]) * k / n
            y = a[1] + (b[1] - a[1]) * k / n
            if self.clearance(x, y) < limit:
                return False
        return True

    def set_prefer(self, path, cost=0.5, width=0.4):
        """
        한 번 정한 쪽 유지: 꼬깔 위치가 조금 흔들리면 왼쪽/오른쪽으로 돌아가는 길 길이가 비슷해서
        A-star가 매번 다른 쪽을 고를 수 있음 -> 왔다 갔다 하다 꼬깔을 박음. 그래서
        지금 길(path)에서 width 밖 칸은 지나가는 비용을 (1 + cost)배로 함
        -> 꼬깔이 조금 움직이면 같은 쪽으로 살짝 옮긴 길을 고름, 그쪽이 막히면 다른 쪽으로 감
        path = None 또는 cost = 0 이면 안 씀
        """
        self.prefer, self.prefer_cost = None, cost
        if not path or cost <= 0.0:
            return
        k = int(math.ceil(width / self.res))
        self.prefer = set()
        for x, y in path:
            ci, cj = self.to_cell(x, y)
            for i in range(ci - k, ci + k + 1):
                for j in range(cj - k, cj + k + 1):
                    px, py = self.to_point(i, j)
                    if math.hypot(px - x, py - y) <= width:
                        self.prefer.add((i, j))

    # ---------------- A-star ----------------
    def nearest_free(self, cell, max_dist=1.0):
        """cell 에서 max_dist(m) 안의 갈 수 있는 칸 중 가장 가까운 칸 (없으면 None)"""
        k = int(math.ceil(max_dist / self.res))
        best, best_d = None, max_dist + 1e-9
        for i in range(cell[0] - k, cell[0] + k + 1):
            for j in range(cell[1] - k, cell[1] + k + 1):
                if self.inside(i, j) and not self.blocked[i][j]:
                    d = math.hypot(i - cell[0], j - cell[1]) * self.res
                    if d < best_d:
                        best, best_d = (i, j), d
        return best

    def plan(self, start_xy, goal_xy):
        """
        start_xy, goal_xy: (x, y) 필드 기준
        반환: 경로 점 목록 [(x, y), ...]  (못 찾으면 None)
        """
        s = self.to_cell(*start_xy)
        g = self.to_cell(*goal_xy)
        # 출발점이 필드 밖이면 (세팅 위치는 라인보다 뒤에 둠) 가장 가까운 필드 안 칸에서 시작
        s = (min(max(s[0], 0), self.nx - 1), min(max(s[1], 0), self.ny - 1))
        self.goal_moved = False
        if not self.inside(*g):
            return None              # 목표가 필드 밖
        if self.blocked[g[0]][g[1]]:
            # 목표가 꼬깔 범위 안 (꼬깔 위치가 잠깐 흔들려서 겹친 것일 수도 있음)
            # -> 바로 포기하지 않고 목표에서 가장 가까운 갈 수 있는 칸으로 감
            g = self.nearest_free(g)
            if g is None:
                return None          # 근처에 갈 수 있는 곳이 없음
            goal_xy = self.to_point(*g)
            self.goal_moved = True

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
                if self.prefer is not None and nb not in self.prefer:
                    new_g += cost * self.res * self.prefer_cost     # 지금 길에서 벗어난 칸 = 비쌈
                if new_g < g_cost.get(nb, float('inf')):
                    g_cost[nb] = new_g
                    parent[nb] = cur
                    heapq.heappush(open_list, (new_g + h(nb), nb))   # f = g + h

        return None                  # 길이 완전히 막힘
