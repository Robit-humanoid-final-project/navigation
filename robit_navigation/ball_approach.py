"""
공을 밀지 않고 경로 짜기 (ROS2와 상관없는 순수 계산)

패스·슛을 하려면 공 바로 뒤(킥 자리)에 서야 하는데, 그냥 직진하면 공을 밀어 버린다. 그래서:
  1. 공도 꼬깔처럼 장애물로 넣어서, A-star가 공을 돌아가게 한다
  2. 목표가 공 근처(킥 자리)면: 공 -> 목표 방향으로 더 떨어진 "진입점"까지 A-star로 가고,
     진입점에서 목표까지는 똑바로 들어간다 (= 항상 공 반대편에서 다가감)
  3. 로봇이 이미 진입로 위(공 뒤 일직선)에 있으면 물러나지 않고 바로 들어간다
  4. 진입점이나 진입로가 꼬깔에 막히면, 진입점을 공 쪽으로 당겨서 다시 시도한다
     (꼬깔 바로 옆에 공이 있어도 길을 찾게)

킥 자리 예시: 공 (bx, by)를 +x 쪽으로 차려면 목표 = (bx - 0.3, by, 0도)
"""
import math


def straight_line(a, b, step):
    """a -> b 직선을 step 간격 점들로 (a는 빼고 b는 포함)"""
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(math.ceil(d / step)))
    return [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]


def plan_path(planner, start, goal, cones, ball=None, ball_radius=0.075,
              approach_margin=0.2, corridor_width=0.1):
    """
    planner: GridPlanner, start/goal: (x, y), cones: [(x, y, 반지름), ...]
    ball: (x, y) 또는 None (공을 피하지 않음)
    approach_margin: 진입점을 공 위험 범위보다 얼마나 더 바깥에 둘지 (m)
    corridor_width: 진입로 폭 (이 안에 있으면 바로 들어감) (m)
    반환: 경로 점 목록 [(x, y), ...] (못 찾으면 None)
    """
    if ball is None:
        planner.set_obstacles(cones)
        return planner.plan(start, goal)

    planner.set_obstacles(list(cones) + [(ball[0], ball[1], ball_radius)])
    R = planner.inflated_radius(ball_radius)      # 공 위험 범위 (로봇 중심이 들어가면 안 되는 곳)

    gx, gy = goal[0] - ball[0], goal[1] - ball[1]
    d_goal = math.hypot(gx, gy)
    if d_goal >= R:
        # 목표가 공에서 멀다: 공은 그냥 피해야 할 장애물
        return planner.plan(start, goal)
    if d_goal < planner.robot_radius + ball_radius:
        return None                               # 목표에 서면 몸이 공에 닿음: 갈 수 없음

    def cone_free(points):
        """점들이 모두 꼬깔 위험 범위 밖이고, 필드 선 띠·금지선 밖인지 (공은 빼고 검사)"""
        return all(math.hypot(px - cx, py - cy) >= planner.inflated_radius(cr)
                   for px, py in points for cx, cy, cr in cones) and \
            all(planner.zone_distance(px, py) >= 0.0 for px, py in points)

    if not cone_free([goal]):
        return None                               # 킥 자리가 꼬깔 범위 안이나 필드 선 근처: 갈 수 없음

    # 목표가 공 근처(킥 자리): 공 -> 목표 방향(u)으로 진입로를 만든다
    ux, uy = gx / d_goal, gy / d_goal

    # 이미 진입로 위에 있으면 (공 뒤 일직선, 목표보다 바깥쪽) 바로 목표로
    sx, sy = start[0] - ball[0], start[1] - ball[1]
    along = sx * ux + sy * uy                     # 진입로 방향으로 얼마나 나가 있나
    side = abs(sx * uy - sy * ux)                 # 진입로에서 옆으로 얼마나 벗어났나
    if along >= d_goal - 0.02 and side < corridor_width:
        line = straight_line(start, goal, planner.res)
        if cone_free(line):
            return [tuple(start)] + line

    # 진입점까지는 A-star로 공·꼬깔을 돌아서, 진입점부터는 똑바로
    # 진입점이 꼬깔에 막히면 공 쪽으로 당겨서 다시 (예: 0.2 -> 0.1 -> 0.05m)
    for m in (approach_margin, approach_margin / 2.0, min(0.05, approach_margin)):
        pre = (ball[0] + ux * (R + m), ball[1] + uy * (R + m))
        line = straight_line(pre, goal, planner.res)
        if not cone_free([pre] + line):
            continue
        head = planner.plan(start, pre)
        if head is not None:
            return head + line
    return None
