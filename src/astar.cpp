#include "robit_navigation_cpp/astar.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

namespace robit_navigation
{

GridPlanner::GridPlanner(
  double x_min, double x_max, double y_min, double y_max,
  double resolution, double robot_radius, double margin)
: x_min_(x_min), y_min_(y_min), res_(resolution),
  nx_(static_cast<int>(std::lround((x_max - x_min) / resolution)) + 1),
  ny_(static_cast<int>(std::lround((y_max - y_min) / resolution)) + 1),
  robot_radius_(robot_radius), margin_(margin),
  blocked_(nx_ * ny_, false)
{
}

// ---------------- 좌표 <-> 칸 번호 ----------------
bool GridPlanner::toCell(double x, double y, int & i, int & j) const
{
  i = static_cast<int>(std::lround((x - x_min_) / res_));
  j = static_cast<int>(std::lround((y - y_min_) / res_));
  return inside(i, j);
}

Point2D GridPlanner::toPoint(int i, int j) const
{
  return {x_min_ + i * res_, y_min_ + j * res_};
}

bool GridPlanner::inside(int i, int j) const
{
  return i >= 0 && i < nx_ && j >= 0 && j < ny_;
}

// ---------------- 장애물 부풀리기 ----------------
double GridPlanner::inflatedRadius(double cone_radius) const
{
  return cone_radius + robot_radius_ + margin_;
}

void GridPlanner::setObstacles(const std::vector<Obstacle> & obstacles)
{
  obstacles_ = obstacles;
  std::fill(blocked_.begin(), blocked_.end(), false);

  for (const auto & o : obstacles_) {
    const double R = inflatedRadius(o.radius);
    const int ci = static_cast<int>(std::lround((o.x - x_min_) / res_));
    const int cj = static_cast<int>(std::lround((o.y - y_min_) / res_));
    const int k = static_cast<int>(std::ceil(R / res_)) + 1;
    for (int i = ci - k; i <= ci + k; ++i) {
      for (int j = cj - k; j <= cj + k; ++j) {
        if (!inside(i, j)) {
          continue;
        }
        const Point2D p = toPoint(i, j);
        if (std::hypot(p.x - o.x, p.y - o.y) < R) {   // d < R 이면 위험
          blocked_[index(i, j)] = true;
        }
      }
    }
  }
}

bool GridPlanner::isFreePoint(double x, double y) const
{
  for (const auto & o : obstacles_) {
    if (std::hypot(x - o.x, y - o.y) < inflatedRadius(o.radius)) {
      return false;
    }
  }
  return true;
}

// ---------------- A-star ----------------
bool GridPlanner::plan(const Point2D & start, const Point2D & goal, std::vector<Point2D> & path)
{
  path.clear();
  last_expanded_ = 0;

  int si, sj, gi, gj;
  if (!toCell(start.x, start.y, si, sj) || !toCell(goal.x, goal.y, gi, gj)) {
    return false;            // 필드 밖
  }
  if (blocked(gi, gj)) {
    return false;            // 목표가 꼬깔 범위 안 -> 갈 수 없음
  }

  const int start_id = index(si, sj);
  const int goal_id = index(gi, gj);
  const double INF = std::numeric_limits<double>::infinity();

  // h: 목표까지 직선거리 (어림값)
  auto h = [&](int i, int j) {
      return std::hypot(i - gi, j - gj) * res_;
    };

  std::vector<double> g_cost(nx_ * ny_, INF);   // g: 출발부터 실제로 온 거리
  std::vector<int> parent(nx_ * ny_, -1);       // 각 칸이 "어디서 왔는지"
  std::vector<bool> closed(nx_ * ny_, false);   // 확장 완료한 칸

  // 후보 목록: f가 가장 작은 칸부터 꺼냄 (f, 칸 번호)
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open_list;

  g_cost[start_id] = 0.0;
  open_list.push({h(si, sj), start_id});

  // 8방향 이웃
  const int di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const double step_cost[8] = {1, 1, 1, 1, M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};

  while (!open_list.empty()) {
    const int cur = open_list.top().second;     // f가 가장 작은 칸
    open_list.pop();
    if (closed[cur]) {
      continue;
    }
    closed[cur] = true;
    ++last_expanded_;

    if (cur == goal_id) {
      // 목표 도착 -> 거꾸로 따라가며 경로 복원
      for (int c = cur; c != -1; c = parent[c]) {
        path.push_back(toPoint(c / ny_, c % ny_));
      }
      std::reverse(path.begin(), path.end());
      path.front() = start;    // 첫 점은 실제 로봇 위치
      path.back() = goal;      // 마지막 점은 실제 목표
      return true;
    }

    const int ci = cur / ny_;
    const int cj = cur % ny_;
    const bool cur_blocked = blocked(ci, cj);

    for (int k = 0; k < 8; ++k) {
      const int ni = ci + di[k];
      const int nj = cj + dj[k];
      if (!inside(ni, nj)) {
        continue;
      }
      const int nb = index(ni, nj);
      if (closed[nb]) {
        continue;
      }
      // 안전한 칸에서 막힌 칸으로는 못 들어감
      // (로봇이 이미 막힌 범위 안에 있으면 밖으로 빠져나가는 것만 허용)
      if (blocked(ni, nj) && !cur_blocked) {
        continue;
      }
      // 대각선 이동할 때 막힌 모서리를 스치지 않게
      if (di[k] != 0 && dj[k] != 0 && !cur_blocked) {
        if (blocked(ci + di[k], cj) || blocked(ci, cj + dj[k])) {
          continue;
        }
      }
      const double new_g = g_cost[cur] + step_cost[k] * res_;
      if (new_g < g_cost[nb]) {
        g_cost[nb] = new_g;
        parent[nb] = cur;
        open_list.push({new_g + h(ni, nj), nb});    // f = g + h
      }
    }
  }
  return false;    // 길이 완전히 막힘
}

}  // namespace robit_navigation
