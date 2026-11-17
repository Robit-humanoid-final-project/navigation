// 격자 A-star 경로 계획 (ROS2와 상관없는 순수 계산)
// 1. 필드를 격자(바둑판)로 나눈다
// 2. 꼬깔을 '꼬깔 반지름 + 로봇 반지름 + 여유'만큼 부풀려서 그 안의 칸을 막는다
// 3. A-star로 출발 칸 -> 목표 칸 최단 경로를 찾는다 (f = g + h)
// 4. 칸 번호를 다시 필드 좌표 점 목록으로 바꿔서 돌려준다
#pragma once

#include <vector>

#include "robit_navigation_cpp/nav_core.hpp"

namespace robit_navigation
{

struct Obstacle
{
  double x{0.0};
  double y{0.0};
  double radius{0.1};   // 꼬깔 바닥 반지름 (m)
};

class GridPlanner
{
public:
  GridPlanner(
    double x_min, double x_max, double y_min, double y_max,
    double resolution = 0.1, double robot_radius = 0.2, double margin = 0.15);

  // R = 꼬깔 반지름 + 로봇 반지름 + 여유
  double inflatedRadius(double cone_radius) const;

  // 꼬깔 목록을 받아서 막힌 칸 다시 계산
  void setObstacles(const std::vector<Obstacle> & obstacles);

  // 점 하나가 안전한지 (부풀린 원 밖인지)
  bool isFreePoint(double x, double y) const;

  // 경로 계획. 성공하면 true, path에 점 목록을 채움
  bool plan(const Point2D & start, const Point2D & goal, std::vector<Point2D> & path);

  int lastExpanded() const {return last_expanded_;}
  const std::vector<Obstacle> & obstacles() const {return obstacles_;}

private:
  bool toCell(double x, double y, int & i, int & j) const;
  Point2D toPoint(int i, int j) const;
  bool inside(int i, int j) const;
  int index(int i, int j) const {return i * ny_ + j;}
  bool blocked(int i, int j) const {return blocked_[index(i, j)];}

  double x_min_, y_min_, res_;
  int nx_, ny_;
  double robot_radius_, margin_;
  std::vector<Obstacle> obstacles_;
  std::vector<bool> blocked_;   // 막힌 칸 표시
  int last_expanded_{0};        // 마지막 탐색에서 확장한 칸 수 (확인용)
};

}  // namespace robit_navigation
