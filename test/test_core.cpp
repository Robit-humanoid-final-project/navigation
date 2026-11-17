// 계산 부분 확인: 손으로 푼 답, A-star 꼬깔 회피, 시뮬레이션 도착, PID
// 실행: ~/nav_ws/install/robit_navigation_cpp/lib/robit_navigation_cpp/test_core
#include <cmath>
#include <cstdio>
#include <vector>

#include "robit_navigation_cpp/astar.hpp"
#include "robit_navigation_cpp/nav_core.hpp"

using namespace robit_navigation;

static int failures = 0;
#define CHECK(cond) do { \
    if (cond) {std::printf("  [통과] %s\n", #cond);} \
    else {std::printf("  [실패] %s (줄 %d)\n", #cond, __LINE__); ++failures;} \
} while (0)

static bool close(double a, double b, double tol = 1e-6) {return std::abs(a - b) < tol;}
static double rad(double deg) {return deg * M_PI / 180.0;}

static const std::vector<Obstacle> CONES = {{-1.0, -1.2, 0.1}, {1.0, 0.0, 0.1}, {2.2, -0.5, 0.1}};

struct SimResult
{
  bool arrived;
  Pose2D pose;
  double min_clear;
  double time;
};

// 가짜 로봇으로 경로 계획 + 따라가기 시뮬레이션
// slip: 걷는 동안 로봇이 뒤로 밀리는 속도 (m/s, 외란)
static SimResult simulate(
  Pose2D robot, Pose2D goal, const ControllerParams & cp = ControllerParams(),
  double slip = 0.0, double max_time = 60.0)
{
  GridPlanner planner(-4.5, 4.5, -3.0, 3.0, 0.1, 0.2, 0.15);
  planner.setObstacles(CONES);
  PathFollower follower(cp, 5);
  std::vector<Point2D> path;
  const double dt = 0.1;
  double min_clear = 1e9;
  const int steps = static_cast<int>(max_time / dt);
  for (int k = 0; k < steps; ++k) {
    if (k % 10 == 0) {     // 1초마다 다시 계획
      if (!planner.plan({robot.x, robot.y}, {goal.x, goal.y}, path)) {
        return {false, robot, min_clear, k * dt};
      }
    }
    const Command cmd = follower.step(robot, path, goal, dt);
    if (cmd.arrived) {
      return {true, robot, min_clear, k * dt};
    }
    // 걷는 중이면 외란(뒤로 밀림) 추가
    const bool walking = std::abs(cmd.front) + std::abs(cmd.left) > 1e-6;
    const double vx = cmd.front - (walking ? slip : 0.0);
    const double c = std::cos(robot.yaw), s = std::sin(robot.yaw);
    robot.x += (c * vx - s * cmd.left) * dt;
    robot.y += (s * vx + c * cmd.left) * dt;
    robot.yaw = normalizeAngle(robot.yaw + cmd.turn * dt);
    for (const auto & o : CONES) {
      min_clear = std::min(min_clear, std::hypot(robot.x - o.x, robot.y - o.y) - o.radius - 0.2);
    }
  }
  return {false, robot, min_clear, max_time};
}

int main()
{
  std::printf("[좌표 변환]\n");
  Point2D p = fieldToRobot({0, 0, rad(180)}, 0, 2);
  CHECK(close(p.x, 0) && close(p.y, -2));      // 오른쪽 2m
  p = fieldToRobot({1, 1, 0}, 3, 2);
  CHECK(close(p.x, 2) && close(p.y, 1));       // 앞 2m, 왼쪽 1m
  p = fieldToRobot({0, 0, rad(-90)}, 1, 0);
  CHECK(close(p.x, 0) && close(p.y, 1));       // 왼쪽 1m

  std::printf("[각도 정규화]\n");
  CHECK(close(normalizeAngle(rad(160) - rad(-160)), rad(-40)));   // 오른쪽 40도
  CHECK(close(normalizeAngle(rad(-170) - rad(170)), rad(20)));    // 왼쪽 20도

  std::printf("[PID]\n");
  {
    Pid p_only({0.8, 0.0, 0.0, 0.1, 0.5});
    CHECK(close(p_only.update(0.1, 0.1, true, 0.3), 0.08));    // P: 0.8 x 0.1
    CHECK(close(p_only.update(3.0, 0.1, true, 0.3), 0.3));     // 최대값으로 자름
    Pid pi({0.8, 0.5, 0.0, 0.1, 0.5});
    double u = 0.0;
    for (int k = 0; k < 100; ++k) {u = pi.update(0.1, 0.1, true, 0.3);}
    CHECK(u > 0.08 && u <= 0.08 + 0.1 + 1e-9);   // I가 쌓여서 커지지만 i_limit(0.1)을 넘지 않음
    Pid pd({0.8, 0.0, 0.5, 0.0, 1.0});
    pd.update(1.0, 0.1, false, 10.0);
    CHECK(pd.update(0.9, 0.1, false, 10.0) < 0.8 * 0.9);   // 다가가는 중이면 D가 브레이크
  }

  std::printf("[A-star]\n");
  {
    GridPlanner empty(-4.5, 4.5, -3.0, 3.0);
    std::vector<Point2D> path;
    bool ok = empty.plan({-2.0, 0.0}, {2.0, 0.0}, path);
    bool straight = true;
    for (const auto & q : path) {straight = straight && std::abs(q.y) < 1e-9;}
    CHECK(ok && straight);                      // 꼬깔 없으면 직선

    GridPlanner planner(-4.5, 4.5, -3.0, 3.0, 0.1, 0.2, 0.15);
    planner.setObstacles(CONES);
    ok = planner.plan({-1.5, 0.0}, {3.0, 0.0}, path);
    bool all_free = true;
    for (const auto & q : path) {all_free = all_free && planner.isFreePoint(q.x, q.y);}
    CHECK(ok && all_free);                      // 모든 경로 점이 부풀린 원 밖

    CHECK(!planner.plan({-1.5, 0.0}, {1.05, 0.0}, path));   // 목표가 꼬깔 위 -> 실패

    std::vector<Obstacle> wall;
    for (int y = -30; y <= 30; y += 3) {wall.push_back({0.0, y / 10.0, 0.1});}
    GridPlanner walled(-4.5, 4.5, -3.0, 3.0);
    walled.setObstacles(wall);
    CHECK(!walled.plan({-2.0, 0.0}, {2.0, 0.0}, path));     // 꼬깔 벽 -> 실패
  }

  std::printf("[시뮬레이션: 미끄러짐 없음, P만]\n");
  SimResult r = simulate({-3.0, -3.0, rad(90)}, {-1.5, 0.0, 0.0});
  std::printf("  Ready: 도착=%d, %.1f초\n", r.arrived, r.time);
  CHECK(r.arrived && r.pose.x < 0.0);
  r = simulate({-1.5, 0.0, 0.0}, {3.0, 0.0, 0.0});
  std::printf("  Playing: 도착=%d, %.1f초, 꼬깔과 최소 여유 %.2fm\n", r.arrived, r.time, r.min_clear);
  CHECK(r.arrived && r.min_clear > 0.0);

  std::printf("[시뮬레이션: 걸을 때 뒤로 0.1m/s 미끄러짐]\n");
  ControllerParams p_only;   // 기본값: P만
  r = simulate({-3.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, p_only, 0.1, 30.0);
  std::printf("  P만:  도착=%d, 30초 후 남은 거리 %.3fm\n", r.arrived, std::hypot(r.pose.x + 1.0, r.pose.y));
  CHECK(!r.arrived);         // P만 쓰면 목표 앞에서 멈춤 (사이트 02장과 같은 현상)

  ControllerParams pid = p_only;
  pid.ki_move = 0.5;
  pid.kd_move = 0.1;
  pid.ki_turn = 0.3;
  pid.kd_turn = 0.1;
  r = simulate({-3.0, 0.0, 0.0}, {-1.0, 0.0, 0.0}, pid, 0.1, 30.0);
  std::printf("  PID:  도착=%d, %.1f초\n", r.arrived, r.time);
  CHECK(r.arrived);          // I가 남은 거리를 밀어서 도착

  r = simulate({-1.5, 0.0, 0.0}, {3.0, 0.0, 0.0}, pid, 0.1, 60.0);
  std::printf("  PID 꼬깔 회피: 도착=%d, %.1f초, 꼬깔과 최소 여유 %.2fm\n",
    r.arrived, r.time, r.min_clear);
  CHECK(r.arrived && r.min_clear > 0.0);

  std::printf(failures == 0 ? "\n모두 통과\n" : "\n실패 %d개\n", failures);
  return failures == 0 ? 0 : 1;
}