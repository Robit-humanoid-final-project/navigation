#include "robit_navigation_cpp/nav_core.hpp"

#include <algorithm>
#include <cmath>

namespace robit_navigation
{

double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Point2D fieldToRobot(const Pose2D & robot, double goal_x, double goal_y)
{
  // 1단계: 빼기 (목표가 로봇에서 필드 기준으로 얼마나 떨어져 있나)
  const double dx = goal_x - robot.x;
  const double dy = goal_y - robot.y;
  // 2단계: 회전 (로봇이 보는 방향 기준으로 다시 읽기)
  const double c = std::cos(robot.yaw);
  const double s = std::sin(robot.yaw);
  Point2D local;
  local.x = c * dx + s * dy;    // 앞 + / 뒤 -
  local.y = -s * dx + c * dy;   // 왼쪽 + / 오른쪽 -
  return local;
}

double clip(double value, double limit)
{
  return std::max(-limit, std::min(limit, value));
}

// ---------------- PID ----------------
Pid::Pid(const PidGains & gains)
: g_(gains) {}

void Pid::reset()
{
  integral_ = 0.0;
  prev_error_ = 0.0;
  d_filtered_ = 0.0;
  has_prev_ = false;
}

double Pid::update(double error, double dt, bool use_i, double limit)
{
  if (dt <= 0.0) {
    dt = 1e-3;
  }

  // P: 지금 오차
  const double p = g_.kp * error;

  // D: 오차가 얼마나 빨리 변하나 (위치값이 흔들리면 튀므로 부드럽게)
  if (has_prev_) {
    const double raw = (error - prev_error_) / dt;
    d_filtered_ = g_.d_alpha * raw + (1.0 - g_.d_alpha) * d_filtered_;
  }
  prev_error_ = error;
  has_prev_ = true;
  const double d = g_.kd * d_filtered_;

  // I: 오차를 계속 쌓기 (외란 때문에 남는 오차 없애기)
  if (!use_i || g_.ki <= 0.0) {
    integral_ = 0.0;
  } else {
    const double next = integral_ + error * dt;
    // 명령이 이미 최대값이면 더 쌓지 않음 (적분이 너무 커지는 것 방지)
    if (std::abs(p + g_.ki * next + d) < limit) {
      integral_ = next;
    }
    // I가 낼 수 있는 명령에 상한 (적분 제한)
    const double max_integral = g_.i_limit / g_.ki;
    integral_ = clip(integral_, max_integral);
  }
  const double i = g_.ki * integral_;

  return clip(p + i + d, limit);
}

// ---------------- GoalController ----------------
static PidGains moveGains(const ControllerParams & p)
{
  return {p.kp_move, p.ki_move, p.kd_move, p.i_limit_move, p.d_alpha};
}

static PidGains turnGains(const ControllerParams & p)
{
  return {p.kp_turn, p.ki_turn, p.kd_turn, p.i_limit_turn, p.d_alpha};
}

GoalController::GoalController(const ControllerParams & params)
: params_(params),
  front_pid_(moveGains(params)), left_pid_(moveGains(params)), turn_pid_(turnGains(params))
{
}

void GoalController::reset()
{
  front_pid_.reset();
  left_pid_.reset();
  turn_pid_.reset();
  mode_ = Mode::NONE;
}

Command GoalController::step(const Pose2D & robot, const Pose2D & goal, double dt)
{
  Command cmd;
  cmd.carrot = {goal.x, goal.y};

  // 목표를 로봇 기준으로
  const Point2D local = fieldToRobot(robot, goal.x, goal.y);
  const double dist = std::hypot(local.x, local.y);
  cmd.remain = dist;

  // 최종 방향까지 돌아야 할 각도 (목표 - 현재)
  const double final_turn = normalizeAngle(goal.yaw - robot.yaw);

  // 도착 판단: 거리와 방향 둘 다 허용 범위 안
  if (dist < params_.tol_dist && std::abs(final_turn) < params_.tol_yaw) {
    cmd.arrived = true;      // 정지 명령 (front, left, turn 모두 0)
    reset();
    return cmd;
  }

  if (dist >= params_.tol_dist) {
    // 아직 멀다: 목표 쪽으로 몸을 돌리면서 걸어간다
    if (mode_ != Mode::APPROACH) {
      turn_pid_.reset();     // 회전 오차의 뜻이 바뀌므로 초기화
      mode_ = Mode::APPROACH;
    }
    const double heading_err = std::atan2(local.y, local.x);   // 목표가 몇 도 방향인가
    // 이동: PID (I로 미끄러짐 때문에 남는 거리까지 없앰)
    cmd.front = front_pid_.update(local.x, dt, true, params_.max_move);
    cmd.left = left_pid_.update(local.y, dt, true, params_.max_move);
    // 회전: 목표 쪽 바라보기 (P + D, 가까워지면 방향이 크게 흔들리므로 I는 안 씀)
    cmd.turn = turn_pid_.update(heading_err, dt, false, params_.max_turn);
  } else {
    // 위치는 도착: 제자리에서 최종 방향만 맞춘다
    if (mode_ != Mode::ALIGN) {
      front_pid_.reset();
      left_pid_.reset();
      turn_pid_.reset();
      mode_ = Mode::ALIGN;
    }
    // 회전: PID (I로 남는 각도까지 없앰)
    cmd.turn = turn_pid_.update(final_turn, dt, true, params_.max_turn);
  }
  return cmd;
}

// ---------------- PathFollower ----------------
PathFollower::PathFollower(const ControllerParams & params, int carrot_points)
: params_(params), final_(params),
  front_pid_(moveGains(params)), left_pid_(moveGains(params)), turn_pid_(turnGains(params)),
  carrot_points_(carrot_points)
{
}

void PathFollower::reset()
{
  final_.reset();
  front_pid_.reset();
  left_pid_.reset();
  turn_pid_.reset();
  on_carrot_ = false;
}

Command PathFollower::step(
  const Pose2D & robot, const std::vector<Point2D> & path, const Pose2D & goal, double dt)
{
  const int n = static_cast<int>(path.size());

  // 1. 가장 가까운 경로 점 찾기
  int nearest = 0;
  double best = 1e18;
  for (int k = 0; k < n; ++k) {
    const double d = std::hypot(path[k].x - robot.x, path[k].y - robot.y);
    if (d < best) {
      best = d;
      nearest = k;
    }
  }

  // 남은 경로 길이
  double remain = best;
  for (int k = nearest; k < n - 1; ++k) {
    remain += std::hypot(path[k + 1].x - path[k].x, path[k + 1].y - path[k].y);
  }

  // 2. 몇 개 앞의 점 = 당근
  const int c = nearest + carrot_points_;
  if (c >= n - 1) {
    // 당근이 경로 끝에 닿음 -> 최종 목표로 정확히 이동 + 방향 맞추기 (PID)
    if (on_carrot_) {
      final_.reset();
      on_carrot_ = false;
    }
    Command cmd = final_.step(robot, goal, dt);
    cmd.remain = remain;
    return cmd;
  }

  // 3. 당근을 로봇 기준으로 바꿔서 따라가기 (P + D)
  //    당근은 항상 조금 앞에 있어서 오차가 0이 되지 않으므로 I는 쓰지 않음
  if (!on_carrot_) {
    front_pid_.reset();
    left_pid_.reset();
    turn_pid_.reset();
    on_carrot_ = true;
  }
  const Point2D local = fieldToRobot(robot, path[c].x, path[c].y);
  Command cmd;
  cmd.front = front_pid_.update(local.x, dt, false, params_.max_move);
  cmd.left = left_pid_.update(local.y, dt, false, params_.max_move);
  cmd.turn = turn_pid_.update(std::atan2(local.y, local.x), dt, false, params_.max_turn);
  cmd.remain = remain;
  cmd.carrot = path[c];
  return cmd;
}

}  // namespace robit_navigation