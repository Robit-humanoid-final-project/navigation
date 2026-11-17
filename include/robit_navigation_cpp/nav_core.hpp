// 내비게이션 계산 부분 (ROS2와 상관없는 순수 계산)
// - 좌표 변환, 각도 정규화, PID 제어, 도착 판단, Carrot(당근) 따라가기
// - 단위: 거리 m, 각도 rad, 시간 s
// - 좌표 약속 (ROS 표준, 팀 합의 전 임시): 로봇 기준 +x = 앞, +y = 왼쪽, 반시계 회전 = +
#pragma once

#include <vector>

namespace robit_navigation
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};   // rad
};

// 걸음 명령 + 상태
struct Command
{
  double front{0.0};    // 앞 속도 (m/s), 앞 + / 뒤 -
  double left{0.0};     // 왼쪽 속도 (m/s), 왼쪽 + / 오른쪽 -
  double turn{0.0};     // 회전 속도 (rad/s), 반시계 +
  bool arrived{false};  // 도착했는지
  double remain{0.0};   // 남은 거리 (m)
  Point2D carrot;       // 이번에 향한 임시 목표 (RViz 표시용)
};

// 각도를 -π ~ +π 로 맞추기 (가장 짧게 도는 쪽)
double normalizeAngle(double angle);

// 필드 좌표의 목표를 로봇 기준(앞, 왼쪽)으로 바꾸기
Point2D fieldToRobot(const Pose2D & robot, double goal_x, double goal_y);

// 최대값 제한: -limit ~ +limit
double clip(double value, double limit);

// ---------------- PID ----------------
// u = Kp*e + Ki*(오차를 쌓은 값) + Kd*(오차 변화 속도)
struct PidGains
{
  double kp{0.0};
  double ki{0.0};
  double kd{0.0};
  double i_limit{0.0};    // I가 낼 수 있는 최대 명령 (적분 제한)
  double d_alpha{0.5};    // D 부드럽게: 0~1, 작을수록 더 부드럽지만 반응이 느림
};

class Pid
{
public:
  explicit Pid(const PidGains & gains = PidGains());

  // error: 오차 (목표 - 현재), dt: 지난 계산 후 흐른 시간 (s)
  // use_i: I를 쓸지 (false면 쌓인 값을 0으로), limit: 최대 명령
  double update(double error, double dt, bool use_i, double limit);

  // 새 목표가 오거나 오차의 뜻이 바뀌면 쌓인 값과 이전 오차를 지운다
  void reset();

private:
  PidGains g_;
  double integral_{0.0};     // 오차를 쌓은 값 (∫e dt)
  double prev_error_{0.0};   // 이전 오차 (D 계산용)
  double d_filtered_{0.0};   // 부드럽게 만든 오차 변화 속도
  bool has_prev_{false};
};

// 튜닝 값
struct ControllerParams
{
  // 이동 (앞뒤, 좌우)
  double kp_move{0.8};
  double ki_move{0.0};
  double kd_move{0.0};
  double i_limit_move{0.1};   // I가 낼 수 있는 최대 이동 속도 (m/s)
  // 회전
  double kp_turn{1.5};
  double ki_turn{0.0};
  double kd_turn{0.0};
  double i_limit_turn{0.3};   // I가 낼 수 있는 최대 회전 속도 (rad/s)
  // 공통
  double d_alpha{0.5};        // D 부드럽게 (0~1)
  double max_move{0.3};       // 최대 이동 속도 (m/s)
  double max_turn{1.0};       // 최대 회전 속도 (rad/s)
  double tol_dist{0.10};      // 허용 거리 (m)
  double tol_yaw{0.1745};     // 허용 각도 (rad) = 10도
};

// 목표 위치까지 PID로 이동하고 도착을 판단
class GoalController
{
public:
  explicit GoalController(const ControllerParams & params = ControllerParams());
  Command step(const Pose2D & robot, const Pose2D & goal, double dt);
  void reset();
  const ControllerParams & params() const {return params_;}

private:
  enum class Mode {NONE, APPROACH, ALIGN};   // 다가가는 중 / 제자리에서 방향 맞추는 중
  ControllerParams params_;
  Pid front_pid_, left_pid_, turn_pid_;
  Mode mode_{Mode::NONE};
};

// 경로 위 조금 앞선 점(Carrot)을 따라가기
class PathFollower
{
public:
  PathFollower(const ControllerParams & params, int carrot_points);
  Command step(
    const Pose2D & robot, const std::vector<Point2D> & path, const Pose2D & goal, double dt);
  void reset();

private:
  ControllerParams params_;
  GoalController final_;               // 경로 끝: 최종 목표에 정확히 도착
  Pid front_pid_, left_pid_, turn_pid_;   // 경로 중간: 당근 따라가기 (I 안 씀)
  int carrot_points_;   // 몇 개 앞의 점을 당근으로 쓸지 (m가 아니라 개수)
  bool on_carrot_{false};
};

}  // namespace robit_navigation