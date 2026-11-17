// 내비게이션 노드 (내 담당 부분)
//
// 받는 토픽 (임시 이름, 팀 합의 후 변경):
//   /robot_pose   geometry_msgs/Pose2D     내 위치 x, y [m], theta(yaw) [rad]   <- Localization 대신
//   /nav_goal     geometry_msgs/Pose2D     목표 x, y [m], theta [rad]          <- Taskplanner 대신
//   /goal_pose    geometry_msgs/PoseStamped  RViz2 "2D Goal Pose" 버튼으로 찍은 목표 (테스트용)
//   /obstacles    geometry_msgs/PoseArray  꼬깔 위치 (position.x, position.y) [m], 필드 기준 <- Vision 대신
//
// 보내는 토픽 (임시 이름, 팀 합의 후 변경):
//   /nav_cmd      geometry_msgs/Twist      linear.x 앞, linear.y 왼쪽 [m/s], angular.z 회전 [rad/s]
//   /nav_status   std_msgs/String          IDLE / MOVING / ARRIVED / NO_POSE / BLOCKED
//   /nav_path     nav_msgs/Path            A-star 경로 (RViz2 표시용)
//   /nav_debug    visualization_msgs/MarkerArray  목표, 부풀린 꼬깔 범위, 당근, 상태 글자 (RViz2 표시용)
//
// 이동, 회전 모두 PID 제어 (ki, kd = 0 이면 P만)
// 튜닝 값은 실행 중에도 바꿀 수 있음 (빌드 다시 안 해도 됨):
//   ros2 param set /navigation_node kp_move 0.5
//   ros2 param set /navigation_node ki_move 0.5

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "robit_navigation_cpp/astar.hpp"
#include "robit_navigation_cpp/nav_core.hpp"

using namespace std::chrono_literals;
namespace rn = robit_navigation;

static const char * FRAME = "map";

class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode()
  : Node("navigation_node")
  {
    // ---- 튜닝 값: launch 파일 또는 실행 중 ros2 param set 으로 바꿀 수 있음 ----
    // PID: ki, kd 를 0으로 두면 P만, kd만 올리면 PD, 둘 다 올리면 PID
    declare_parameter("kp_move", 0.8);
    declare_parameter("ki_move", 0.0);
    declare_parameter("kd_move", 0.0);
    declare_parameter("i_limit_move", 0.1);      // I가 낼 수 있는 최대 이동 속도 (m/s)
    declare_parameter("kp_turn", 1.5);
    declare_parameter("ki_turn", 0.0);
    declare_parameter("kd_turn", 0.0);
    declare_parameter("i_limit_turn", 0.3);      // I가 낼 수 있는 최대 회전 속도 (rad/s)
    declare_parameter("d_alpha", 0.5);           // D 부드럽게 (0~1, 작을수록 부드러움)
    declare_parameter("max_move", 0.3);          // m/s
    declare_parameter("max_turn", 1.0);          // rad/s
    declare_parameter("tol_dist", 0.10);         // m
    declare_parameter("tol_yaw_deg", 10.0);      // 도
    declare_parameter("carrot_points", 5);       // 당근: 몇 개 앞 점
    declare_parameter("robot_radius", 0.2);      // 로봇 몸통 반지름 (m)
    declare_parameter("margin", 0.15);           // 추가 여유 (m)
    declare_parameter("cone_radius", 0.1);       // 꼬깔 바닥 반지름 (m)
    declare_parameter("replan_period", 1.0);     // 몇 초마다 다시 계획
    // 아래 값은 시작할 때만 읽음
    rate_ = declare_parameter("rate", 10.0);               // 1초에 몇 번 계산
    field_l_ = declare_parameter("field_length", 9.0);     // m (예시 값)
    field_w_ = declare_parameter("field_width", 6.0);
    res_ = declare_parameter("grid_resolution", 0.1);      // 격자 한 칸 (m)

    applyParameters();

    // 실행 중에 값이 바뀌면: 먼저 검사하고(on_set), 통과하면 적용(post_set)
    on_set_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return checkParameters(params);});
    post_set_handle_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &) {
        applyParameters();
        need_replan_ = true;
        RCLCPP_INFO(get_logger(), "튜닝 값 변경 적용");
      });

    // 받기
    pose_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "robot_pose", 10, [this](geometry_msgs::msg::Pose2D::SharedPtr m) {
        robot_ = rn::Pose2D{m->x, m->y, m->theta};
      });
    goal_sub_ = create_subscription<geometry_msgs::msg::Pose2D>(
      "nav_goal", 10, [this](geometry_msgs::msg::Pose2D::SharedPtr m) {onGoal(*m);});
    // RViz2 "2D Goal Pose" 버튼: 위치 + 방향(쿼터니언)을 Pose2D로 바꿔서 같은 방식으로 처리
    click_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {
        geometry_msgs::msg::Pose2D g;
        g.x = m->pose.position.x;
        g.y = m->pose.position.y;
        g.theta = quaternionToYaw(m->pose.orientation);
        onGoal(g);
      });
    obstacle_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "obstacles", 10, [this](geometry_msgs::msg::PoseArray::SharedPtr m) {onObstacles(*m);});

    // 보내기
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("nav_cmd", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("nav_status", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("nav_path", 10);
    debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("nav_debug", 10);

    // 반복 (기본 0.1초마다)
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_), [this]() {loop();});

    RCLCPP_INFO(get_logger(), "내비게이션 노드 시작: 위치와 목표를 기다리는 중");
  }

private:
  // ---------------- 튜닝 값 ----------------
  // 잘못된 값(0 이하 등)이 들어오면 거부
  rcl_interfaces::msg::SetParametersResult checkParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      const std::string & n = p.get_name();
      if (n == "rate" || n == "field_length" || n == "field_width" || n == "grid_resolution") {
        result.successful = false;
        result.reason = n + " 은(는) 시작할 때만 정할 수 있음 (launch 파일에서 바꾸기)";
      } else if (n == "carrot_points" && p.as_int() < 1) {
        result.successful = false;
        result.reason = "carrot_points 는 1 이상";
      } else if (n == "d_alpha" && (p.as_double() <= 0.0 || p.as_double() > 1.0)) {
        result.successful = false;
        result.reason = "d_alpha 는 0보다 크고 1 이하";
      } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE && p.as_double() < 0.0) {
        result.successful = false;
        result.reason = n + " 는 0 이상";
      }
    }
    return result;
  }

  // 현재 파라미터 값으로 계산 도구를 다시 만든다
  void applyParameters()
  {
    rn::ControllerParams cp;
    cp.kp_move = get_parameter("kp_move").as_double();
    cp.ki_move = get_parameter("ki_move").as_double();
    cp.kd_move = get_parameter("kd_move").as_double();
    cp.i_limit_move = get_parameter("i_limit_move").as_double();
    cp.kp_turn = get_parameter("kp_turn").as_double();
    cp.ki_turn = get_parameter("ki_turn").as_double();
    cp.kd_turn = get_parameter("kd_turn").as_double();
    cp.i_limit_turn = get_parameter("i_limit_turn").as_double();
    cp.d_alpha = get_parameter("d_alpha").as_double();
    cp.max_move = get_parameter("max_move").as_double();
    cp.max_turn = get_parameter("max_turn").as_double();
    cp.tol_dist = get_parameter("tol_dist").as_double();
    cp.tol_yaw = get_parameter("tol_yaw_deg").as_double() * M_PI / 180.0;
    cone_radius_ = get_parameter("cone_radius").as_double();
    replan_period_ = get_parameter("replan_period").as_double();
    for (auto & o : obstacles_) {
      o.radius = cone_radius_;
    }

    follower_ = std::make_unique<rn::PathFollower>(
      cp, static_cast<int>(get_parameter("carrot_points").as_int()));
    planner_ = std::make_unique<rn::GridPlanner>(
      -field_l_ / 2, field_l_ / 2, -field_w_ / 2, field_w_ / 2, res_,
      get_parameter("robot_radius").as_double(), get_parameter("margin").as_double());
  }

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
  {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  // ---------------- 받기 ----------------
  void onGoal(const geometry_msgs::msg::Pose2D & m)
  {
    goal_ = rn::Pose2D{m.x, m.y, m.theta};
    need_replan_ = true;
    follower_->reset();   // 새 목표: PID에 쌓인 값(I)과 이전 오차(D)를 지움
    RCLCPP_INFO(
      get_logger(), "새 목표: (%.2f, %.2f), %.0f도", m.x, m.y, m.theta * 180.0 / M_PI);
  }

  void onObstacles(const geometry_msgs::msg::PoseArray & m)
  {
    std::vector<rn::Obstacle> fresh;
    for (const auto & ps : m.poses) {
      fresh.push_back({ps.position.x, ps.position.y, cone_radius_});
    }
    // 꼬깔이 생기거나 10cm 이상 움직이면 경로를 다시 짠다
    bool changed = fresh.size() != obstacles_.size();
    for (size_t k = 0; !changed && k < fresh.size(); ++k) {
      changed = std::hypot(fresh[k].x - obstacles_[k].x, fresh[k].y - obstacles_[k].y) > 0.10;
    }
    if (changed) {
      need_replan_ = true;
    }
    obstacles_ = fresh;
  }

  // ---------------- 경로 계획 ----------------
  void replan()
  {
    planner_->setObstacles(obstacles_);
    has_path_ = planner_->plan({robot_->x, robot_->y}, {goal_->x, goal_->y}, path_);
    last_plan_time_ = now();
    need_replan_ = false;
    if (!has_path_) {
      // 같은 경고가 계속 쌓이지 않게 5초에 한 번만 출력
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "경로를 찾지 못함 (목표가 꼬깔 범위 안이거나 길이 막힘)");
    }
    publishPath();
  }

  bool timeToReplan()
  {
    if (need_replan_ || !last_plan_time_) {
      return true;
    }
    return (now() - *last_plan_time_).seconds() > replan_period_;
  }

  // ---------------- 반복 ----------------
  void loop()
  {
    geometry_msgs::msg::Twist cmd;   // 기본값 = 정지

    if (!robot_) {
      publish(cmd, "NO_POSE");       // 위치를 모르면 걷지 않는다
      return;
    }
    if (!goal_) {
      publish(cmd, "IDLE");
      publishDebug(nullptr);
      return;
    }
    if (timeToReplan()) {
      replan();
    }
    if (!has_path_) {
      publish(cmd, "BLOCKED");       // 갈 길이 없으면 멈추고 알린다
      publishDebug(nullptr);
      return;
    }

    // 지난 계산 후 흐른 시간 (PID의 I, D 계산에 필요)
    const rclcpp::Time t = now();
    double dt = last_step_time_ ? (t - *last_step_time_).seconds() : 0.0;
    if (dt <= 0.0 || dt > 0.5) {
      dt = 1.0 / rate_;   // 처음이거나 너무 오래 멈췄으면 기본 주기로
    }
    last_step_time_ = t;

    const rn::Command c = follower_->step(*robot_, path_, *goal_, dt);
    cmd.linear.x = c.front;
    cmd.linear.y = c.left;
    cmd.angular.z = c.turn;
    remain_ = c.remain;
    publish(cmd, c.arrived ? "ARRIVED" : "MOVING");
    publishDebug(c.arrived ? nullptr : &c.carrot);
  }

  void publish(const geometry_msgs::msg::Twist & cmd, const std::string & status)
  {
    cmd_pub_->publish(cmd);
    std_msgs::msg::String s;
    s.data = status;
    status_pub_->publish(s);
    if (status != last_status_) {    // 상태가 바뀔 때만 화면에 출력
      RCLCPP_INFO(get_logger(), "상태: %s", status.c_str());
      last_status_ = status;
    }
  }

  // ---------------- RViz2 표시 ----------------
  void publishPath()
  {
    nav_msgs::msg::Path msg;
    msg.header.frame_id = FRAME;
    msg.header.stamp = now();
    if (has_path_) {
      for (const auto & p : path_) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = msg.header;
        ps.pose.position.x = p.x;
        ps.pose.position.y = p.y;
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
      }
    }
    path_pub_->publish(msg);
  }

  void publishDebug(const rn::Point2D * carrot)
  {
    visualization_msgs::msg::MarkerArray arr;
    const auto stamp = now();

    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = FRAME;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    // 부풀린 꼬깔 범위 (연한 주황 원판) = 로봇 중심이 들어가면 안 되는 곳
    int id = 0;
    for (const auto & o : obstacles_) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = FRAME;
      m.header.stamp = stamp;
      m.ns = "inflated";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = o.x;
      m.pose.position.y = o.y;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = 2.0 * planner_->inflatedRadius(o.radius);
      m.scale.z = 0.01;
      m.color.r = 1.0f;
      m.color.g = 0.5f;
      m.color.a = 0.25f;
      arr.markers.push_back(m);
    }

    // 당근 (노란 공)
    if (carrot != nullptr) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = FRAME;
      m.header.stamp = stamp;
      m.ns = "carrot";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::SPHERE;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = carrot->x;
      m.pose.position.y = carrot->y;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = 0.12;
      m.color.r = 1.0f;
      m.color.g = 0.9f;
      m.color.a = 1.0f;
      arr.markers.push_back(m);
    }

    // 목표 (빨간 화살표 = 도착 후 바라볼 방향)
    if (goal_) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = FRAME;
      m.header.stamp = stamp;
      m.ns = "goal";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = goal_->x;
      m.pose.position.y = goal_->y;
      m.pose.orientation.z = std::sin(goal_->yaw / 2.0);
      m.pose.orientation.w = std::cos(goal_->yaw / 2.0);
      m.scale.x = 0.4;
      m.scale.y = m.scale.z = 0.12;
      m.color.r = 0.9f;
      m.color.g = 0.1f;
      m.color.b = 0.1f;
      m.color.a = 1.0f;
      arr.markers.push_back(m);
    }

    // 상태 글자 (로봇 위에 표시): 예) MOVING 2.35m
    if (robot_) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = FRAME;
      m.header.stamp = stamp;
      m.ns = "status";
      m.id = 0;
      m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = robot_->x;
      m.pose.position.y = robot_->y + 0.45;
      m.pose.position.z = 0.5;
      m.pose.orientation.w = 1.0;
      m.scale.z = 0.25;   // 글자 높이 (m)
      char buf[64];
      if (last_status_ == "MOVING") {
        std::snprintf(buf, sizeof(buf), "MOVING %.2fm", remain_);
      } else {
        std::snprintf(buf, sizeof(buf), "%s", last_status_.c_str());
      }
      m.text = buf;
      // 상태별 색: 이동 중 흰색, 도착 초록, 막힘 빨강
      m.color.r = (last_status_ == "ARRIVED") ? 0.3f : 1.0f;
      m.color.g = (last_status_ == "BLOCKED") ? 0.3f : 1.0f;
      m.color.b = (last_status_ == "MOVING" || last_status_ == "IDLE") ? 1.0f : 0.3f;
      m.color.a = 1.0f;
      arr.markers.push_back(m);
    }
    debug_pub_->publish(arr);
  }

  // 계산 도구
  std::unique_ptr<rn::PathFollower> follower_;
  double rate_{10.0};
  std::optional<rclcpp::Time> last_step_time_;
  std::unique_ptr<rn::GridPlanner> planner_;
  double cone_radius_{0.1};
  double replan_period_{1.0};
  double field_l_{9.0}, field_w_{6.0}, res_{0.1};
  double remain_{0.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_handle_;

  // 상태
  std::optional<rn::Pose2D> robot_;   // 아직 위치를 못 받았으면 비어 있음
  std::optional<rn::Pose2D> goal_;    // 아직 목표를 못 받았으면 비어 있음
  std::vector<rn::Obstacle> obstacles_;
  std::vector<rn::Point2D> path_;
  bool has_path_{false};
  bool need_replan_{false};
  std::optional<rclcpp::Time> last_plan_time_;
  std::string last_status_;

  // ROS2 연결
  rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_sub_, goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr click_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}