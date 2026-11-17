// 가짜 로봇 노드 (연습용, 내 담당 아님)
// - 실제로는 Walking(걷기) + Localization(위치 추정) + Vision(꼬깔 찾기)이 하는 일을 흉내만 낸다.
// - /nav_cmd 를 받아서 그만큼 움직였다고 치고, /robot_pose 로 새 위치를 보낸다.
// - 가짜 꼬깔 위치를 /obstacles 로 보내고, 꼬깔에 닿으면 경고를 띄운다.
// - RViz2 화면에 필드, 로봇, 지나온 길, 꼬깔을 그린다.
// - RViz2 "2D Pose Estimate" 버튼으로 찍은 곳으로 로봇을 옮긴다 (Reset 역할).
// - slip 파라미터로 걸을 때 뒤로 미끄러지는 외란을 흉내 낼 수 있다 (PID 실험용).
//
// 받는 토픽:  /nav_cmd (Twist), /initialpose (PoseWithCovarianceStamped, RViz2 버튼)
// 보내는 토픽: /robot_pose (Pose2D), /obstacles (PoseArray), /nav_markers (MarkerArray)

#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using visualization_msgs::msg::Marker;

static const char * FRAME = "map";   // 필드 좌표계 이름 (RViz2의 Fixed Frame과 같아야 함)

static geometry_msgs::msg::Point makePoint(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

class FakeRobotNode : public rclcpp::Node
{
public:
  FakeRobotNode()
  : Node("fake_robot_node")
  {
    x_ = declare_parameter("start_x", -3.0);
    y_ = declare_parameter("start_y", -3.0);
    const double start_yaw_deg = declare_parameter("start_yaw_deg", 90.0);
    yaw_ = start_yaw_deg * M_PI / 180.0;
    const double rate = declare_parameter("rate", 20.0);
    dt_ = 1.0 / rate;
    field_l_ = declare_parameter("field_length", 9.0);   // 필드 크기는 예시 값
    field_w_ = declare_parameter("field_width", 6.0);
    // 가짜 꼬깔 위치 (필드 기준, m). cones_x[k], cones_y[k]가 한 꼬깔
    const auto cx = declare_parameter("cones_x", std::vector<double>{-1.0, 1.0, 2.2});
    const auto cy = declare_parameter("cones_y", std::vector<double>{-1.2, 0.0, -0.5});
    for (size_t k = 0; k < std::min(cx.size(), cy.size()); ++k) {
      cones_.emplace_back(cx[k], cy[k]);
    }
    cone_r_ = declare_parameter("cone_radius", 0.1);
    robot_r_ = declare_parameter("robot_radius", 0.2);
    // 미끄러짐(외란): 걷는 동안 로봇이 뒤로 밀리는 속도 (m/s). 0이면 미끄러짐 없음
    // 실행 중에도 변경 가능: ros2 param set /fake_robot_node slip 0.1
    declare_parameter("slip", 0.0);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "nav_cmd", 10, [this](geometry_msgs::msg::Twist::SharedPtr m) {cmd_ = *m;});
    // RViz2 "2D Pose Estimate": 찍은 위치·방향으로 로봇을 옮기고 지나온 길을 지운다
    initpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10, [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m) {
        const auto & q = m->pose.pose.orientation;
        x_ = m->pose.pose.position.x;
        y_ = m->pose.pose.position.y;
        yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        cmd_ = geometry_msgs::msg::Twist();
        trail_.clear();
        RCLCPP_INFO(
          get_logger(), "로봇 위치 옮김: (%.2f, %.2f), %.0f도", x_, y_, yaw_ * 180.0 / M_PI);
      });
    pose_pub_ = create_publisher<geometry_msgs::msg::Pose2D>("robot_pose", 10);
    obstacle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("obstacles", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("nav_markers", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this]() {loop();});

    RCLCPP_INFO(get_logger(), "가짜 로봇 시작: (%.1f, %.1f), %.0f도", x_, y_, start_yaw_deg);
  }

private:
  void loop()
  {
    // 걷는 중이면 미끄러짐만큼 앞으로 덜 감 (외란)
    const bool walking = std::abs(cmd_.linear.x) + std::abs(cmd_.linear.y) > 1e-6;
    const double vx = cmd_.linear.x - (walking ? get_parameter("slip").as_double() : 0.0);

    // 로봇 기준 속도 명령을 필드 기준 이동으로 바꿔서 위치를 더한다 (좌표 변환의 반대 방향)
    const double c = std::cos(yaw_), s = std::sin(yaw_);
    x_ += (c * vx - s * cmd_.linear.y) * dt_;
    y_ += (s * vx + c * cmd_.linear.y) * dt_;
    yaw_ += cmd_.angular.z * dt_;
    yaw_ = std::atan2(std::sin(yaw_), std::cos(yaw_));   // -π ~ π 유지

    geometry_msgs::msg::Pose2D pose;
    pose.x = x_;
    pose.y = y_;
    pose.theta = yaw_;
    pose_pub_->publish(pose);

    // 가짜 Vision: 꼬깔 위치 보내기
    geometry_msgs::msg::PoseArray obs;
    obs.header.frame_id = FRAME;
    obs.header.stamp = now();
    for (const auto & cone : cones_) {
      geometry_msgs::msg::Pose ps;
      ps.position.x = cone.first;
      ps.position.y = cone.second;
      ps.orientation.w = 1.0;
      obs.poses.push_back(ps);
    }
    obstacle_pub_->publish(obs);

    // 꼬깔에 닿았는지 검사 (규칙: 닿으면 세팅 위치로)
    bool touching = false;
    for (const auto & cone : cones_) {
      touching = touching ||
        std::hypot(x_ - cone.first, y_ - cone.second) < cone_r_ + robot_r_;
    }
    if (touching && !touching_) {
      RCLCPP_WARN(get_logger(), "꼬깔에 닿음!");
    }
    touching_ = touching;

    // 지나온 길 저장 (2cm 이상 움직였을 때만)
    if (trail_.empty() ||
      std::hypot(x_ - trail_.back().first, y_ - trail_.back().second) > 0.02)
    {
      trail_.emplace_back(x_, y_);
      if (trail_.size() > 2000) {
        trail_.erase(trail_.begin());
      }
    }
    publishMarkers();
  }

  // ---------------- RViz2 화면 그리기 ----------------
  Marker makeMarker(int id, int type, float r, float g, float b, float a = 1.0f)
  {
    Marker m;
    m.header.frame_id = FRAME;
    m.header.stamp = now();
    m.ns = "nav";
    m.id = id;
    m.type = type;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    return m;
  }

  static void setYaw(Marker & m, double yaw)
  {
    m.pose.orientation.z = std::sin(yaw / 2.0);
    m.pose.orientation.w = std::cos(yaw / 2.0);
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    const double hl = field_l_ / 2.0, hw = field_w_ / 2.0;

    // 0: 필드 바깥선 + 센터라인
    Marker field = makeMarker(0, Marker::LINE_LIST, 1.0f, 1.0f, 1.0f);
    field.scale.x = 0.03;
    const double corners[4][2] = {{-hl, -hw}, {hl, -hw}, {hl, hw}, {-hl, hw}};
    for (int i = 0; i < 4; ++i) {
      field.points.push_back(makePoint(corners[i][0], corners[i][1]));
      field.points.push_back(makePoint(corners[(i + 1) % 4][0], corners[(i + 1) % 4][1]));
    }
    field.points.push_back(makePoint(0.0, -hw));
    field.points.push_back(makePoint(0.0, hw));
    arr.markers.push_back(field);

    // 1: 로봇 (초록 화살표 = 바라보는 방향)
    Marker robot = makeMarker(1, Marker::ARROW, 0.1f, 0.8f, 0.2f);
    robot.pose.position.x = x_;
    robot.pose.position.y = y_;
    setYaw(robot, yaw_);
    robot.scale.x = 0.4;
    robot.scale.y = robot.scale.z = 0.12;
    arr.markers.push_back(robot);

    // 2: 지나온 길
    if (trail_.size() >= 2) {
      Marker trail = makeMarker(2, Marker::LINE_STRIP, 0.1f, 0.8f, 0.2f, 0.6f);
      trail.scale.x = 0.02;
      for (const auto & t : trail_) {
        trail.points.push_back(makePoint(t.first, t.second));
      }
      arr.markers.push_back(trail);
    }

    // 10~: 꼬깔 (주황 원기둥)
    int id = 10;
    for (const auto & cone : cones_) {
      Marker m = makeMarker(id++, Marker::CYLINDER, 1.0f, 0.4f, 0.0f);
      m.pose.position.x = cone.first;
      m.pose.position.y = cone.second;
      m.pose.position.z = 0.15;
      m.scale.x = m.scale.y = 2.0 * cone_r_;
      m.scale.z = 0.3;
      arr.markers.push_back(m);
    }

    marker_pub_->publish(arr);
  }

  double x_, y_, yaw_, dt_;
  double field_l_, field_w_;
  double cone_r_, robot_r_;
  std::vector<std::pair<double, double>> cones_;
  std::vector<std::pair<double, double>> trail_;
  bool touching_{false};
  geometry_msgs::msg::Twist cmd_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initpose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeRobotNode>());
  rclcpp::shutdown();
  return 0;
}