// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Hybrid A* — 에너지 인식 2.5D 하이브리드 Nav2 글로벌 플래너
//
// 지상 주행과 비행을 함께 계획하고, 모드 전환 지점을 ModeSwitchPlan 으로
// 퍼블리시한다. drobot_mode_manager 가 그걸 받아 PX4 이착륙을 명령한다.
//
// 알고리즘 선택 근거: benchmark/RESEARCH_LOG.md
//   3D 상태공간에서 A*가 2초 제약 안에 최적해를 낸다 (0.12~1.00초).
//   RRT*는 같은 시간에 근사해이고 실행마다 결과가 다르다.
//   해 품질 차이는 1% 안팎으로 작지만, 최적성 보장 / 결정론적 재현성 /
//   튜닝 부담이 적다는 점에서 A*를 택했다.
//
// 구현은 benchmark/planners/grid_search.py + smoothing.py 를 옮긴 것이다.

#ifndef DROBOT_HYBRID_PLANNER__HYBRID_ASTAR_PLANNER_HPP_
#define DROBOT_HYBRID_PLANNER__HYBRID_ASTAR_PLANNER_HPP_

// cpplint 규칙상 C 헤더(.h)가 C++ 헤더(.hpp)보다 앞에 와야 한다.
#include <tf2_ros/buffer.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/header.hpp>

#include <drobot_msgs/msg/mode_switch_plan.hpp>
#include <drobot_msgs/msg/mode_switch_point.hpp>

#include "drobot_hybrid_planner/energy_model.hpp"
#include "drobot_hybrid_planner/state_space.hpp"

namespace drobot_hybrid_planner
{

/// 실좌표 경로점. 스무딩과 ModeSwitchPlan 생성에 쓴다.
struct Waypoint
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;     ///< 지면 기준 절대 고도. GROUND면 0
  uint8_t mode = GROUND;
};

/// 꺾임점(waypoint) 경로를 컨트롤러가 쓰는 nav_msgs/Path 로 바꾼다.
///
/// 왜 촘촘하게 하나:
///     스무딩 결과는 꺾이는 점 몇 개(몇 m 간격)뿐이다. Nav2 컨트롤러(DWB 의
///     PathDist·PathAlign, RotationShim)는 경로가 격자 간격으로 이어져 있다고 보고
///     '경로까지의 거리' 와 '경로 방향' 을 잰다. 점이 듬성하면 점 사이 구간이
///     경로로 안 잡혀 로봇이 제자리에서 좌우로 흔들렸다 (205 s 동안 13 바퀴,
///     회전만 76 %, 전진 3 m). 그래서 step 간격으로 보간한다.
/// 왜 방향을 넣나:
///     예전에는 모든 점이 orientation.w = 1 (yaw 0, +x) 이었다. 진행 방향과
///     무관한 값이라 방향 정렬 critic 이 엉뚱한 쪽을 가리켰다.
///
/// 이착륙처럼 같은 (x, y) 에서 고도만 바뀌는 점은 보간하지 않고 그대로 둔다.
/// 그 점의 yaw 는 직전 진행 방향을 잇는다.
nav_msgs::msg::Path densifyPath(
  const std::vector<Waypoint> & wps, double step, const std_msgs::msg::Header & header);


class HybridAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  HybridAStarPlanner() = default;
  ~HybridAStarPlanner() override = default;

  // ---- nav2_core::GlobalPlanner 인터페이스 ---------------------------
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  // Nav2 Jazzy 부터 cancel_checker 인자가 추가됐다.
  // 장시간 탐색 중 취소 요청을 받을 수 있게 하기 위한 것으로,
  // 여기서도 주기적으로 확인해 조기 종료한다.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  // ---- 탐색 ----------------------------------------------------------
  struct SearchResult
  {
    bool found = false;
    std::vector<State> path;
    double cost = 0.0;
    CostAccumulator acc;
    size_t n_expanded = 0;
    bool timed_out = false;
  };

  SearchResult search(
    const State & start, const State & goal,
    const std::function<bool()> & cancel_checker) const;

  // ---- 후처리 --------------------------------------------------------
  /// 격자 경로를 실좌표 경로로 변환
  std::vector<Waypoint> toWaypoints(const std::vector<State> & path) const;

  /// shortcut smoothing.
  /// 격자 A*는 이동 각도가 45도 배수로 제한되므로 후처리가 필요하다.
  /// 스무딩 없이는 연속 공간 플래너 대비 약 5% 손해를 본다.
  /// 모드 전환 지점은 건너뛰지 않는다 — 이착륙은 '같은 위치에서만'
  /// 허용되는 전이라 잘라내면 물리적으로 불가능한 경로가 된다.
  std::vector<Waypoint> smooth(const std::vector<Waypoint> & path) const;

  /// 두 점 사이 직선 이동이 가능한가 (같은 모드 가정)
  bool segmentOk(const Waypoint & a, const Waypoint & b) const;
  /// 한 구간의 비용. 불가능하면 false
  bool segmentCost(const Waypoint & a, const Waypoint & b, CostAccumulator & out) const;

  /// 지상 구간의 비용. rover_climb 이면 지나가는 지형의 총 상승량을 반영한다.
  CostAccumulator groundSegmentCost(
    const Waypoint & a, const Waypoint & b, double d) const;

  // ---- 출력 ----------------------------------------------------------
  nav_msgs::msg::Path toPathMsg(
    const std::vector<Waypoint> & wps, const std_msgs::msg::Header & header) const;

  /// 모드 전환 지점을 뽑아 ModeSwitchPlan 으로 만든다.
  /// 이륙-착륙 쌍에 같은 pair_id 를 부여해 BT 에서 매칭할 수 있게 한다.
  drobot_msgs::msg::ModeSwitchPlan buildSwitchPlan(
    const std::vector<Waypoint> & wps, const std_msgs::msg::Header & header) const;

  // ---- 멤버 ----------------------------------------------------------
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  std::string name_;
  std::string modal_name_ = "hybrid";   ///< 로그 표시용
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("HybridAStarPlanner")};

  EnergyModel energy_;
  std::shared_ptr<TerrainSource> terrain_;
  /// terrain_ 이 LayerTerrainSource 일 때 같은 객체 (출발점 완화용). 아니면 nullptr
  std::shared_ptr<LayerTerrainSource> layer_terrain_;
  std::string terrain_layer_ = "elevation_layer";   ///< 지형 등급을 읽을 costmap 레이어 이름
  std::unique_ptr<ProblemSpec> spec_;
  ProblemSpec::Params spec_params_;

  // 파라미터
  double timeout_s_ = 2.0;
  int max_expansions_ = 2000000;
  bool smooth_path_ = true;
  int smoothing_max_passes_ = 30;
  bool publish_switch_plan_ = true;
  double start_relax_radius_ = 0.35;   ///< m — 출발점 주변 INSCRIBED 를 막지 않는 반경
  double goal_tolerance_ = 0.5;        ///< m — 목표 칸이 막혔을 때 대신 쓸 칸을 찾는 반경

  rclcpp_lifecycle::LifecyclePublisher<drobot_msgs::msg::ModeSwitchPlan>::SharedPtr
    switch_plan_pub_;
};

}  // namespace drobot_hybrid_planner

#endif  // DROBOT_HYBRID_PLANNER__HYBRID_ASTAR_PLANNER_HPP_
