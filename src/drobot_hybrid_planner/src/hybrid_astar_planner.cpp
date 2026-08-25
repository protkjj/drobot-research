// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "drobot_hybrid_planner/hybrid_astar_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <string>
#include <unordered_map>

#include <nav2_util/node_utils.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace drobot_hybrid_planner
{

// using-directive 대신 using-declaration 을 쓴다 (cpplint build/namespaces).
using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::steady_clock;

namespace
{
double getD(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & n, double v)
{
  nav2_util::declare_parameter_if_not_declared(node, n, rclcpp::ParameterValue(v));
  return node->get_parameter(n).as_double();
}
int getI(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & n, int v)
{
  nav2_util::declare_parameter_if_not_declared(node, n, rclcpp::ParameterValue(v));
  return node->get_parameter(n).as_int();
}
bool getB(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & n, bool v)
{
  nav2_util::declare_parameter_if_not_declared(node, n, rclcpp::ParameterValue(v));
  return node->get_parameter(n).as_bool();
}
std::string getS(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & n, const std::string & v)
{
  nav2_util::declare_parameter_if_not_declared(node, n, rclcpp::ParameterValue(v));
  return node->get_parameter(n).as_string();
}
}  // namespace


// ---------------------------------------------------------------------------
// 생명주기
// ---------------------------------------------------------------------------
void HybridAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  logger_ = node->get_logger();

  const std::string p = name_ + ".";

  timeout_s_ = getD(node, p + "timeout", timeout_s_);
  max_expansions_ = getI(node, p + "max_expansions", max_expansions_);
  smooth_path_ = getB(node, p + "smooth_path", smooth_path_);
  smoothing_max_passes_ = getI(node, p + "smoothing_max_passes", smoothing_max_passes_);
  publish_switch_plan_ = getB(node, p + "publish_mode_switch_plan", publish_switch_plan_);

  spec_params_.flight_clearance = getD(node, p + "flight_clearance", 0.8);
  spec_params_.allow_diagonal = getB(node, p + "allow_diagonal", true);
  spec_params_.check_diagonal_corners = getB(node, p + "check_diagonal_corners", true);

  // 천장 높이와 로버 통과 높이는 costmap 쪽 설정과 맞춰야 한다.
  // elevation_params.yaml 의 값과 동일하게 두는 것을 전제로 한다.
  spec_params_.ceiling_height = getD(node, p + "ceiling_height", 2.5);
  spec_params_.rover_max_height = getD(node, p + "rover_traversable_max", 0.15);

  // 이동 방식(modal). 세 modal 을 같은 비용 함수로 비교하기 위해
  // 파라미터 하나로 갈아끼울 수 있게 둔다.
  //   rover_detour  우회      장애물을 피해 돌아간다
  //   rover_climb   밟고넘기   climb_mode.max_height 이하 장애물은 타고 넘는다
  //   hybrid        하이브리드  드론으로 전환해 날아 넘는다
  modal_name_ = getS(node, p + "modal", modal_name_);
  if (modal_name_ == "rover_detour") {
    spec_params_.modal = Modal::RoverDetour;
  } else if (modal_name_ == "rover_climb") {
    spec_params_.modal = Modal::RoverClimb;
  } else if (modal_name_ == "hybrid") {
    spec_params_.modal = Modal::Hybrid;
  } else {
    RCLCPP_WARN(
      logger_,
      "알 수 없는 modal '%s' — hybrid 로 처리한다. "
      "가능한 값: rover_detour / rover_climb / hybrid",
      modal_name_.c_str());
    spec_params_.modal = Modal::Hybrid;
    modal_name_ = "hybrid";
  }

  // 에너지 모델은 energy_model 네임스페이스 아래에서 읽는다
  energy_.configure(node, "energy_model");

  // 지형 소스 — Phase 1 은 costmap cost 값에서 등급을 읽는다 (state_space.hpp 주석 참고)
  CostmapTerrainSource::Config tcfg;
  tcfg.h_rover = spec_params_.rover_max_height;
  tcfg.h_flyover = getD(node, p + "flyover_representative_height", 0.60);
  terrain_ = std::make_shared<CostmapTerrainSource>(costmap_, tcfg);

  spec_ = std::make_unique<ProblemSpec>(costmap_, terrain_, &energy_, spec_params_);

  if (publish_switch_plan_) {
    switch_plan_pub_ = node->create_publisher<drobot_msgs::msg::ModeSwitchPlan>(
      "mode_switch_points", rclcpp::QoS(1).transient_local());
  }

  RCLCPP_INFO(
    logger_,
    "HybridAStarPlanner '%s' 설정 완료: modal=%s, timeout=%.2fs, "
    "flight_clearance=%.2fm, z_max=%.2fm, 통과가능 h<=%.2fm, "
    "로버 통과높이<=%.2fm, corner_check=%s",
    name_.c_str(), modal_name_.c_str(), timeout_s_, spec_params_.flight_clearance,
    spec_->zMax(), spec_->hLimit(), spec_->roverHLimit(),
    spec_params_.check_diagonal_corners ? "on" : "off");

  // 설정 충돌 경고 — 벤치마크에서 실제로 문제가 됐던 조합
  if (spec_->hLimit() <= spec_params_.rover_max_height) {
    RCLCPP_WARN(
      logger_,
      "천장(%.2fm) 대비 flight_clearance(%.2fm)가 커서 비행으로 넘을 수 있는 "
      "장애물이 없다. 사실상 지상 전용 플래너가 된다.",
      spec_params_.ceiling_height, spec_params_.flight_clearance);
  }
}

void HybridAStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "HybridAStarPlanner '%s' 정리", name_.c_str());
  spec_.reset();
  terrain_.reset();
  switch_plan_pub_.reset();
}

void HybridAStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "HybridAStarPlanner '%s' 활성화", name_.c_str());
  if (switch_plan_pub_) {switch_plan_pub_->on_activate();}
}

void HybridAStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "HybridAStarPlanner '%s' 비활성화", name_.c_str());
  if (switch_plan_pub_) {switch_plan_pub_->on_deactivate();}
}


// ---------------------------------------------------------------------------
// A* 탐색
// ---------------------------------------------------------------------------
HybridAStarPlanner::SearchResult HybridAStarPlanner::search(
  const State & start, const State & goal,
  const std::function<bool()> & cancel_checker) const
{
  SearchResult result;
  const auto t0 = steady_clock::now();

  // g[s] = 시작점에서 s까지의 최소 비용
  std::unordered_map<size_t, double> g;
  std::unordered_map<size_t, CostAccumulator> g_acc;
  std::unordered_map<size_t, State> parent;
  std::vector<bool> closed(spec_->numStates(), false);

  const size_t start_idx = spec_->index(start);
  g[start_idx] = 0.0;
  g_acc[start_idx] = CostAccumulator{};

  // (f, tie, state) — tie는 동률 시 결정론적 순서를 보장하기 위한 카운터.
  // 이게 없으면 같은 f를 가진 상태들의 처리 순서가 불안정해져
  // "결정론적 재현성"이라는 A*의 장점이 깨진다.
  struct Item
  {
    double f;
    uint64_t tie;
    State s;
    bool operator>(const Item & o) const
    {
      return f > o.f || (f == o.f && tie > o.tie);
    }
  };
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  uint64_t counter = 0;
  open.push({spec_->heuristic(start, goal), counter++, start});

  std::vector<ProblemSpec::Edge> edges;
  edges.reserve(16);

  while (!open.empty()) {
    // 타임아웃/취소 확인은 매 반복이 아니라 주기적으로 (호출 자체도 비용)
    if ((result.n_expanded & 0x3FF) == 0) {
      const double elapsed =
        duration_cast<duration<double>>(steady_clock::now() - t0).count();
      if (elapsed > timeout_s_) {
        result.timed_out = true;
        break;
      }
      if (cancel_checker && cancel_checker()) {
        RCLCPP_WARN(logger_, "경로계획이 취소됐다 (%zu 노드 확장 후)", result.n_expanded);
        break;
      }
    }
    if (static_cast<int>(result.n_expanded) >= max_expansions_) {
      result.timed_out = true;
      break;
    }

    const Item top = open.top();
    open.pop();
    const size_t idx = spec_->index(top.s);
    if (closed[idx]) {continue;}
    closed[idx] = true;
    ++result.n_expanded;

    if (top.s == goal) {
      // 경로 복원
      State cur = top.s;
      result.path.push_back(cur);
      while (true) {
        auto it = parent.find(spec_->index(cur));
        if (it == parent.end()) {break;}
        cur = it->second;
        result.path.push_back(cur);
      }
      std::reverse(result.path.begin(), result.path.end());
      result.found = true;
      result.cost = g[idx];
      result.acc = g_acc[idx];
      return result;
    }

    spec_->neighbors(top.s, edges);
    for (const auto & e : edges) {
      const size_t nidx = spec_->index(e.next);
      if (closed[nidx]) {continue;}

      const CostAccumulator new_acc = g_acc[idx] + e.acc;
      const double new_g = energy_.cost(new_acc);

      auto it = g.find(nidx);
      if (it == g.end() || new_g < it->second - 1e-12) {
        g[nidx] = new_g;
        g_acc[nidx] = new_acc;
        parent[nidx] = top.s;
        open.push({new_g + spec_->heuristic(e.next, goal), counter++, e.next});
      }
    }
  }

  return result;
}


// ---------------------------------------------------------------------------
// 후처리
// ---------------------------------------------------------------------------
std::vector<Waypoint> HybridAStarPlanner::toWaypoints(const std::vector<State> & path) const
{
  std::vector<Waypoint> out;
  out.reserve(path.size());
  for (const auto & s : path) {
    double wx, wy;
    costmap_->mapToWorld(s.mx, s.my, wx, wy);
    Waypoint w;
    w.x = wx;
    w.y = wy;
    w.mode = s.mode;
    // 고도가 지형을 따라가지 않고 비행 구간 내내 고정이므로
    // 셀이 아니라 level 로 조회한다.
    w.z = (s.mode == AIR) ? spec_->levelZ(s.level) : 0.0;
    out.push_back(w);
  }
  return out;
}


bool HybridAStarPlanner::segmentOk(const Waypoint & a, const Waypoint & b) const
{
  if (a.mode != b.mode) {return false;}

  const double res = costmap_->getResolution();
  const double dist = std::hypot(b.x - a.x, b.y - a.y);
  // 보간 간격을 셀 크기의 절반 이하로. 성기면 두 샘플 사이의 얇은 벽을
  // 그냥 지나쳐 버린다 (corner-cutting과 같은 종류의 함정).
  const int n = std::max(2, static_cast<int>(dist / (res * 0.4)) + 1);

  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    const double x = a.x + (b.x - a.x) * t;
    const double y = a.y + (b.y - a.y) * t;
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my)) {return false;}

    if (a.mode == GROUND) {
      if (!spec_->groundOk(mx, my)) {return false;}
    } else {
      const double z = a.z + (b.z - a.z) * t;
      const double h = spec_->terrain(mx, my);
      if (z < h + energy_.minFlightClearance() - 1e-9) {return false;}
      if (z > spec_->zMax() + 1e-9) {return false;}
    }
  }
  return true;
}


bool HybridAStarPlanner::segmentCost(
  const Waypoint & a, const Waypoint & b, CostAccumulator & out) const
{
  const double d = std::hypot(b.x - a.x, b.y - a.y);

  // 모드 전환 — 같은 위치에서만
  if (a.mode != b.mode) {
    if (d > 1e-6) {return false;}
    out = (b.mode == AIR) ? energy_.takeoff(b.z) : energy_.landing(a.z);
    return true;
  }

  if (!segmentOk(a, b)) {return false;}

  if (a.mode == GROUND) {
    out = groundSegmentCost(a, b, d);
    return true;
  }

  // 비행 구간은 고도가 고정이라 두 끝점의 z 가 같아야 한다.
  // 다르면 서로 다른 비행 구간의 점을 이으려는 것이므로 허용하지 않는다.
  // (고도를 바꾸려면 착륙 후 재이륙해야 하고, 그 지점은 모드 전환으로 보존된다)
  if (std::fabs(b.z - a.z) > 1e-9) {return false;}

  unsigned int amx, amy, bmx, bmy;
  if (!costmap_->worldToMap(a.x, a.y, amx, amy)) {return false;}
  if (!costmap_->worldToMap(b.x, b.y, bmx, bmy)) {return false;}
  const double clr = std::min(
    a.z - spec_->terrain(amx, amy), b.z - spec_->terrain(bmx, bmy));

  out = energy_.airMoveHorizontal(d, clr);
  return true;
}


CostAccumulator HybridAStarPlanner::groundSegmentCost(
  const Waypoint & a, const Waypoint & b, double d) const
{
  if (!spec_->allowClimb()) {
    return energy_.groundMove(d);
  }

  // rover_climb 에서는 직선이 장애물 위를 지날 수 있다.
  // 평지 주행 비용만 매기면 '밟고 넘기'가 공짜가 되어 부당하게 싸진다.
  // ProblemSpec::groundEdge 와 같은 규칙(양의 높이차에만 부과)을
  // 직선 구간에도 적용해야 격자 경로와 스무딩 경로의 비용이 일관된다.
  //
  // 격자 해상도 간격으로 샘플링해 셀 단위 이동을 흉내낸다.
  // 지형이 셀 단위 계단 함수라 이 간격이면 상승 지점을 놓치지 않는다.
  const double res = costmap_->getResolution();
  const int n = std::max(1, static_cast<int>(std::lround(d / res)));
  const double step = d / n;

  CostAccumulator total;
  unsigned int mx, my;
  double prev_h = 0.0;
  if (costmap_->worldToMap(a.x, a.y, mx, my)) {prev_h = spec_->terrain(mx, my);}

  for (int i = 1; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    const double x = a.x + (b.x - a.x) * t;
    const double y = a.y + (b.y - a.y) * t;
    double h = prev_h;
    if (costmap_->worldToMap(x, y, mx, my)) {h = spec_->terrain(mx, my);}
    total += energy_.roverClimbMove(step, h - prev_h);
    prev_h = h;
  }
  return total;
}


std::vector<Waypoint> HybridAStarPlanner::smooth(const std::vector<Waypoint> & path) const
{
  if (path.size() < 3) {return path;}

  // 성능 주의:
  //   예전 구현은 pass 마다 모든 (i, j) 쌍을 보고, 각 쌍마다 i..j 구간 비용을
  //   다시 합산했다. 즉 O(passes * n^3) 이고 segmentCost 안에는 충돌검사
  //   보간까지 들어 있다. 경로 381점에서 23.9초가 걸렸다
  //   (탐색은 381노드/0.1초인데 스무딩이 나머지를 다 썼다).
  //
  //   두 가지로 고쳤다:
  //     1) 구간 비용을 누적합(prefix sum)으로 O(1) 조회
  //     2) j 탐색 범위를 lookahead 로 제한 — 멀리 있는 두 점이 직선으로
  //        이어지는 경우는 드물고, 있어도 여러 pass 에 걸쳐 점진적으로 줄어든다
  //   결과적으로 O(passes * n * lookahead) 가 된다.
  constexpr size_t kLookahead = 40;

  std::vector<Waypoint> cur = path;

  for (int pass = 0; pass < smoothing_max_passes_; ++pass) {
    const size_t n = cur.size();
    if (n < 3) {break;}

    // prefix[k] = cur[0]..cur[k] 까지의 누적 비용.
    // 구간 i..j 비용 = prefix[j] - prefix[i] (스칼라이므로 차이로 구할 수 있다)
    std::vector<double> prefix(n, 0.0);
    bool prefix_ok = true;
    for (size_t k = 0; k + 1 < n; ++k) {
      CostAccumulator seg;
      if (!segmentCost(cur[k], cur[k + 1], seg)) {prefix_ok = false; break;}
      prefix[k + 1] = prefix[k] + energy_.cost(seg);
    }
    if (!prefix_ok) {break;}   // 경로가 이미 무효하면 더 손대지 않는다

    bool changed = false;
    std::vector<Waypoint> next;
    next.reserve(n);
    next.push_back(cur[0]);

    size_t i = 0;
    while (i + 1 < n) {
      const size_t j_max = std::min(n - 1, i + kLookahead);
      size_t best_j = i + 1;
      // 가장 멀리 있는 j 부터 본다 — 많이 자를수록 이득이 크다
      for (size_t j = j_max; j > i + 1; --j) {
        CostAccumulator direct;
        if (!segmentCost(cur[i], cur[j], direct)) {continue;}
        if (prefix[j] - prefix[i] - energy_.cost(direct) > 1e-9) {
          best_j = j;
          changed = true;
          break;
        }
      }
      next.push_back(cur[best_j]);
      i = best_j;
    }

    cur = std::move(next);
    if (!changed) {break;}
  }
  return cur;
}


// ---------------------------------------------------------------------------
// 출력
// ---------------------------------------------------------------------------
nav_msgs::msg::Path HybridAStarPlanner::toPathMsg(
  const std::vector<Waypoint> & wps, const std_msgs::msg::Header & header) const
{
  nav_msgs::msg::Path msg;
  msg.header = header;
  msg.poses.reserve(wps.size());
  for (const auto & w : wps) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = header;
    ps.pose.position.x = w.x;
    ps.pose.position.y = w.y;
    // 비행 구간의 고도를 z에 담는다. 지상 컨트롤러는 z를 무시하고,
    // mode_manager 는 ModeSwitchPlan 으로 이착륙을 판단한다.
    ps.pose.position.z = w.z;
    ps.pose.orientation.w = 1.0;
    msg.poses.push_back(ps);
  }
  return msg;
}


drobot_msgs::msg::ModeSwitchPlan HybridAStarPlanner::buildSwitchPlan(
  const std::vector<Waypoint> & wps, const std_msgs::msg::Header & header) const
{
  drobot_msgs::msg::ModeSwitchPlan plan;
  plan.header = header;

  uint32_t pair_id = 0;
  double total_flight_energy = 0.0;
  double seg_flight_energy = 0.0;
  bool in_flight = false;

  for (size_t i = 0; i + 1 < wps.size(); ++i) {
    const auto & a = wps[i];
    const auto & b = wps[i + 1];

    if (a.mode == GROUND && b.mode == AIR) {
      // 이륙
      drobot_msgs::msg::ModeSwitchPoint pt;
      pt.header = header;
      pt.position.x = a.x;
      pt.position.y = a.y;
      pt.position.z = 0.0;
      pt.switch_type = drobot_msgs::msg::ModeSwitchPoint::GROUND_TO_AIR;
      pt.flight_altitude = b.z;
      pt.estimated_energy_cost = energy_.takeoff(b.z).e_switch;
      pt.pair_id = pair_id;
      plan.switch_points.push_back(pt);

      total_flight_energy += pt.estimated_energy_cost;
      seg_flight_energy = 0.0;
      in_flight = true;

    } else if (a.mode == AIR && b.mode == GROUND) {
      // 착륙
      drobot_msgs::msg::ModeSwitchPoint pt;
      pt.header = header;
      pt.position.x = a.x;
      pt.position.y = a.y;
      pt.position.z = 0.0;
      pt.switch_type = drobot_msgs::msg::ModeSwitchPoint::AIR_TO_GROUND;
      pt.flight_altitude = a.z;
      pt.estimated_energy_cost = energy_.landing(a.z).e_switch;
      pt.pair_id = pair_id;
      plan.switch_points.push_back(pt);

      // 이 비행 구간의 수평 이동 에너지도 합산.
      //
      // 구간을 통째로 airMoveHorizontal(dist, 고도) 로 다시 계산하지
      // 않는다. 두 번째 인자는 '고도'가 아니라 '지형 상단으로부터의 여유'라
      // 값의 의미가 다르고, 그러면 ground effect 보정이 엉뚱하게 적용된다.
      // 대신 아래 AIR->AIR 분기에서 실제 구간 비용을 누적해 둔 값을 쓴다.
      total_flight_energy += pt.estimated_energy_cost;
      total_flight_energy += seg_flight_energy;

      ++pair_id;
      in_flight = false;

    } else if (a.mode == AIR && b.mode == AIR) {
      CostAccumulator seg;
      if (segmentCost(a, b, seg)) {seg_flight_energy += seg.e_air_horiz;}
    }
  }

  if (in_flight) {
    RCLCPP_WARN(
      logger_, "경로가 비행 상태로 끝난다 — 착륙 지점이 없다. pair_id %u 미완결",
      pair_id);
  }

  plan.total_flight_energy = total_flight_energy;
  return plan;
}


// ---------------------------------------------------------------------------
// createPlan
// ---------------------------------------------------------------------------
nav_msgs::msg::Path HybridAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  std_msgs::msg::Header header;
  if (auto node = node_.lock()) {
    header.stamp = node->now();
  }
  header.frame_id = global_frame_;

  nav_msgs::msg::Path empty;
  empty.header = header;

  // costmap 은 다른 스레드가 갱신하므로 탐색 동안 잠근다
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

  State s_start, s_goal;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y,
    s_start.mx, s_start.my))
  {
    RCLCPP_WARN(logger_, "시작점이 costmap 밖이다");
    return empty;
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y,
    s_goal.mx, s_goal.my))
  {
    RCLCPP_WARN(logger_, "목표점이 costmap 밖이다");
    return empty;
  }
  s_start.mode = GROUND;
  s_goal.mode = GROUND;

  if (!spec_->groundOk(s_start.mx, s_start.my)) {
    RCLCPP_WARN(logger_, "시작점이 로버가 설 수 없는 셀이다");
    return empty;
  }
  if (!spec_->groundOk(s_goal.mx, s_goal.my)) {
    RCLCPP_WARN(logger_, "목표점이 로버가 설 수 없는 셀이다");
    return empty;
  }

  const auto t0 = steady_clock::now();
  const SearchResult r = search(s_start, s_goal, cancel_checker);

  if (!r.found) {
    RCLCPP_WARN(
      logger_, "경로를 찾지 못했다 (%s, %zu 노드 확장)",
      r.timed_out ? "타임아웃" : "해 없음", r.n_expanded);
    return empty;
  }

  std::vector<Waypoint> wps = toWaypoints(r.path);
  double final_cost = r.cost;

  if (smooth_path_) {
    const std::vector<Waypoint> sm = smooth(wps);
    // 스무딩 결과가 실제로 더 싼지 확인하고 채택 (최악의 경우 원본 유지)
    CostAccumulator acc_sm;
    bool ok = true;
    CostAccumulator total;
    for (size_t i = 0; i + 1 < sm.size(); ++i) {
      if (!segmentCost(sm[i], sm[i + 1], acc_sm)) {ok = false; break;}
      total += acc_sm;
    }
    if (ok && energy_.cost(total) <= r.cost) {
      wps = sm;
      final_cost = energy_.cost(total);
    }
  }

  const double elapsed =
    duration_cast<duration<double>>(steady_clock::now() - t0).count();

  RCLCPP_INFO(
    logger_,
    "경로 생성: C=%.4f (E=%.3fWh T=%.1fs 전환=%d) | %zu 노드 확장, %zu 점, %.3fs",
    final_cost, r.acc.eTotal(), r.acc.time_s, r.acc.nSwitches(),
    r.n_expanded, wps.size(), elapsed);

  // 탐색은 timeout 안에 끝나도 후처리가 늦으면 전체가 제약을 넘는다.
  // 조용히 넘어가면 실시간성 위반을 놓치므로 경고를 남긴다.
  if (elapsed > timeout_s_) {
    RCLCPP_WARN(
      logger_,
      "전체 소요 %.2fs 가 timeout %.2fs 를 초과했다 "
      "(탐색 %zu 노드). 스무딩 비용을 확인할 것.",
      elapsed, timeout_s_, r.n_expanded);
  }

  if (publish_switch_plan_ && switch_plan_pub_ && switch_plan_pub_->is_activated()) {
    switch_plan_pub_->publish(buildSwitchPlan(wps, header));
  }

  return toPathMsg(wps, header);
}

}  // namespace drobot_hybrid_planner

PLUGINLIB_EXPORT_CLASS(
  drobot_hybrid_planner::HybridAStarPlanner, nav2_core::GlobalPlanner)
