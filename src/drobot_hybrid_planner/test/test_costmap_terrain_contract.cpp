// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// cost <-> 높이 계약이 호스트 costmap 설정에 얼마나 오염되는가 — 측정 테스트
//
// 배경
// ----
// HybridAStarPlanner 는 지형 높이를 별도 채널로 받지 않고, master costmap 의
// cost 에서 등급을 역추론한다 (CostmapTerrainSource, state_space.cpp):
//     cost <= 50  -> 0.00 m  free
//     cost <= 150 -> 0.15 m  rover_traversable  (로버 주행 가능)
//     cost <= 253 -> 0.60 m  fly_over           (로버 주행 불가)
//     254         -> 99 m    impassable
// master 는 모든 레이어를 합친 결과다. global_costmap 의 레이어 순서가
// [obstacle_layer, elevation_layer, inflation_layer] 라서, ElevationLayer 뒤에
// 도는 InflationLayer 가 쓴 값(내접 반경 안 253, 그 밖 252*exp(-k(d - r)))도
// '높이'로 읽힌다. 그 값은 호스트가 정한 inflation_radius / cost_scaling_factor
// 와 footprint 가 정한다.
//
// 무엇을 재는가
// -------------
// Gazebo·SLAM 없이 실제 nav2_costmap_2d::InflationLayer 를 LayeredCostmap 에
// 붙여 master 를 만들고, 플래너와 같은 CostmapTerrainSource / ProblemSpec 으로 읽는다.
//   (a) 벽 옆 빈 공간이 h > 0 으로 읽히는 띠의 폭
//   (b) 실제 0.15 m 턱(rover_traversable 등급)이 벽 근처에 있을 때의 오분류
//   (c) 목표점이 벽에 가까울 때 roverTraversable(goal) == false 가 되는 거리
//   (d) 호스트 obstacle_layer 가 fly_over 장애물을 lethal 로 찍으면 등급이 가려진다
//   (e) 벽 하나만 있어도 생기는 가짜 지형 — 비행 고도 후보, rover_climb 의 내접 구간
// 호스트 설정 네 가지로 반복해 표를 찍는다
// (결과 정리: src/drobot_hybrid_planner/COST_HEIGHT_CONTRACT.md).
//
// ElevationLayer 자리에는 같은 결합 규칙('더 큰 값만 덮어쓴다', elevation_layer.cpp
// 의 updateCosts)과 같은 cost 값(nav2_params_hybrid.yaml: 0/100/200/254)을 쓰는
// 고정 격자 레이어를 둔다. 센서 콜백 없이 등급을 직접 넣기 위해서다.
//
// 이 파일은 설계를 바꾸지 않는다. 현재 동작을 수치로 못박는 특성화 테스트다.
// 높이를 별도 채널로 받도록 바꾸면 여기 단언이 깨지는 게 정상이다.

// cpplint 규칙상 C 헤더(.h)가 C++ 헤더보다 앞에 와야 한다.
#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <ostream>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "drobot_hybrid_planner/energy_model.hpp"
#include "drobot_hybrid_planner/state_space.hpp"

using drobot_hybrid_planner::CostmapTerrainSource;
using drobot_hybrid_planner::EnergyModel;
using drobot_hybrid_planner::Modal;
using drobot_hybrid_planner::ProblemSpec;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace
{
// ---- global_costmap 설정 (nav2_params_hybrid.yaml) --------------------------
constexpr double kRes = 0.05;
constexpr double kHalfSide = 0.225;    // footprint 0.45 x 0.45 m -> 내접 반경 0.225 m

// ElevationLayer 등급 cost (nav2_params_hybrid.yaml 의 cost_values)
constexpr unsigned char kGradeRover = 100;
constexpr unsigned char kGradeFlyOver = 200;

// 맵: 왼쪽에 세로 벽 한 줄 (맵 높이 전체). 그 오른쪽으로 거리를 잰다.
// 벽이 맵 끝까지 이어지므로 모든 행에서 '가장 가까운 장애물'이 같은 행의 벽 칸이다.
constexpr unsigned int kNx = 80;       // 4.0 m
constexpr unsigned int kNy = 60;       // 3.0 m
constexpr unsigned int kWallX = 5;
constexpr unsigned int kRowFloor = 12;             // 빈 바닥을 재는 행
constexpr unsigned int kStepRow0 = 30;             // 턱 띠: 이 행부터
constexpr unsigned int kStepRow1 = 40;             //        이 행 전까지
constexpr unsigned int kRowStep = 35;              // 턱 띠 가운데 행
constexpr unsigned int kMaxK = kNx - 1 - kWallX;   // 잴 수 있는 최대 거리 (칸)

/// 호스트의 inflation 설정 한 벌
struct HostInflation
{
  const char * label;
  double radius;
  double scaling;
};

// 표의 행. 앞의 셋은 과제에서 지정한 '흔히 쓰는 값', 마지막은 파라미터를 하나도
// 안 줬을 때 InflationLayer 가 쓰는 코드 기본값이다 (DefaultsAreWhatTheTableSays 참고).
const HostInflation kHosts[] = {
  {"0.55 / 3.0", 0.55, 3.0},
  {"0.70 / 3.0 (Jazzy nav2_bringup 기본)", 0.70, 3.0},
  {"1.00 / 2.0 (이 저장소)", 1.00, 2.0},
  {"0.55 / 10.0 (InflationLayer 코드 기본)", 0.55, 10.0},
};
constexpr int kRepoHost = 2;


/// 등급을 고정 격자로 들고 있다가 master 에 '더 큰 값만' 덮어쓰는 레이어.
/// elevation_layer.cpp 의 updateCosts 와 같은 규칙이고, nav2 ObstacleLayer 의
/// 기본 결합(updateWithMax)과도 같다. NO_INFORMATION 칸은 쓰지 않는다.
class GridLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    enabled_ = true;
    current_ = true;
    matchSize();
  }

  void matchSize() override
  {
    const auto * m = layered_costmap_->getCostmap();
    nx_ = m->getSizeInCellsX();
    ny_ = m->getSizeInCellsY();
    cells_.assign(static_cast<size_t>(nx_) * ny_, NO_INFORMATION);
  }

  void reset() override {}
  bool isClearable() override {return false;}

  void updateBounds(
    double, double, double,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    // 매번 맵 전체를 갱신한다
    const auto * m = layered_costmap_->getCostmap();
    *min_x = std::min(*min_x, m->getOriginX());
    *min_y = std::min(*min_y, m->getOriginY());
    *max_x = std::max(*max_x, m->getOriginX() + m->getSizeInMetersX());
    *max_y = std::max(*max_y, m->getOriginY() + m->getSizeInMetersY());
  }

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master,
    int min_i, int min_j, int max_i, int max_j) override
  {
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        const unsigned char c = cells_[static_cast<size_t>(j) * nx_ + i];
        if (c == NO_INFORMATION) {continue;}
        const unsigned char old = master.getCost(i, j);
        if (old == NO_INFORMATION || c > old) {master.setCost(i, j, c);}
      }
    }
  }

  void set(unsigned int mx, unsigned int my, unsigned char c)
  {
    cells_[static_cast<size_t>(my) * nx_ + mx] = c;
  }

private:
  unsigned int nx_ = 0;
  unsigned int ny_ = 0;
  std::vector<unsigned char> cells_;
};


/// 레이어 순서가 global_costmap 의 plugins 와 같은 costmap 하나.
struct Scene
{
  nav2_util::LifecycleNode::SharedPtr node;
  std::unique_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> layers;
  std::shared_ptr<GridLayer> obstacle;     // 호스트 2D 장애물 레이어 자리 (lethal)
  std::shared_ptr<GridLayer> elevation;    // ElevationLayer 자리 (등급)
  std::shared_ptr<nav2_costmap_2d::InflationLayer> inflation;

  nav2_costmap_2d::Costmap2D * master() {return layers->getCostmap();}
  void update() {layers->updateMap(0.0, 0.0, 0.0);}
};

std::vector<geometry_msgs::msg::Point> squareFootprint(double half)
{
  std::vector<geometry_msgs::msg::Point> fp(4);
  const double xs[] = {half, half, -half, -half};
  const double ys[] = {half, -half, -half, half};
  for (int i = 0; i < 4; ++i) {
    fp[i].x = xs[i];
    fp[i].y = ys[i];
  }
  return fp;
}

struct SceneOptions
{
  bool inflation = true;
  unsigned int nx = kNx;
  unsigned int ny = kNy;
  bool track_unknown = false;    // nav2_params_hybrid.yaml 은 false, Jazzy nav2_bringup 은 true
};

SceneOptions withoutInflation()
{
  SceneOptions o;
  o.inflation = false;
  return o;
}

/// host 가 nullptr 이면 inflation 파라미터를 아예 주지 않는다 (코드 기본값).
std::unique_ptr<Scene> makeScene(
  const HostInflation * host, const SceneOptions & o = SceneOptions{})
{
  static int serial = 0;
  auto s = std::make_unique<Scene>();

  rclcpp::NodeOptions opts;
  if (host) {
    opts.parameter_overrides({
        {"inflation_layer.inflation_radius", host->radius},
        {"inflation_layer.cost_scaling_factor", host->scaling}});
  }
  s->node = std::make_shared<nav2_util::LifecycleNode>(
    "terrain_contract_" + std::to_string(serial++), "", opts);
  s->tf = std::make_unique<tf2_ros::Buffer>(s->node->get_clock());
  s->layers = std::make_unique<nav2_costmap_2d::LayeredCostmap>(
    "map", /*rolling_window=*/ false, o.track_unknown);
  s->layers->resizeMap(o.nx, o.ny, kRes, 0.0, 0.0);

  s->obstacle = std::make_shared<GridLayer>();
  s->layers->addPlugin(s->obstacle);
  s->obstacle->initialize(s->layers.get(), "obstacle_layer", s->tf.get(), s->node, nullptr);

  s->elevation = std::make_shared<GridLayer>();
  s->layers->addPlugin(s->elevation);
  s->elevation->initialize(s->layers.get(), "elevation_layer", s->tf.get(), s->node, nullptr);

  if (o.inflation) {
    s->inflation = std::make_shared<nav2_costmap_2d::InflationLayer>();
    s->layers->addPlugin(s->inflation);
    s->inflation->initialize(
      s->layers.get(), "inflation_layer", s->tf.get(), s->node, nullptr);
  }
  // footprint 를 넣어야 내접 반경이 정해지고 InflationLayer 가 cost 표를 만든다
  s->layers->setFootprint(squareFootprint(kHalfSide));
  return s;
}

/// 벽 한 줄(lethal) + 벽에서 오른쪽으로 뻗은 턱 띠(rover 등급). 나머지는 빈 바닥.
/// 벽은 호스트의 2D 장애물 레이어가 찍은 것으로 둔다 (LiDAR 가 보는 벽).
void buildWallScene(Scene & s)
{
  for (unsigned int my = 0; my < kNy; ++my) {
    s.obstacle->set(kWallX, my, LETHAL_OBSTACLE);
  }
  for (unsigned int my = kStepRow0; my < kStepRow1; ++my) {
    for (unsigned int mx = kWallX + 1; mx < kNx; ++mx) {
      s.elevation->set(mx, my, kGradeRover);
    }
  }
  s.update();
}

// ---- 플래너가 읽는 등급 ------------------------------------------------------
enum class Read { Free, Rover, FlyOver, Impassable };

// gtest 실패 메시지에 등급 이름이 찍히게 한다
void PrintTo(Read r, std::ostream * os)
{
  switch (r) {
    case Read::Free: *os << "free"; break;
    case Read::Rover: *os << "rover"; break;
    case Read::FlyOver: *os << "fly_over"; break;
    default: *os << "impassable"; break;
  }
}

Read readAt(const CostmapTerrainSource & t, unsigned int mx, unsigned int my)
{
  const CostmapTerrainSource::Config c;
  const double h = t.heightAt(mx, my);
  if (h == c.h_free) {return Read::Free;}
  if (h == c.h_rover) {return Read::Rover;}
  if (h == c.h_flyover) {return Read::FlyOver;}
  return Read::Impassable;
}

/// InflationLayer::computeCost 를 독립적으로 다시 쓴 것 (inflation_layer.hpp:149-163).
/// 레이어가 실제로 쓴 값과 비교해 손계산을 검증하는 데 쓴다.
unsigned char formulaCost(int k, double radius, double scaling, double inscribed)
{
  // Costmap2D::cellDistance: ceil(거리 / 해상도) 칸까지만 퍼진다
  const int cells = static_cast<int>(std::max(0.0, std::ceil(radius / kRes)));
  if (k > cells) {return 0;}
  const double d = k * kRes;
  if (d <= inscribed) {return INSCRIBED_INFLATED_OBSTACLE;}
  return static_cast<unsigned char>(
    (INSCRIBED_INFLATED_OBSTACLE - 1) * std::exp(-scaling * (d - inscribed)));
}

Read formulaRead(unsigned char cost)
{
  const CostmapTerrainSource::Config c;
  if (cost <= c.free_max) {return Read::Free;}
  if (cost <= c.rover_max) {return Read::Rover;}
  if (cost <= c.flyover_max) {return Read::FlyOver;}
  return Read::Impassable;
}

/// 벽에서 오른쪽으로 한 행을 읽은 결과.
struct Profile
{
  std::vector<unsigned char> cost;   // [k] = 벽 중심에서 k 칸 떨어진 칸의 cost (k >= 1)
  std::vector<Read> read;
  int fly = 0;        // 벽부터 연속으로 fly_over 로 읽힌 칸 수 (주행 불가)
  int rover = 0;      // 이어서 연속으로 rover(0.15 m) 로 읽힌 칸 수
};

Profile profileAlong(Scene & s, unsigned int row)
{
  const CostmapTerrainSource terrain(s.master(), CostmapTerrainSource::Config{});
  Profile p;
  p.cost.assign(kMaxK + 1, 0);
  p.read.assign(kMaxK + 1, Read::Free);
  for (unsigned int k = 1; k <= kMaxK; ++k) {
    p.cost[k] = s.master()->getCost(kWallX + k, row);
    p.read[k] = readAt(terrain, kWallX + k, row);
  }
  unsigned int k = 1;
  while (k <= kMaxK && p.read[k] == Read::FlyOver) {++p.fly; ++k;}
  while (k <= kMaxK && p.read[k] == Read::Rover) {++p.rover; ++k;}
  return p;
}

double metres(int cells) {return cells * kRes;}

/// 두 벽(lethal) 사이에 빈 칸 n 개가 있는 통로. 가로지르는 칸 중 하나라도
/// cost <= max_cost 이면 지나갈 수 있다. 그렇게 되는 가장 작은 n (칸) 을 돌려준다.
///   max_cost = 150 (rover_max) : 이 플러그인이 로버로 지나갈 수 있는가
///   max_cost = 252 (253 미만)  : 순정 Nav2 플래너가 지나갈 수 있는가
int minPassableGap(const HostInflation & host, unsigned char max_cost)
{
  for (unsigned int n = 1; n < 60; ++n) {
    SceneOptions o;
    o.nx = n + 12;
    o.ny = 10;
    auto s = makeScene(&host, o);
    for (unsigned int my = 0; my < o.ny; ++my) {
      s->obstacle->set(5, my, LETHAL_OBSTACLE);
      s->obstacle->set(5 + n + 1, my, LETHAL_OBSTACLE);
    }
    s->update();
    for (unsigned int k = 1; k <= n; ++k) {
      if (s->master()->getCost(5 + k, o.ny / 2) <= max_cost) {return static_cast<int>(n);}
    }
  }
  return -1;
}
}  // namespace


// ---------------------------------------------------------------------------
// 표를 만드는 데 쓰는 전제가 맞는지 먼저 확인한다
// ---------------------------------------------------------------------------
TEST(TerrainContract, DefaultsAreWhatTheTableSays)
{
  // 파라미터를 하나도 안 주면 InflationLayer 는 0.55 m / 10.0 을 쓴다 — 표 마지막 행
  auto s = makeScene(nullptr);
  EXPECT_DOUBLE_EQ(s->inflation->getInflationRadius(), kHosts[3].radius);
  EXPECT_DOUBLE_EQ(s->inflation->getCostScalingFactor(), kHosts[3].scaling);
  // footprint 0.45 x 0.45 의 내접 반경
  EXPECT_NEAR(s->layers->getInscribedRadius(), kHalfSide, 1e-9);
}


// ---------------------------------------------------------------------------
// inflation 이 없으면 계약은 멀쩡하다 (대조군)
// ---------------------------------------------------------------------------
TEST(TerrainContract, WithoutInflationGradesReadBackExactly)
{
  auto s = makeScene(&kHosts[kRepoHost], withoutInflation());
  buildWallScene(*s);
  const Profile floor = profileAlong(*s, kRowFloor);
  const Profile step = profileAlong(*s, kRowStep);
  for (unsigned int k = 1; k <= kMaxK; ++k) {
    EXPECT_EQ(floor.read[k], Read::Free) << "k=" << k;
    EXPECT_EQ(step.read[k], Read::Rover) << "k=" << k;
  }
}


// ---------------------------------------------------------------------------
// (a)(b)(c) — 호스트 설정별 표. 실제 레이어가 쓴 값이 식과 칸 단위로 같은지도 본다.
// ---------------------------------------------------------------------------
TEST(TerrainContract, ContaminationTablePerHostConfig)
{
  std::printf(
    "\n| inflation_radius / cost_scaling_factor | (a) 주행불가 띠 fly_over(0.60 m) |"
    " (a) 가짜 턱 띠 rover(0.15 m) | (a) h>0 띠 전체 |"
    " (b) 0.15 m 턱 오분류 | (c) 목표 거부 거리 | 순정 플래너 거부(253 이상) |"
    " 로버 통과 최소 통로 폭 | 순정 플래너 최소 통로 폭 |\n"
    "|---|---|---|---|---|---|---|---|---|\n");

  for (const auto & host : kHosts) {
    SCOPED_TRACE(host.label);
    auto s = makeScene(&host);
    buildWallScene(*s);
    const double inscribed = s->layers->getInscribedRadius();

    const Profile floor = profileAlong(*s, kRowFloor);
    const Profile step = profileAlong(*s, kRowStep);

    // 실제 InflationLayer 결과 == 식 (칸 단위, cost 값까지)
    for (unsigned int k = 1; k <= kMaxK; ++k) {
      const unsigned char want = formulaCost(static_cast<int>(k), host.radius, host.scaling,
        inscribed);
      ASSERT_EQ(floor.cost[k], want) << "k=" << k << " (벽 중심에서 " << metres(k) << " m)";
      // 턱 칸은 max(100, inflation) 이다 — ElevationLayer 와 같은 결합 규칙
      ASSERT_EQ(step.cost[k], std::max(kGradeRover, want)) << "k=" << k;
      ASSERT_EQ(floor.read[k], formulaRead(want)) << "k=" << k;
    }

    // (a) 빈 공간이 h > 0 으로 읽히는 띠 — 벽 바로 옆부터 끊김 없이 이어진다
    ASSERT_GT(floor.fly, 0);
    ASSERT_EQ(floor.read[floor.fly + floor.rover + 1], Read::Free);
    const int band = floor.fly + floor.rover;

    // (b) rover 등급 턱이 fly_over(주행 불가)로 읽히는 칸 = 빈 바닥의 fly 띠와 같다.
    //     그 바깥 rover 띠에서는 턱과 바닥이 둘 다 0.15 m 로 읽혀 턱의 높이차가 사라진다.
    int step_misread = 0;
    for (unsigned int k = 1; k <= kMaxK; ++k) {
      if (step.read[k] != Read::Rover) {
        ++step_misread;
        EXPECT_EQ(step.read[k], Read::FlyOver) << "k=" << k;
      }
    }
    EXPECT_EQ(step_misread, floor.fly);
    int step_edge_lost = 0;
    for (unsigned int k = 1; k <= kMaxK; ++k) {
      if (step.read[k] == Read::Rover && floor.read[k] == Read::Rover) {++step_edge_lost;}
    }
    EXPECT_EQ(step_edge_lost, floor.rover);

    // (c) 목표 거부 — createPlan 은 spec_->groundOk(goal) 이 false 면 계획을 거부한다
    //     ("목표점이 로버가 설 수 없는 셀이다"). hybrid / rover_detour 에서 groundOk 는
    //     roverTraversable 과 같다.
    const CostmapTerrainSource terrain(s->master(), CostmapTerrainSource::Config{});
    const EnergyModel model;
    ProblemSpec::Params pp;
    pp.modal = Modal::Hybrid;
    const ProblemSpec spec(
      s->master(), std::make_shared<CostmapTerrainSource>(s->master(),
      CostmapTerrainSource::Config{}), &model, pp);
    int reject = 0;
    for (unsigned int k = 1; k <= kMaxK; ++k) {
      const bool ok = terrain.roverTraversable(kWallX + k, kRowFloor);
      EXPECT_EQ(ok, spec.groundOk(kWallX + k, kRowFloor)) << "k=" << k;
      if (!ok) {reject = static_cast<int>(k);}
    }
    EXPECT_EQ(reject, floor.fly) << "거부되는 칸이 벽부터 연속이어야 한다";
    // 순정 Nav2 플래너가 목표를 거부하는 구간 = INSCRIBED(253) 이상
    int stock_reject = 0;
    for (unsigned int k = 1; k <= kMaxK; ++k) {
      if (floor.cost[k] >= INSCRIBED_INFLATED_OBSTACLE) {stock_reject = static_cast<int>(k);}
    }
    EXPECT_GT(reject, stock_reject) << "이 플러그인이 순정보다 벽에서 더 멀리까지 거부한다";

    // (a) 의 결과로 좁은 통로가 닫힌다. 양쪽 fly 띠가 만나면 로버가 못 지나가고,
    // hybrid 는 비행(또는 우회)을, rover_detour 는 우회를 강요당한다.
    const int gap_rover = minPassableGap(host, CostmapTerrainSource::Config{}.rover_max);
    const int gap_stock = minPassableGap(host, INSCRIBED_INFLATED_OBSTACLE - 1);
    EXPECT_EQ(gap_rover, 2 * floor.fly + 1);
    EXPECT_EQ(gap_stock, 2 * stock_reject + 1);

    std::printf(
      "| %s | %.2f m (%d칸) | %.2f ~ %.2f m (%d칸) | %.2f m | %.2f m 이내 주행불가 오독,"
      " %.2f ~ %.2f m 높이차 소실 | %.2f m 이내 | %.2f m 이내 | %.2f m | %.2f m |\n",
      host.label, metres(floor.fly), floor.fly,
      metres(floor.fly + 1), metres(band), floor.rover, metres(band),
      metres(step_misread), metres(floor.fly + 1), metres(band),
      metres(reject), metres(stock_reject), metres(gap_rover), metres(gap_stock));
  }
  std::printf(
    "거리는 벽 칸 중심 ~ 칸 중심 (벽 표면까지는 %.3f m 짧다). 통로 폭은 두 벽 표면 사이"
    " 빈 칸 폭. footprint 0.45 x 0.45 m, 내접 %.3f m, 해상도 %.2f m.\n\n",
    kRes / 2, kHalfSide, kRes);
}


// ---------------------------------------------------------------------------
// (a)(b)(c) — 이 저장소 설정(1.00 / 2.0) 값을 못박는다.
// 과제의 손계산: Δ < 0.26 m 주행불가, 0.26 <= Δ < 0.81 m 가짜 턱 (Δ = d - 0.225).
// 실제로는 두 번째 경계가 cost 50 이 아니라 inflation_radius(1.0 m)에서 끊긴다 —
// 반경 끝 칸의 cost 가 아직 53 이다.
// ---------------------------------------------------------------------------
TEST(TerrainContract, RepoConfigNumbers)
{
  auto s = makeScene(&kHosts[kRepoHost]);
  buildWallScene(*s);
  const Profile floor = profileAlong(*s, kRowFloor);

  EXPECT_EQ(floor.fly, 9);        // d = 0.05 ~ 0.45 m  (Δ <= 0.225 m)
  EXPECT_EQ(floor.rover, 11);     // d = 0.50 ~ 1.00 m  (Δ = 0.275 ~ 0.775 m)
  EXPECT_EQ(floor.cost[9], 160);   // 마지막 주행불가 칸
  EXPECT_EQ(floor.cost[10], 145);  // 첫 가짜 턱 칸
  EXPECT_EQ(floor.cost[20], 53);   // 반경 끝 — 아직 free(<= 50) 가 아니다
  EXPECT_EQ(floor.cost[21], 0);    // 반경 밖
}


// ---------------------------------------------------------------------------
// (d) 호스트 obstacle_layer 가 fly_over 장애물을 lethal 로 찍으면 등급이 가려진다.
//
// ElevationLayer 는 '더 큰 값만' 쓰므로 다른 레이어의 254 를 낮출 수 없다.
// 이 저장소의 LiDAR 는 지면에서 약 0.55 m (base_footprint->base_link 0.25 +
// lidar_joint 0.3015) 에 있어, 그보다 높은 장애물은 obstacle_layer 가 254 로 찍는다.
// 순정 스택의 static_layer(SLAM 지도)도 같은 장애물을 점유로 찍는다.
// ---------------------------------------------------------------------------
TEST(TerrainContract, HostObstacleLayerMasksFlyOverGrade)
{
  const unsigned int x0 = 40, x1 = 46, y0 = 27, y1 = 33;   // 0.3 m 상자
  const EnergyModel model;
  ProblemSpec::Params pp;
  pp.modal = Modal::Hybrid;

  for (const bool lidar_sees_it : {false, true}) {
    SCOPED_TRACE(lidar_sees_it ? "LiDAR 가 본다" : "LiDAR 아래");
    auto s = makeScene(&kHosts[kRepoHost]);
    for (unsigned int my = y0; my < y1; ++my) {
      for (unsigned int mx = x0; mx < x1; ++mx) {
        s->elevation->set(mx, my, kGradeFlyOver);           // 날아 넘을 수 있는 상자
        if (lidar_sees_it) {s->obstacle->set(mx, my, LETHAL_OBSTACLE);}
      }
    }
    s->update();
    const auto terrain = std::make_shared<CostmapTerrainSource>(
      s->master(), CostmapTerrainSource::Config{});
    const ProblemSpec spec(s->master(), terrain, &model, pp);

    const unsigned int cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    bool flyable = false;
    for (size_t lv = 0; lv < spec.airLevels().size(); ++lv) {
      flyable = flyable || spec.airOkAt(cx, cy, static_cast<uint8_t>(lv));
    }
    // 상자 바로 옆 바닥 (상자 가장자리에서 2칸)
    const Read beside = readAt(*terrain, x1 + 1, cy);

    if (lidar_sees_it) {
      EXPECT_EQ(readAt(*terrain, cx, cy), Read::Impassable);
      EXPECT_FALSE(flyable) << "등급 200 이 254 에 가려 비행으로도 못 넘는다";
      EXPECT_EQ(beside, Read::FlyOver) << "lethal 이 되면 주변에 inflation 띠도 생긴다";
    } else {
      EXPECT_EQ(readAt(*terrain, cx, cy), Read::FlyOver);
      EXPECT_TRUE(flyable);
      EXPECT_EQ(beside, Read::Free) << "등급 200 은 inflation 의 씨앗이 아니다";
    }
  }
}


// ---------------------------------------------------------------------------
// (e) 벽 하나만 있는 맵에서 생기는 가짜 지형
//   - 비행 고도 후보: 실제로는 평지뿐인데 0.15 / 0.60 m 지형이 '있다'고 읽혀
//     후보가 {0.80} 에서 {0.80, 0.95, 1.40} 으로 는다.
//   - rover_climb: 내접 구간(253, 로봇 footprint 가 벽과 겹치는 곳)을 0.60 m
//     장애물로 읽어 '밟고 넘을 수 있다'고 판정한다. 충돌 구간이 주행 가능이 된다.
// ---------------------------------------------------------------------------
TEST(TerrainContract, WallAloneCreatesPhantomTerrain)
{
  const EnergyModel model;
  const auto wallOnly = [](bool with_inflation) {
      auto s = makeScene(
        &kHosts[kRepoHost], with_inflation ? SceneOptions{} : withoutInflation());
      for (unsigned int my = 0; my < kNy; ++my) {
        s->obstacle->set(kWallX, my, LETHAL_OBSTACLE);
      }
      s->update();
      return s;
    };
  const auto specOf = [&model](Scene & s, Modal modal) {
      ProblemSpec::Params pp;
      pp.modal = modal;
      return ProblemSpec(
        s.master(), std::make_shared<CostmapTerrainSource>(
          s.master(), CostmapTerrainSource::Config{}), &model, pp);
    };

  // inflation 없음 — 평지뿐이라 비행 고도 후보는 하나
  auto plain = wallOnly(false);
  const ProblemSpec plain_spec = specOf(*plain, Modal::Hybrid);
  ASSERT_EQ(plain_spec.airLevels().size(), 1u);
  EXPECT_NEAR(plain_spec.airLevels()[0], 0.80, 1e-9);

  // inflation 있음 — 벽 옆 띠가 0.15 / 0.60 m 지형으로 읽혀 후보가 는다
  auto inflated = wallOnly(true);
  const ProblemSpec hybrid = specOf(*inflated, Modal::Hybrid);
  ASSERT_EQ(hybrid.airLevels().size(), 3u);
  EXPECT_NEAR(hybrid.airLevels()[1], 0.95, 1e-9);   // 가짜 0.15 m + 0.8
  EXPECT_NEAR(hybrid.airLevels()[2], 1.40, 1e-9);   // 가짜 0.60 m + 0.8

  // rover_climb — 로봇 중심이 벽 칸 중심에서 k*0.05 m. k <= 4 면 반폭 0.225 m 인
  // 로봇이 벽과 겹치는데(그래서 253), 이를 0.60 m 장애물로 읽고 밟고 넘을 수 있다고 본다.
  const ProblemSpec climb = specOf(*inflated, Modal::RoverClimb);
  for (unsigned int k = 1; k * kRes <= kHalfSide; ++k) {
    EXPECT_EQ(inflated->master()->getCost(kWallX + k, kRowFloor), INSCRIBED_INFLATED_OBSTACLE);
    EXPECT_TRUE(climb.groundOk(kWallX + k, kRowFloor))
      << "k=" << k << ": rover_climb 이 충돌 구간을 주행 가능으로 본다";
    EXPECT_FALSE(hybrid.groundOk(kWallX + k, kRowFloor)) << "k=" << k;
  }
  EXPECT_FALSE(climb.groundOk(kWallX, kRowFloor)) << "벽 자체(254)는 막힌다";
}


// ---------------------------------------------------------------------------
// (f) 미탐색 칸의 의미가 호스트의 track_unknown_space 로 뒤집힌다.
//   false (이 저장소)          -> 기본값 0   -> 평지 0.00 m, 주행 가능
//   true  (Jazzy nav2_bringup) -> 기본값 255 -> 99 m 벽, 주행도 비행도 불가
// 순정 플래너는 allow_unknown(기본 true)으로 미탐색 칸을 지나간다.
// ---------------------------------------------------------------------------
TEST(TerrainContract, UnknownSpaceMeaningFollowsTrackUnknownSpace)
{
  const EnergyModel model;
  ProblemSpec::Params pp;
  pp.modal = Modal::Hybrid;
  for (const bool track_unknown : {false, true}) {
    SCOPED_TRACE(track_unknown ? "track_unknown_space: true" : "track_unknown_space: false");
    SceneOptions o;
    o.track_unknown = track_unknown;
    auto s = makeScene(&kHosts[kRepoHost], o);
    s->update();    // 아무 레이어도 쓰지 않은 칸 = 미탐색
    const auto terrain = std::make_shared<CostmapTerrainSource>(
      s->master(), CostmapTerrainSource::Config{});
    const ProblemSpec spec(s->master(), terrain, &model, pp);
    bool flyable = false;
    for (size_t lv = 0; lv < spec.airLevels().size(); ++lv) {
      flyable = flyable || spec.airOkAt(kNx / 2, kNy / 2, static_cast<uint8_t>(lv));
    }
    if (track_unknown) {
      EXPECT_EQ(s->master()->getCost(kNx / 2, kNy / 2), NO_INFORMATION);
      EXPECT_EQ(readAt(*terrain, kNx / 2, kNy / 2), Read::Impassable);
      EXPECT_FALSE(terrain->roverTraversable(kNx / 2, kNy / 2));
      EXPECT_FALSE(flyable);
    } else {
      EXPECT_EQ(readAt(*terrain, kNx / 2, kNy / 2), Read::Free);
      EXPECT_TRUE(terrain->roverTraversable(kNx / 2, kNy / 2));
      EXPECT_TRUE(flyable);
    }
  }
}


// ---------------------------------------------------------------------------
// (g) 호스트와 무관한 역추론 자체의 한계 — fly_over 등급 하나가 0.15 < h <= 1.2 m
// (fly_over_max) 를 전부 덮는데 대표 높이는 0.60 m 하나다. 그래서 가장 낮은 비행
// 고도 0.80 m 로 '넘을 수 있다'고 판정한다. 실제 높이가 0.6 m 를 넘으면 최소 여유
// (0.2 m)가 깨지고, 1.0 m 를 넘으면 비행 고도가 장애물 윗면보다 낮다.
// ---------------------------------------------------------------------------
TEST(TerrainContract, FlyOverGradeHidesTrueHeight)
{
  auto s = makeScene(&kHosts[kRepoHost], withoutInflation());
  const unsigned int bx = 40, by = 30;
  s->elevation->set(bx, by, kGradeFlyOver);   // ElevationLayer 는 0.2 m 든 1.2 m 든 200 을 쓴다
  s->update();
  const EnergyModel model;
  ProblemSpec::Params pp;
  pp.modal = Modal::Hybrid;
  const ProblemSpec spec(
    s->master(), std::make_shared<CostmapTerrainSource>(
      s->master(), CostmapTerrainSource::Config{}), &model, pp);

  int lowest = -1;
  for (size_t lv = 0; lv < spec.airLevels().size() && lowest < 0; ++lv) {
    if (spec.airOkAt(bx, by, static_cast<uint8_t>(lv))) {lowest = static_cast<int>(lv);}
  }
  ASSERT_GE(lowest, 0);
  const double z = spec.levelZ(static_cast<uint8_t>(lowest));
  EXPECT_NEAR(z, 0.80, 1e-9);
  std::printf("\nfly_over 칸 위 계획 고도 %.2f m -> 실제 높이별 여유:", z);
  for (const double h_true : {0.20, 0.60, 1.00, 1.20}) {
    std::printf("  h=%.2f: %+.2f m", h_true, z - h_true);
  }
  std::printf("  (요구 여유 %.2f m)\n", model.minFlightClearance());
  EXPECT_LT(z, 1.20 + model.minFlightClearance()) << "fly_over_max 높이는 못 넘는 고도다";
}


// ---------------------------------------------------------------------------
// 실제 맵에서의 영향 — 호스트 설정만 바꿔 가며 같은 맵·같은 에너지 세트로 계획한다.
//
// '순수' = ElevationLayer 등급만 (inflation 없음, LiDAR 가림 없음).
// 나머지 = 호스트 inflation + obstacle_layer. obstacle_layer 는 LiDAR 평면
// (지면에서 0.55 m = base_footprint->base_link 0.25 + lidar_joint 0.3015) 보다 높은
// 장애물을 254 로 찍는다고 둔다.
// 맵 칸 배치는 Python 원본과 같다 (int(round(x / res))).
// ---------------------------------------------------------------------------
namespace
{
constexpr double kLidarHeight = 0.25 + 0.30154;
constexpr double kWallHeight = 3.0;

struct Box
{
  double x0, y0, x1, y1, h;
};

/// 벤치마크 맵 한 장 (실좌표 m). 외벽 높이는 3.0 m.
struct BoxMap
{
  std::string name;
  double width, height, border;
  std::vector<Box> boxes;
  double sx, sy, gx, gy;    // 시작 / 목표 (m)
};

unsigned int cellOf(double m) {return static_cast<unsigned int>(std::lround(m / kRes));}

/// ElevationLayer::classify 를 평평한 상자에 적용한 등급 (경사·거칠기 조건은 생략)
unsigned char gradeFor(double h)
{
  if (h + 0.8 + 0.5 > 2.5 || h > 1.2) {return LETHAL_OBSTACLE;}   // 천장 제약 / fly_over_max
  if (h <= 0.15) {return kGradeRover;}
  return kGradeFlyOver;
}

/// 등급(과 LiDAR 가림)을 레이어에 넣고 master 를 만든다. 칸별 실제 높이를 돌려준다.
std::vector<double> buildBoxMap(Scene & s, const BoxMap & map, bool lidar_masks)
{
  const unsigned int nx = cellOf(map.width), ny = cellOf(map.height), t = cellOf(map.border);
  std::vector<double> true_h(static_cast<size_t>(nx) * ny, 0.0);
  const auto put = [&](unsigned int mx, unsigned int my, double h) {
      true_h[static_cast<size_t>(my) * nx + mx] = h;
      s.elevation->set(mx, my, gradeFor(h));
      if (lidar_masks && h > kLidarHeight) {s.obstacle->set(mx, my, LETHAL_OBSTACLE);}
    };
  for (unsigned int my = 0; my < ny; ++my) {
    for (unsigned int mx = 0; mx < nx; ++mx) {
      if (mx < t || my < t || mx >= nx - t || my >= ny - t) {put(mx, my, kWallHeight);}
    }
  }
  // Python 원본처럼 뒤에 그린 상자가 외벽 칸을 덮어쓴다
  for (const auto & b : map.boxes) {
    for (unsigned int my = cellOf(b.y0); my < std::min(ny, cellOf(b.y1)); ++my) {
      for (unsigned int mx = cellOf(b.x0); mx < std::min(nx, cellOf(b.x1)); ++mx) {
        put(mx, my, b.h);
      }
    }
  }
  s.update();
  return true_h;
}

using drobot_hybrid_planner::AIR;
using drobot_hybrid_planner::CostAccumulator;
using drobot_hybrid_planner::GROUND;
using drobot_hybrid_planner::State;

State stateOf(const ProblemSpec & spec, size_t idx)
{
  const size_t slots = spec.slotsPerCell();
  const size_t cell = idx / slots;
  const size_t slot = idx % slots;
  const size_t nx = spec.costmap()->getSizeInCellsX();
  State s;
  s.mx = static_cast<unsigned int>(cell % nx);
  s.my = static_cast<unsigned int>(cell / nx);
  s.mode = slot == 0 ? GROUND : AIR;
  s.level = slot == 0 ? 0 : static_cast<uint8_t>(slot - 1);
  return s;
}

struct PlanResult
{
  bool found = false;
  CostAccumulator acc;
  std::vector<State> path;
};

/// HybridAStarPlanner::search 와 같은 문제 (같은 전이·비용·휴리스틱). 비용이 누적값에
/// 선형이라 간선 비용을 더해 가도 최적해가 같다. 스무딩은 하지 않는다.
PlanResult solve(
  const ProblemSpec & spec, const EnergyModel & model, const State & start, const State & goal)
{
  const size_t n = spec.numStates();
  constexpr size_t kNone = std::numeric_limits<size_t>::max();
  std::vector<double> g(n, std::numeric_limits<double>::infinity());
  std::vector<CostAccumulator> acc(n);
  std::vector<size_t> parent(n, kNone);
  std::vector<bool> closed(n, false);
  using Item = std::pair<double, size_t>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  const size_t s0 = spec.index(start);
  const size_t goal_idx = spec.index(goal);
  g[s0] = 0.0;
  open.push({spec.heuristic(start, goal), s0});
  std::vector<ProblemSpec::Edge> edges;
  while (!open.empty()) {
    const size_t i = open.top().second;
    open.pop();
    if (closed[i]) {continue;}
    closed[i] = true;
    if (i == goal_idx) {
      PlanResult r;
      r.found = true;
      r.acc = acc[i];
      for (size_t k = i; k != kNone; k = parent[k]) {
        r.path.push_back(stateOf(spec, k));
      }
      std::reverse(r.path.begin(), r.path.end());
      return r;
    }
    spec.neighbors(stateOf(spec, i), edges);
    for (const auto & e : edges) {
      const size_t j = spec.index(e.next);
      if (closed[j]) {continue;}
      const double ng = g[i] + model.cost(e.acc);
      if (ng < g[j] - 1e-12) {
        g[j] = ng;
        acc[j] = acc[i] + e.acc;
        parent[j] = i;
        open.push({ng + spec.heuristic(e.next, goal), j});
      }
    }
  }
  return PlanResult{};
}

/// nav2_params_hybrid_derived.yaml 과 같은 값 (나머지는 default 세트와 같다)
EnergyModel derivedModel()
{
  rclcpp::NodeOptions o;
  o.parameter_overrides({
      {"energy_model.mode_switch.takeoff_energy", 0.5},
      {"energy_model.mode_switch.landing_energy", 0.3},
      {"energy_model.air_mode.energy_per_meter", 0.6},
      {"energy_model.mode_switch.energy_per_altitude_meter", 0.4}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("derived_energy", o);
  EnergyModel m;
  m.configure(node, "energy_model");
  return m;
}

/// 표 한 칸: 전환 횟수 · 총 에너지 · 비행 거리, 그리고 비행 고도가 실제 높이 + 최소 여유
/// 보다 낮았던 구간이 있으면 표시한다.
std::string describe(
  const PlanResult & r, const ProblemSpec & spec, const EnergyModel & model,
  const std::vector<double> & true_h)
{
  if (!r.found) {return "경로 없음";}
  const size_t nx = spec.costmap()->getSizeInCellsX();
  double worst = std::numeric_limits<double>::infinity();   // 가장 작은 (z - h_true)
  for (const auto & st : r.path) {
    if (st.mode != AIR) {continue;}
    worst = std::min(worst, spec.levelZ(st.level) - true_h[st.my * nx + st.mx]);
  }
  char buf[160];
  std::snprintf(
    buf, sizeof(buf), "전환 %d · %.2f Wh · 비행 %.1f m%s", r.acc.nSwitches(), r.acc.eTotal(),
    r.acc.dist_air,
    worst < model.minFlightClearance() - 1e-9 ?
    (worst < 0.0 ? " · **충돌 고도**" : " · **여유 부족**") : "");
  return buf;
}
}  // namespace

TEST(TerrainContract, BenchmarkMapDecisionsPerHostConfig)
{
  const EnergyModel def;
  const EnergyModel der = derivedModel();
  ASSERT_NEAR(der.takeoff(0.0).e_switch, 0.5, 1e-12);

  std::vector<BoxMap> maps;
  // 독립연구 통제 맵 — scripts/test_maps.py build_base_map (benchmark/envs/base_maps/*.npz)
  for (const double h : {0.50, 1.00}) {
    char name[32];
    std::snprintf(name, sizeof(name), "base_map_h%.1f", h);
    maps.push_back({name, 6.0, 11.0, 0.1, {{0.1, 4.0, 4.0, 7.0, h}}, 2.0, 1.0, 2.0, 10.0});
  }
  // 벤치마크 맵 — benchmark/envs/heightmap.py make_medium_open
  maps.push_back({"medium_open", 22.0, 14.0, 0.3,
      {{5.0, 0.0, 5.6, 12.0, 0.55}, {11.0, 2.5, 11.6, 14.0, 1.10}, {17.0, 0.0, 17.6, 10.5, 0.80}},
      1.5, 7.0, 20.5, 7.0});

  std::printf("\n| 맵 | 에너지 세트 | 순수(등급만) |");
  for (const auto & host : kHosts) {
    std::printf(" %s + LiDAR |", host.label);
  }
  std::printf("\n|---|---|---|");
  for (size_t i = 0; i < std::size(kHosts); ++i) {
    std::printf("---|");
  }
  std::printf("\n");

  for (const auto & map : maps) {
    SceneOptions o;
    o.nx = cellOf(map.width);
    o.ny = cellOf(map.height);
    const State start{cellOf(map.sx), cellOf(map.sy), GROUND, 0};
    const State goal{cellOf(map.gx), cellOf(map.gy), GROUND, 0};
    for (const auto * model : {&def, &der}) {
      // 한 행을 다 모은 뒤 한 번에 찍는다 (노드 소멸 로그가 행 중간에 끼지 않게)
      std::string row = "| " + map.name + " | " + (model == &def ? "default" : "derived") + " |";
      for (int col = -1; col < static_cast<int>(std::size(kHosts)); ++col) {
        const bool pure = col < 0;
        SceneOptions oo = o;
        oo.inflation = !pure;
        auto s = makeScene(pure ? &kHosts[kRepoHost] : &kHosts[col], oo);
        const auto true_h = buildBoxMap(*s, map, /*lidar_masks=*/ !pure);
        ProblemSpec::Params pp;
        pp.modal = Modal::Hybrid;
        const ProblemSpec spec(
          s->master(), std::make_shared<CostmapTerrainSource>(
            s->master(), CostmapTerrainSource::Config{}), model, pp);
        const PlanResult r = solve(spec, *model, start, goal);
        row += " " + describe(r, spec, *model, true_h) + " |";
      }
      std::printf("%s\n", row.c_str());
      std::fflush(stdout);
    }
  }
  std::printf("\n");
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
