// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// 상태공간 전이 테스트 — 특히 '비행으로 장애물을 넘을 수 있는가'.
//
// 왜 이 파일이 생겼나
// -------------------
// Python 벤치마크에서는 3D 비행 고도를 '비행 구간 내내 고정'으로 다룬다.
// C++ 은 그 이전 방식인 '지형 추종'(airZ = 지형높이 + flight_clearance)을 쓴다.
// 지형 추종은 상승각 제약과 충돌한다:
//
//     격자 0.05m, 최대상승각 45도 -> 한 스텝 최대 고도변화 0.050 m
//     0.60m 장애물 경계에서 필요한 고도변화        0.600 m  -> 거부
//
// 즉 장애물 경계에서 모든 공중 전이가 거부되어, 하이브리드 플래너가
// 어떤 장애물도 날아서 넘지 못한다. 이륙은 되지만 갈 곳이 없다.
//
// 수정: '비행 구간 고도 고정' 방식으로 바꿨다. 이륙할 때 고도를 정하고
// 그 구간 내내 유지한다. 아래 두 테스트가 그 회귀를 막는다:
//   FlightStepCanEnterObstacleAirspace  장애물 상공으로 진입할 수 있는가
//   ReachesAcrossFlyOverObstacle        반대편에 도달하는가

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <queue>
#include <set>
#include <tuple>
#include <vector>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "drobot_hybrid_planner/state_space.hpp"

using drobot_hybrid_planner::AIR;
using drobot_hybrid_planner::CostmapTerrainSource;
using drobot_hybrid_planner::EnergyModel;
using drobot_hybrid_planner::GROUND;
using drobot_hybrid_planner::LayerTerrainSource;
using drobot_hybrid_planner::Modal;
using drobot_hybrid_planner::ProblemSpec;
using drobot_hybrid_planner::State;
using drobot_hybrid_planner::TerrainSource;

namespace
{
// 맵 크기 — 장애물 한 줄을 사이에 두고 좌우로 나뉜다.
constexpr unsigned int kNx = 40;
constexpr unsigned int kNy = 20;
constexpr double kRes = 0.05;          // nav2_params_hybrid.yaml 과 같은 값

constexpr unsigned int kWallX = 20;    // 장애물이 있는 열
constexpr unsigned int kMidY = 10;

// elevation_params.yaml 의 cost 매핑.
// fly_over 등급 -> CostmapTerrainSource 가 h_flyover(0.60m)로 읽는다.
constexpr unsigned char kCostFree = 0;
constexpr unsigned char kCostFlyOver = 200;

// fly_over 등급의 대표 높이.
//   기본 0.60m — flight_clearance(0.8) 만으로도 최소여유(0.2)를 겨우 만족한다.
//   0.90m 는 0.8 로는 부족해서 더 높은 고도를 골라야만 넘을 수 있다.
//   두 경우를 모두 시험해야 '고도 선택'이 실제로 동작하는지 알 수 있다.
constexpr double kHDefault = 0.60;
constexpr double kHTall = 0.90;
}  // namespace


class StateSpaceTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("state_space_test");

    // EnergyModel::configure 는 declare_parameter_if_not_declared 로
    // 없는 파라미터에 기본값을 채운다. 그 기본값이 energy_params.yaml 과
    // 같으므로 여기서 따로 선언하지 않는다.
    model_.configure(node_, "energy_model");

    // 빈 맵에 세로 장애물 한 줄. 좌우를 완전히 가른다.
    costmap_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
      kNx, kNy, kRes, 0.0, 0.0, kCostFree);
    for (unsigned int my = 0; my < kNy; ++my) {
      costmap_->setCost(kWallX, my, kCostFlyOver);
    }

    setObstacleHeight(kHDefault);
  }

  /// fly_over 등급의 대표 높이를 바꾼다
  /// (플래너의 flyover_representative_height 파라미터에 해당).
  void setObstacleHeight(double h_flyover)
  {
    CostmapTerrainSource::Config cfg;
    cfg.h_flyover = h_flyover;
    terrain_ = std::make_shared<CostmapTerrainSource>(costmap_.get(), cfg);
  }

  /// modal 을 지정해 ProblemSpec 을 만든다.
  ProblemSpec makeSpec(Modal modal) const
  {
    ProblemSpec::Params p;
    p.flight_clearance = 0.8;
    p.ceiling_height = 2.5;
    p.modal = modal;
    return ProblemSpec(costmap_.get(), terrain_, &model_, p);
  }

  /// 이 셀 위를 날 수 있는 가장 낮은 level. 없으면 -1.
  static int lowestUsableLevel(const ProblemSpec & spec, unsigned int mx, unsigned int my)
  {
    for (size_t lv = 0; lv < spec.airLevels().size(); ++lv) {
      if (spec.airOkAt(mx, my, static_cast<uint8_t>(lv))) {return static_cast<int>(lv);}
    }
    return -1;
  }

  /// s 에서 한 스텝에 갈 수 있는 상태 중 target 이 있는가.
  static bool hasTransition(const ProblemSpec & spec, const State & s, const State & target)
  {
    std::vector<ProblemSpec::Edge> out;
    spec.neighbors(s, out);
    return std::any_of(
      out.begin(), out.end(),
      [&target](const ProblemSpec::Edge & e) {return e.next == target;});
  }

  /// start 에서 전이를 따라 도달 가능한 상태 전체 (BFS).
  ///
  /// 키에 level 이 반드시 들어가야 한다. 빠뜨리면 같은 셀의 다른 고도가
  /// 이미 방문한 것으로 처리되어, 가장 먼저 큐에 들어간 고도만 탐색된다.
  /// 실제로 그 실수 때문에 '낮은 고도로는 못 넘는 장애물' 테스트가
  /// 코드는 멀쩡한데 실패했다.
  using Key = std::tuple<unsigned int, unsigned int, uint8_t, uint8_t>;

  static Key keyOf(const State & s)
  {
    return std::make_tuple(s.mx, s.my, s.mode, s.level);
  }

  static std::set<Key> reachable(const ProblemSpec & spec, const State & start)
  {
    std::set<Key> seen;
    std::queue<State> q;
    seen.insert(keyOf(start));
    q.push(start);

    std::vector<ProblemSpec::Edge> out;
    while (!q.empty()) {
      const State cur = q.front();
      q.pop();
      spec.neighbors(cur, out);
      for (const auto & e : out) {
        if (seen.insert(keyOf(e.next)).second) {
          q.push(e.next);
        }
      }
    }
    return seen;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  EnergyModel model_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  std::shared_ptr<TerrainSource> terrain_;
};


// ---------------------------------------------------------------------------
// 설정이 의도대로 됐는지 먼저 확인 — 아래 결함 테스트의 전제다
// ---------------------------------------------------------------------------
TEST_F(StateSpaceTest, ObstacleIsFlyableButNotDrivable)
{
  const ProblemSpec spec = makeSpec(Modal::Hybrid);

  // 지형 높이가 fly_over 등급(0.60m)으로 읽혀야 한다
  EXPECT_NEAR(spec.terrain(kWallX, kMidY), 0.60, 1e-9);
  EXPECT_NEAR(spec.terrain(kWallX - 1, kMidY), 0.0, 1e-9);

  // 로버는 못 지나간다 (0.60 > rover_traversable_max 0.15)
  EXPECT_FALSE(spec.groundOk(kWallX, kMidY));

  // 그런데 '그 위를 나는 것' 자체는 허용된다.
  //   z_max = 2.5 - 0.25(로봇높이) - 0.5(천장여유) = 1.75 m
  EXPECT_NEAR(spec.zMax(), 1.75, 1e-9);
  EXPECT_GE(lowestUsableLevel(spec, kWallX, kMidY), 0);
}

TEST_F(StateSpaceTest, AirLevelsComeFromTerrainHeights)
{
  // 맵에 있는 지형 높이는 0.0(평지)과 0.60(장애물) 두 가지.
  // 후보 고도는 그 위 flight_clearance(0.8) 만큼이다.
  const ProblemSpec spec = makeSpec(Modal::Hybrid);
  ASSERT_EQ(spec.airLevels().size(), 2u);
  EXPECT_NEAR(spec.airLevels()[0], 0.80, 1e-9);   // 0.00 + 0.8
  EXPECT_NEAR(spec.airLevels()[1], 1.40, 1e-9);   // 0.60 + 0.8

  // 후보가 오름차순이어야 level 인덱스 비교가 의미를 갖는다
  EXPECT_LT(spec.airLevels()[0], spec.airLevels()[1]);
}

TEST_F(StateSpaceTest, ModalGatesTakeoffAndClimb)
{
  const State ground{1, kMidY, GROUND};

  // 우회 — 이륙 불가, 등반 불가
  {
    const ProblemSpec spec = makeSpec(Modal::RoverDetour);
    EXPECT_FALSE(spec.allowFly());
    EXPECT_FALSE(spec.allowClimb());
    EXPECT_FALSE(hasTransition(spec, ground, State{1, kMidY, AIR, 0}));
    EXPECT_FALSE(spec.groundOk(kWallX, kMidY));
  }
  // 밟고넘기 — 이륙 불가, 0.60m 장애물은 지상으로 통과 가능
  {
    const ProblemSpec spec = makeSpec(Modal::RoverClimb);
    EXPECT_FALSE(spec.allowFly());
    EXPECT_TRUE(spec.allowClimb());
    EXPECT_NEAR(spec.roverHLimit(), 0.70, 1e-9);
    EXPECT_FALSE(hasTransition(spec, ground, State{1, kMidY, AIR, 0}));
    EXPECT_TRUE(spec.groundOk(kWallX, kMidY)) << "0.60m <= 0.70m 이므로 밟고 넘을 수 있어야 한다";
  }
  // 하이브리드 — 이륙 가능, 등반 불가
  {
    const ProblemSpec spec = makeSpec(Modal::Hybrid);
    EXPECT_TRUE(spec.allowFly());
    EXPECT_FALSE(spec.allowClimb());
    EXPECT_TRUE(hasTransition(spec, ground, State{1, kMidY, AIR, 0}));
    EXPECT_FALSE(spec.groundOk(kWallX, kMidY));
  }
}


// ---------------------------------------------------------------------------
// 밟고넘기는 장애물을 지상으로 건넌다 (대조군)
// ---------------------------------------------------------------------------
TEST_F(StateSpaceTest, RoverClimbCrossesObstacleOnGround)
{
  const ProblemSpec spec = makeSpec(Modal::RoverClimb);
  const auto seen = reachable(spec, State{1, kMidY, GROUND, 0});

  const bool crossed = std::any_of(
    seen.begin(), seen.end(),
    [](const auto & k) {return std::get<0>(k) > kWallX;});

  EXPECT_TRUE(crossed) << "밟고넘기는 장애물 오른쪽에 도달해야 한다";
}


// ---------------------------------------------------------------------------
// 회귀 방지 — 하이브리드가 비행으로 장애물을 넘을 수 있어야 한다
//
// 지형 추종 방식일 때는 이 두 테스트가 실패했다.
// 장애물 경계에서 고도가 한 셀 만에 0.60m 점프해 상승각 제약(격자 0.05m,
// 45도 -> 0.05m)에 걸렸기 때문이다.
// ---------------------------------------------------------------------------
TEST_F(StateSpaceTest, FlightStepCanEnterObstacleAirspace)
{
  const ProblemSpec spec = makeSpec(Modal::Hybrid);

  const int lv = lowestUsableLevel(spec, kWallX, kMidY);
  ASSERT_GE(lv, 0) << "장애물 상공을 날 수 있는 고도가 하나도 없다";
  const auto level = static_cast<uint8_t>(lv);

  // 장애물 바로 왼쪽 상공에서, 같은 고도로 장애물 상공까지 한 칸.
  const State from{kWallX - 1, kMidY, AIR, level};
  const State to{kWallX, kMidY, AIR, level};

  ASSERT_TRUE(spec.airOkAt(from.mx, from.my, level));
  ASSERT_TRUE(spec.airOkAt(to.mx, to.my, level));

  EXPECT_TRUE(hasTransition(spec, from, to))
    << "장애물 상공으로 진입하는 전이가 없다. 고도 " << spec.levelZ(level) << " m";
}

TEST_F(StateSpaceTest, FlightHasNoVerticalCostWithinSegment)
{
  // 비행 구간 고도가 고정이므로 수평 이동에 수직 에너지가 붙으면 안 된다.
  const ProblemSpec spec = makeSpec(Modal::Hybrid);
  const auto level = static_cast<uint8_t>(lowestUsableLevel(spec, kWallX, kMidY));

  std::vector<ProblemSpec::Edge> out;
  spec.neighbors(State{kWallX - 1, kMidY, AIR, level}, out);

  bool saw_air_move = false;
  for (const auto & e : out) {
    if (e.next.mode != AIR) {continue;}
    saw_air_move = true;
    EXPECT_EQ(e.next.level, level) << "비행 중 고도가 바뀌었다";
    EXPECT_NEAR(e.acc.e_air_vert, 0.0, 1e-12) << "고도 고정인데 수직 에너지가 붙었다";
  }
  EXPECT_TRUE(saw_air_move);
}

TEST_F(StateSpaceTest, ReachesAcrossFlyOverObstacle)
{
  const ProblemSpec spec = makeSpec(Modal::Hybrid);
  const auto seen = reachable(spec, State{1, kMidY, GROUND, 0});

  const bool crossed = std::any_of(
    seen.begin(), seen.end(),
    [](const auto & k) {return std::get<0>(k) > kWallX;});

  EXPECT_TRUE(crossed)
    << "하이브리드가 장애물 오른쪽에 도달하지 못한다.\n"
    << "  이 맵에는 우회로가 없으므로 비행으로만 건널 수 있다.";
}

// 0.60m 장애물은 flight_clearance(0.8) 만으로도 최소여유(0.2)를 겨우 만족한다.
// 즉 가장 낮은 고도로도 넘어진다. 그것만 시험하면 '고도 선택'이 실제로
// 동작하는지 알 수 없으므로, 낮은 고도로는 못 넘는 장애물도 시험한다.
TEST_F(StateSpaceTest, ChoosesHigherAltitudeWhenLowestIsNotEnough)
{
  setObstacleHeight(kHTall);            // 0.90m
  const ProblemSpec spec = makeSpec(Modal::Hybrid);

  ASSERT_EQ(spec.airLevels().size(), 2u);
  EXPECT_NEAR(spec.airLevels()[0], 0.80, 1e-9);   // 0.00 + 0.8
  EXPECT_NEAR(spec.airLevels()[1], 1.70, 1e-9);   // 0.90 + 0.8

  // 가장 낮은 고도로는 못 넘는다 (0.80 < 0.90 + 0.2)
  EXPECT_FALSE(spec.airOkAt(kWallX, kMidY, 0));
  // 높은 고도로는 넘는다 (1.70 >= 1.10, 그리고 1.70 <= z_max 1.75)
  EXPECT_TRUE(spec.airOkAt(kWallX, kMidY, 1));

  // 이륙 시 두 고도가 모두 선택지로 제시되어야 한다 — 그래야 A* 가 고를 수 있다
  std::vector<ProblemSpec::Edge> out;
  spec.neighbors(State{1, kMidY, GROUND, 0}, out);
  int n_takeoff = 0;
  for (const auto & e : out) {
    if (e.next.mode == AIR) {++n_takeoff;}
  }
  EXPECT_EQ(n_takeoff, 2) << "이륙 고도 후보가 모두 제시되지 않았다";

  // 그리고 반대편에 실제로 도달해야 한다
  const auto seen = reachable(spec, State{1, kMidY, GROUND, 0});
  const bool crossed = std::any_of(
    seen.begin(), seen.end(),
    [](const auto & k) {return std::get<0>(k) > kWallX;});
  EXPECT_TRUE(crossed) << "높은 고도를 골라서라도 넘어야 한다";
}


// ---------------------------------------------------------------------------
// LayerTerrainSource — 지형은 등급 격자, 충돌은 master (COST_HEIGHT_CONTRACT.md (b-0))
//
// master 에는 inflation 이 섞여 있다. CostmapTerrainSource 는 그 값을 지형으로
// 읽어 벽 옆 빈 바닥을 fly_over 로 봤다. LayerTerrainSource 는 지형을 등급
// 격자(ElevationLayer 자체 격자)에서 읽고, master 는 충돌(253·254)에만 쓴다.
//
// 맵: 등급 격자는 벽 왼쪽이 관측된 평지, 벽 열이 fly_over, 벽 오른쪽이 미관측.
//     카메라가 장애물 앞면만 보고 그 뒤(윗면)는 못 본 상황과 같다.
// ---------------------------------------------------------------------------
class LayerTerrainTest : public StateSpaceTest
{
protected:
  void SetUp() override
  {
    StateSpaceTest::SetUp();
    grade_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
      kNx, kNy, kRes, 0.0, 0.0, nav2_costmap_2d::NO_INFORMATION);
    master_ = std::make_unique<nav2_costmap_2d::Costmap2D>(
      kNx, kNy, kRes, 0.0, 0.0, kCostFree);
    for (unsigned int my = 0; my < kNy; ++my) {
      for (unsigned int mx = 0; mx < kWallX; ++mx) {
        grade_->setCost(mx, my, kCostFree);
      }
      grade_->setCost(kWallX, my, kCostFlyOver);
      master_->setCost(kWallX, my, kCostFlyOver);
    }
    layer_terrain_ = std::make_shared<LayerTerrainSource>(
      grade_.get(), master_.get(), CostmapTerrainSource::Config{});
  }

  /// master 값을 바꾼 '뒤에' 불러야 한다 (비행 고도 후보를 생성 시점에 만든다).
  ProblemSpec makeLayerSpec(Modal modal, bool land_only_on_observed = true) const
  {
    ProblemSpec::Params p;
    p.flight_clearance = 0.8;
    p.ceiling_height = 2.5;
    p.modal = modal;
    p.land_only_on_observed = land_only_on_observed;
    return ProblemSpec(master_.get(), layer_terrain_, &model_, p);
  }

  std::unique_ptr<nav2_costmap_2d::Costmap2D> grade_;
  std::unique_ptr<nav2_costmap_2d::Costmap2D> master_;
  std::shared_ptr<TerrainSource> layer_terrain_;
};

TEST_F(LayerTerrainTest, InflationIsNotReadAsTerrain)
{
  // 벽 왼쪽 칸에 inflation 180 이 쌓였다. 등급 격자에서는 평지다.
  const unsigned int mx = kWallX - 3;
  master_->setCost(mx, kMidY, 180);

  const ProblemSpec spec = makeLayerSpec(Modal::Hybrid);
  EXPECT_NEAR(spec.terrain(mx, kMidY), 0.0, 1e-9);
  EXPECT_TRUE(spec.groundOk(mx, kMidY));

  // 대조군 — master 에서 등급을 읽는 예전 방식은 같은 칸을 0.60 m 장애물로 본다.
  // 이게 통과해야 위 단언이 '차이를 잡아내는' 테스트다.
  ProblemSpec::Params p;
  p.modal = Modal::Hybrid;
  const ProblemSpec old_spec(
    master_.get(),
    std::make_shared<CostmapTerrainSource>(master_.get(), CostmapTerrainSource::Config{}),
    &model_, p);
  EXPECT_NEAR(old_spec.terrain(mx, kMidY), 0.60, 1e-9);
  EXPECT_FALSE(old_spec.groundOk(mx, kMidY));
}

TEST_F(LayerTerrainTest, FootprintCollisionBlocksEveryModal)
{
  // 내접(253) — 로봇 중심이 여기 오면 footprint 가 장애물과 겹친다.
  // 밟고넘기도 막혀야 한다 (예전 방식은 0.60 m 로 읽어 '밟고 넘을 수 있다' 고 했다).
  const unsigned int mx = kWallX - 1;
  master_->setCost(mx, kMidY, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);

  for (const Modal m : {Modal::RoverDetour, Modal::RoverClimb, Modal::Hybrid}) {
    const ProblemSpec spec = makeLayerSpec(m);
    EXPECT_FALSE(spec.groundOk(mx, kMidY)) << "modal " << static_cast<int>(m);
  }
  EXPECT_TRUE(makeLayerSpec(Modal::Hybrid).groundOk(mx - 1, kMidY)) << "옆 칸은 영향 없다";
}

TEST_F(LayerTerrainTest, LethalInMasterBlocksFlightToo)
{
  // 등급 격자는 평지로 봤지만 다른 레이어(LiDAR)가 LETHAL 로 막았다.
  // 높이를 모르므로 지상도 비행도 안 된다.
  const unsigned int mx = kWallX - 2;
  master_->setCost(mx, kMidY, nav2_costmap_2d::LETHAL_OBSTACLE);

  const ProblemSpec spec = makeLayerSpec(Modal::Hybrid);
  EXPECT_FALSE(spec.groundOk(mx, kMidY));
  EXPECT_EQ(lowestUsableLevel(spec, mx, kMidY), -1) << "LETHAL 위로 비행 계획이 나오면 안 된다";
}

TEST_F(LayerTerrainTest, UnknownInMasterIsNotCollision)
{
  // 순정 플래너의 allow_unknown 과 같은 의미 — 미탐색은 충돌이 아니다.
  master_->setCost(1, kMidY, nav2_costmap_2d::NO_INFORMATION);
  const ProblemSpec spec = makeLayerSpec(Modal::Hybrid);
  EXPECT_TRUE(spec.groundOk(1, kMidY));
}

TEST_F(LayerTerrainTest, UnobservedIsDrivableButNotLandable)
{
  // 벽 오른쪽은 미관측 — 카메라가 못 본 장애물 윗면과 같은 상황.
  const unsigned int mx = kWallX + 3;
  const ProblemSpec spec = makeLayerSpec(Modal::Hybrid);

  ASSERT_FALSE(layer_terrain_->observed(mx, kMidY));
  EXPECT_TRUE(spec.groundOk(mx, kMidY)) << "주행은 낙관 — 가 보면 보인다";
  EXPECT_FALSE(spec.landingOk(mx, kMidY)) << "착륙은 보수 — 못 본 곳에 내려앉지 않는다";
  EXPECT_TRUE(spec.landingOk(1, kMidY)) << "본 평지에는 착륙한다";

  const State above{mx, kMidY, AIR, 0};
  const State below{mx, kMidY, GROUND, 0};
  EXPECT_FALSE(hasTransition(spec, above, below));

  // 끄면 예전처럼 어디든 착륙한다 (대조군)
  const ProblemSpec loose = makeLayerSpec(Modal::Hybrid, false);
  EXPECT_TRUE(loose.landingOk(mx, kMidY));
  EXPECT_TRUE(hasTransition(loose, above, below));
}


// ---------------------------------------------------------------------------
// 비행 한 구간 거리 제약 (max_flight_segment, 인터페이스 규약 SSOT 5.0 m)
// ---------------------------------------------------------------------------
// 제약이 없으면 derived 에너지 세트에서 출발점부터 목표까지 9 m 를 한 번에
// 날았다 (mode_manager 가 '1회 비행거리 초과' 경고). 아래가 그 회귀를 막는다.
TEST_F(StateSpaceTest, FlightSegmentLimitBlocksLongCrossing)
{
  // 폭 1.0 m (x 10~29) fly_over 띠. 건너려면 x=9 에서 떠 x=30 에 내려야 한다:
  // 21 칸 x 0.05 m = 1.05 m 비행.
  nav2_costmap_2d::Costmap2D band(kNx, kNy, kRes, 0.0, 0.0, kCostFree);
  for (unsigned int my = 0; my < kNy; ++my) {
    for (unsigned int mx = 10; mx < 30; ++mx) {
      band.setCost(mx, my, kCostFlyOver);
    }
  }
  auto terrain = std::make_shared<CostmapTerrainSource>(&band, CostmapTerrainSource::Config{});

  const auto crosses = [&](double max_flight) {
      ProblemSpec::Params p;
      p.modal = Modal::Hybrid;
      p.max_flight_distance = max_flight;
      const ProblemSpec spec(&band, terrain, &model_, p);
      const auto seen = reachable(spec, State{1, kMidY, GROUND, 0});
      return std::any_of(
        seen.begin(), seen.end(), [](const auto & k) {
          return std::get<0>(k) >= 30 && std::get<2>(k) == GROUND;
        });
    };
  EXPECT_FALSE(crosses(0.5)) << "0.5 m 제한으로 1.05 m 띠를 건너면 안 된다";
  EXPECT_TRUE(crosses(1.2)) << "1.2 m 면 건널 수 있어야 한다";
  EXPECT_TRUE(crosses(0.0)) << "0 이하는 제한 없음";
}

TEST_F(StateSpaceTest, FlightLimitUsesExactDistanceNotBins)
{
  ProblemSpec::Params p;
  p.modal = Modal::Hybrid;
  p.max_flight_distance = 0.5;
  p.flight_distance_bin = 0.25;
  const ProblemSpec spec(costmap_.get(), terrain_, &model_, p);
  ASSERT_EQ(spec.numFlightBins(), 2u);

  // 0.44 m 날아온 상태 — 직선 한 칸(0.05)은 되고 대각선(0.0707)은 넘는다
  State s{2, kMidY, AIR, 0, 1, 0.44f};
  std::vector<ProblemSpec::Edge> out;
  spec.neighbors(s, out);
  bool straight = false, diagonal = false;
  for (const auto & e : out) {
    if (e.next.mode != AIR) {continue;}
    EXPECT_LE(e.next.flown, 0.5f + 1e-5f);
    const bool diag = e.next.mx != s.mx && e.next.my != s.my;
    (diag ? diagonal : straight) = true;
  }
  EXPECT_TRUE(straight);
  EXPECT_FALSE(diagonal);

  // 구간이 다르면 다른 상태다
  State a{2, kMidY, AIR, 0, 0, 0.1f};
  State b{2, kMidY, AIR, 0, 1, 0.3f};
  EXPECT_NE(spec.index(a), spec.index(b));
}

TEST(HeuristicTest, AdmissibleWhenFlightIsCheaperPerMeter)
{
  // derived 세트처럼 1m 비행이 1m 주행보다 싸면, 지상 단가만 쓰는 휴리스틱은
  // 실제 비용을 넘는다 (admissible 하지 않다).
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"energy_model.air_mode.energy_per_meter", 0.6}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("heuristic_test", "", opts);
  EnergyModel m;
  m.configure(node, "energy_model");

  nav2_costmap_2d::Costmap2D grid(kNx, kNy, kRes, 0.0, 0.0, kCostFree);
  auto terrain = std::make_shared<CostmapTerrainSource>(&grid, CostmapTerrainSource::Config{});
  ProblemSpec::Params p;
  p.modal = Modal::Hybrid;
  const ProblemSpec spec(&grid, terrain, &m, p);

  const double ground = m.cost(m.groundMove(1.0));
  const double air = m.cost(m.airMoveHorizontal(1.0, 1e3));
  ASSERT_LT(air, ground) << "전제: 1m 비행이 1m 주행보다 싸다";
  EXPECT_LE(spec.unitCostMin(), air + 1e-12);

  // 비행할 수 없는 modal 은 지상 단가 그대로 (더 타이트한 하한)
  p.modal = Modal::RoverDetour;
  const ProblemSpec rover(&grid, terrain, &m, p);
  EXPECT_DOUBLE_EQ(rover.unitCostMin(), ground);
}

// ---------------------------------------------------------------------------
// 출발점 INSCRIBED 완화 · 목표 허용 반경
// ---------------------------------------------------------------------------
TEST(StartGoalTest, RelaxOpensInscribedOnlyNearStart)
{
  // 로봇이 벽에 붙어 서 있는 상황: 출발 칸과 그 주변이 253
  nav2_costmap_2d::Costmap2D master(kNx, kNy, kRes, 0.0, 0.0, kCostFree);
  nav2_costmap_2d::Costmap2D grade(kNx, kNy, kRes, 0.0, 0.0, kCostFree);
  for (unsigned int mx = 4; mx <= 12; ++mx) {
    master.setCost(mx, 5, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  }
  master.setCost(8, 6, nav2_costmap_2d::LETHAL_OBSTACLE);   // 벽 자체

  LayerTerrainSource src(&grade, &master, LayerTerrainSource::Config{});
  EXPECT_TRUE(src.collides(8, 5)) << "완화 전에는 충돌";

  src.relaxInscribedAround(8, 5, 2);
  EXPECT_FALSE(src.collides(8, 5)) << "출발 칸";
  EXPECT_FALSE(src.collides(10, 5)) << "반경 2칸 안";
  EXPECT_TRUE(src.collides(11, 5)) << "반경 밖의 253 은 그대로 충돌";
  EXPECT_TRUE(src.collides(8, 6)) << "LETHAL 은 완화하지 않는다";

  src.relaxInscribedAround(0, 0, 0);
  EXPECT_TRUE(src.collides(8, 5)) << "반경 0 이면 해제";
}

TEST_F(StateSpaceTest, NearestGroundCellFindsClosestReachableCell)
{
  const ProblemSpec spec = makeSpec(Modal::Hybrid);
  unsigned int ox = 0, oy = 0;
  // 장애물 열(kWallX) 위의 목표 -> 바로 옆 칸
  ASSERT_TRUE(spec.nearestGroundCell(kWallX, kMidY, 3, ox, oy));
  EXPECT_EQ(oy, kMidY);
  EXPECT_TRUE(ox == kWallX - 1 || ox == kWallX + 1);
  // 이미 갈 수 있는 칸이면 그대로
  ASSERT_TRUE(spec.nearestGroundCell(2, 2, 3, ox, oy));
  EXPECT_EQ(ox, 2u);
  EXPECT_EQ(oy, 2u);
  // 반경 0 이면 대신 쓸 칸이 없다
  EXPECT_FALSE(spec.nearestGroundCell(kWallX, kMidY, 0, ox, oy));
}


// ---------------------------------------------------------------------------
// 경로 여유 (ProblemSpec::updateClearance 등)
//
// 2026-10-02 시뮬에서 경로가 상자 모서리를 바짝 돌아 로봇이 거기 박혔다.
// 같은 맵(0.60 m fly_over 벽 한 줄)으로 시험한다. 이 벽은
//   우회·하이브리드 (로버 0.15 m) -> 설 수 없는 장애물 -> 여유를 둔다
//   밟고넘기       (로버 0.70 m) -> 올라탈 곳        -> 여유를 두지 않는다
// 벽에서 k 칸 떨어진 칸의 거리는 k × 0.05 m 다.
// ---------------------------------------------------------------------------
class ClearanceTest : public StateSpaceTest
{
protected:
  ProblemSpec makeClearanceSpec(
    Modal modal, double min_clearance, double weight, double range = 1.0) const
  {
    ProblemSpec::Params p;
    p.flight_clearance = 0.8;
    p.ceiling_height = 2.5;
    p.modal = modal;
    p.min_ground_clearance = min_clearance;
    p.clearance_weight = weight;
    p.clearance_decay = 2.0;
    p.clearance_range = range;
    return ProblemSpec(costmap_.get(), terrain_, &model_, p);
  }
};

TEST_F(ClearanceTest, CellsTooCloseToObstacleAreNotDrivableOrLandable)
{
  const ProblemSpec spec = makeClearanceSpec(Modal::Hybrid, 0.30, 0.0);
  for (unsigned int k = 1; k <= 5; ++k) {   // 0.05 ~ 0.25 m
    EXPECT_FALSE(spec.groundOk(kWallX - k, kMidY)) << "k=" << k;
  }
  EXPECT_TRUE(spec.groundOk(kWallX - 6, kMidY)) << "0.30 m 는 허용";
  EXPECT_TRUE(spec.groundOk(kWallX - 7, kMidY));
  // 착륙도 같은 제약 — 장애물 바로 옆에 내려앉지 않는다
  EXPECT_FALSE(spec.landingOk(kWallX + 3, kMidY));
  EXPECT_TRUE(spec.landingOk(kWallX + 6, kMidY));
}

TEST_F(ClearanceTest, ClimbModalKeepsNoClearanceFromClimbableObstacle)
{
  // 밟고넘기에서 0.60 m 벽은 올라탈 곳이다. 앞에서 막으면 밟고넘기가 성립하지 않는다.
  const ProblemSpec climb = makeClearanceSpec(Modal::RoverClimb, 0.30, 1.0);
  EXPECT_TRUE(climb.groundOk(kWallX - 1, kMidY));
  EXPECT_TRUE(climb.groundOk(kWallX, kMidY)) << "벽 위에 올라설 수 있어야 한다";
  EXPECT_NEAR(climb.clearancePenaltyPerMeter(kWallX - 1, kMidY), 0.0, 1e-12);

  // 대조군 — 같은 설정의 우회에서는 막힌다
  const ProblemSpec detour = makeClearanceSpec(Modal::RoverDetour, 0.30, 1.0);
  EXPECT_FALSE(detour.groundOk(kWallX - 1, kMidY));
}

TEST_F(ClearanceTest, PenaltyIsLargestAtMinimumAndZeroBeyondRange)
{
  const ProblemSpec spec = makeClearanceSpec(Modal::Hybrid, 0.30, 1.0);
  EXPECT_NEAR(spec.clearancePenaltyPerMeter(kWallX - 6, kMidY), 1.0, 1e-5);             // 0.30 m
  EXPECT_NEAR(spec.clearancePenaltyPerMeter(kWallX - 12, kMidY), std::exp(-0.6), 1e-5);  // 0.60 m

  // 범위 밖은 0. 경계(정확히 range)는 float 누적 오차로 어느 쪽이든 될 수 있어 피한다.
  const ProblemSpec short_range = makeClearanceSpec(Modal::Hybrid, 0.30, 1.0, 0.8);
  EXPECT_GT(short_range.clearancePenaltyPerMeter(kWallX - 15, kMidY), 0.0);         // 0.75 m
  EXPECT_NEAR(short_range.clearancePenaltyPerMeter(kWallX - 17, kMidY), 0.0, 1e-12);  // 0.85 m

  // 같은 길이의 이동이라도 장애물 옆이 더 비싸다 (벽과 나란히 한 칸)
  const auto near = spec.groundEdge(kWallX - 6, kMidY, kWallX - 6, kMidY + 1, kRes);
  const auto far = spec.groundEdge(1, kMidY, 1, kMidY + 1, kRes);
  EXPECT_NEAR(near.clearance_penalty, 1.0 * kRes, 1e-6);
  EXPECT_GT(model_.cost(near), model_.cost(far));
  // 에너지·시간은 그대로다 — 여유 비용은 별도 항
  EXPECT_NEAR(near.eTotal(), far.eTotal(), 1e-12);
  EXPECT_NEAR(near.time_s, far.time_s, 1e-12);
}

TEST_F(ClearanceTest, ZeroSettingsKeepPreviousBehaviour)
{
  // 기본(0) 이면 예전과 같아야 한다 — 벤치마크·다른 테스트가 이 경로를 탄다
  const ProblemSpec spec = makeSpec(Modal::Hybrid);
  EXPECT_TRUE(spec.groundOk(kWallX - 1, kMidY));
  const auto e = spec.groundEdge(kWallX - 1, kMidY, kWallX - 1, kMidY + 1, kRes);
  EXPECT_NEAR(e.clearance_penalty, 0.0, 1e-12);
  EXPECT_NEAR(model_.cost(e), model_.cost(model_.groundMove(kRes)), 1e-12);
}

TEST_F(ClearanceTest, StartRelaxLetsRobotLeaveObstacleSide)
{
  // 로봇이 이미 벽에서 0.10 m 에 서 있다 — 풀어 주지 않으면 출발 칸부터 막힌다
  ProblemSpec spec = makeClearanceSpec(Modal::Hybrid, 0.30, 1.0);
  ASSERT_FALSE(spec.groundOk(kWallX - 2, kMidY));

  spec.relaxClearanceAround(kWallX - 2, kMidY, 7);   // 0.35 m
  EXPECT_TRUE(spec.groundOk(kWallX - 2, kMidY));
  EXPECT_FALSE(spec.groundOk(kWallX - 2, kMidY + 8)) << "반경 밖은 그대로 막힌다";

  spec.relaxClearanceAround(kWallX - 2, kMidY, 0);   // 해제
  EXPECT_FALSE(spec.groundOk(kWallX - 2, kMidY));
}

TEST_F(ClearanceTest, SegmentPenaltyMatchesGridEdges)
{
  // 스무딩한 직선(segmentClearancePenalty)과 격자 경로(groundEdge)가 같은 값을 내야
  // 스무딩이 모서리를 다시 자르지 않는다. 벽 쪽으로 다가가는 가로 구간으로 잰다.
  const ProblemSpec spec = makeClearanceSpec(Modal::Hybrid, 0.30, 1.0);
  const unsigned int x0 = kWallX - 16, x1 = kWallX - 6;
  double grid = 0.0;
  for (unsigned int x = x0; x < x1; ++x) {
    grid += spec.groundEdge(x, kMidY, x + 1, kMidY, kRes).clearance_penalty;
  }
  double ax, ay, bx, by;
  costmap_->mapToWorld(x0, kMidY, ax, ay);
  costmap_->mapToWorld(x1, kMidY, bx, by);
  EXPECT_NEAR(spec.segmentClearancePenalty(ax, ay, bx, by), grid, 1e-9);
  EXPECT_GT(grid, 0.0);
}

TEST_F(ClearanceTest, HybridStillCrossesByFlightWithClearance)
{
  // 여유를 지켜도 이륙·착륙할 칸이 남아 비행으로 건너야 한다 (이 맵엔 우회로가 없다)
  const ProblemSpec spec = makeClearanceSpec(Modal::Hybrid, 0.30, 1.0);
  const auto seen = reachable(spec, State{1, kMidY, GROUND, 0});
  const bool landed_across = std::any_of(
    seen.begin(), seen.end(),
    [](const auto & k) {return std::get<0>(k) > kWallX && std::get<2>(k) == GROUND;});
  EXPECT_TRUE(landed_across);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
