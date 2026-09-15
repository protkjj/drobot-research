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
#include <memory>
#include <queue>
#include <set>
#include <tuple>
#include <vector>

#include <nav2_costmap_2d/costmap_2d.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "drobot_hybrid_planner/state_space.hpp"

using drobot_hybrid_planner::AIR;
using drobot_hybrid_planner::CostmapTerrainSource;
using drobot_hybrid_planner::EnergyModel;
using drobot_hybrid_planner::GROUND;
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


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
