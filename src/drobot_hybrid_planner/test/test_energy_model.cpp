// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// Python↔C++ 동등성 테스트
//
// C++ EnergyModel 이 Python 벤치마크(benchmark/cost/energy.py)와
// 같은 입력에 같은 값을 내는지 확인한다.
//
// 왜 필요한가:
//   A* 채택 근거는 Python 벤치마크에서 나왔다. C++ 이식 과정에서 수식이
//   미묘하게 달라지면 그 결론을 C++ 구현에 인용할 수 없게 된다.
//   기댓값은 benchmark/gen_cpp_fixtures.py 가 Python 쪽에서 뽑아 둔 것이다.
//
// 기댓값 재생성:
//   python3 benchmark/gen_cpp_fixtures.py
//   (energy_params.yaml 을 바꿨다면 반드시 다시 돌릴 것)

// cpplint 규칙상 C 헤더(.h)가 C++ 헤더보다 앞에 와야 한다.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "drobot_hybrid_planner/energy_model.hpp"
#include "energy_fixtures.hpp"

using drobot_hybrid_planner::CostAccumulator;
using drobot_hybrid_planner::EnergyModel;

namespace
{
// 부동소수 비교 허용 오차.
// Python(float64)과 C++(double)은 같은 IEEE754 배정밀도라 연산 순서만 같으면
// 비트 단위로 일치한다. 여유를 조금 두되 느슨하지 않게 잡는다.
constexpr double kTol = 1e-9;
}  // namespace


class EnergyModelTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("energy_model_test");

    // energy_params.yaml 과 같은 값을 파라미터로 선언한다.
    // (테스트에서 yaml 파일을 로드하지 않고 직접 넣는 이유:
    //  파일 경로 의존성을 없애 어느 환경에서든 같은 결과가 나오게 하기 위함)
    const std::string p = "energy_model.";
    node_->declare_parameter(p + "cost_weights.w_energy", 0.5);
    node_->declare_parameter(p + "cost_weights.w_switch", 0.2);
    node_->declare_parameter(p + "cost_weights.w_time", 0.3);
    node_->declare_parameter(p + "ground_mode.energy_per_meter", 0.5);
    node_->declare_parameter(p + "ground_mode.speed", 0.3);
    node_->declare_parameter(p + "air_mode.energy_per_meter", 2.0);
    node_->declare_parameter(p + "air_mode.hover_power", 50.0);
    node_->declare_parameter(p + "air_mode.speed", 0.5);
    node_->declare_parameter(p + "climb_mode.max_height", 0.7);
    node_->declare_parameter(p + "climb_mode.energy_per_height_m", 2.0);
    node_->declare_parameter(p + "climb_mode.speed_factor", 0.4);
    node_->declare_parameter(p + "mode_switch.takeoff_energy", 5.0);
    node_->declare_parameter(p + "mode_switch.takeoff_time", 5.0);
    node_->declare_parameter(p + "mode_switch.landing_energy", 3.0);
    node_->declare_parameter(p + "mode_switch.landing_time", 4.0);
    node_->declare_parameter(p + "mode_switch.energy_per_altitude_meter", 2.0);
    node_->declare_parameter(p + "ground_effect.enabled", true);
    node_->declare_parameter(p + "ground_effect.activation_height", 0.5);
    node_->declare_parameter(p + "ground_effect.power_multiplier", 1.15);
    node_->declare_parameter(p + "vertical_motion.climb_energy_per_meter", 2.0);
    node_->declare_parameter(p + "vertical_motion.descent_energy_ratio", 0.3);
    node_->declare_parameter(p + "vertical_motion.climb_speed", 0.5);
    node_->declare_parameter(p + "vertical_motion.descent_speed", 0.5);
    node_->declare_parameter(p + "apply_altitude_cost_to_landing", true);
    node_->declare_parameter(p + "robot.height", 0.25);
    node_->declare_parameter(p + "robot.ceiling_margin", 0.5);
    node_->declare_parameter(p + "min_flight_clearance", 0.2);
    node_->declare_parameter(p + "max_climb_angle_deg", 45.0);
    node_->declare_parameter(p + "idle_power.enabled", false);
    node_->declare_parameter(p + "idle_power.ground_w", 0.0);
    node_->declare_parameter(p + "idle_power.air_w", 0.0);

    model_.configure(node_, "energy_model");
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  EnergyModel model_;
};


// ---------------------------------------------------------------------------
// 참조값과 등가 계수
// ---------------------------------------------------------------------------
TEST_F(EnergyModelTest, ReferenceValuesMatchPython)
{
  EXPECT_NEAR(model_.eRef(), drobot_test::kERef, kTol);
  EXPECT_NEAR(model_.eSwitchRef(), drobot_test::kESwitchRef, kTol);
  EXPECT_NEAR(model_.tRef(), drobot_test::kTRef, kTol);
}

TEST_F(EnergyModelTest, EquivalentAlphaBetaGammaMatchPython)
{
  // 정규화 형태와 alpha/beta/gamma 형태가 동등한지 확인.
  // 논문에서 두 식을 연결해 설명하려면 이 변환이 정확해야 한다.
  EXPECT_NEAR(model_.alpha(), drobot_test::kAlpha, kTol);
  EXPECT_NEAR(model_.beta(), drobot_test::kBeta, kTol);
  EXPECT_NEAR(model_.gamma(), drobot_test::kGamma, kTol);
}

TEST_F(EnergyModelTest, NormalizedAndAlphaBetaGammaFormsAgree)
{
  // C = wE·(E_motion + E_switch)/E_ref + wS·n_switch + wT·T/T_ref
  //   = alpha·(E_motion + E_switch) + beta·n_switch + gamma·T
  // 두 식이 같은 값을 내는지 임의의 누적값으로 확인한다.
  CostAccumulator a;
  a.e_ground = 10.0;
  a.e_air_horiz = 3.0;
  a.e_switch = 8.0;
  a.n_takeoff = 1;
  a.n_landing = 1;
  a.time_s = 50.0;

  const double by_alpha =
    model_.alpha() * (a.eMotion() + a.e_switch) +
    model_.beta() * static_cast<double>(a.nSwitches()) +
    model_.gamma() * a.time_s;

  EXPECT_NEAR(model_.cost(a), by_alpha, kTol);
}

TEST_F(EnergyModelTest, SwitchEnergyCostsSameAsMotionEnergy)
{
  // 같은 1 Wh 는 어디서 쓰든 같은 비용이어야 한다.
  //
  // 이전 수식은 E_switch 를 E_switch_ref(8.0 Wh)로 따로 나눠서
  // 전환 에너지가 주행 에너지보다 40배 싸게 계상됐다. 그 결과
  // 총 52.9 Wh 쓰는 경로가 14.1 Wh 쓰는 경로를 이기고 '최적'으로 뽑혔다.
  // 이 테스트가 그 회귀를 막는다.
  CostAccumulator by_motion;
  by_motion.e_ground = 1.0;

  CostAccumulator by_switch;
  by_switch.e_switch = 1.0;

  EXPECT_NEAR(model_.cost(by_motion), model_.cost(by_switch), kTol);
}


// ---------------------------------------------------------------------------
// 원자 연산
// ---------------------------------------------------------------------------
TEST_F(EnergyModelTest, RoverClimbMoveMatchesPython)
{
  for (int i = 0; i < drobot_test::kNumRoverClimbCases; ++i) {
    const auto & c = drobot_test::kRoverClimbCases[i];
    const CostAccumulator a = model_.roverClimbMove(c.dist, c.delta_h);

    EXPECT_NEAR(a.e_ground, c.e_wh, kTol)
      << "dist=" << c.dist << " dh=" << c.delta_h;
    EXPECT_NEAR(a.time_s, c.time_s, kTol)
      << "dist=" << c.dist << " dh=" << c.delta_h;
    EXPECT_NEAR(model_.cost(a), c.cost, kTol)
      << "dist=" << c.dist << " dh=" << c.delta_h;
    EXPECT_NEAR(a.dist_ground, c.dist, kTol);
  }
}

TEST_F(EnergyModelTest, RoverClimbChargesOnAscentOnly)
{
  // 등반 비용은 '높이 차이'에만 붙는다.
  // 목적지 높이에 비례해 부과하면 장애물 위를 여러 셀 지나갈 때
  // 같은 값이 중복 계상되기 때문이다.
  const double d = 0.1;
  const CostAccumulator flat = model_.roverClimbMove(d, 0.0);
  const CostAccumulator down = model_.roverClimbMove(d, -0.55);
  const CostAccumulator plain = model_.groundMove(d);

  // 평지 이동과 하강은 평지 주행과 완전히 같아야 한다
  EXPECT_NEAR(flat.e_ground, plain.e_ground, kTol);
  EXPECT_NEAR(flat.time_s, plain.time_s, kTol);
  EXPECT_NEAR(down.e_ground, plain.e_ground, kTol);
  EXPECT_NEAR(down.time_s, plain.time_s, kTol);

  // 상승만 추가 비용이 붙는다
  const CostAccumulator up = model_.roverClimbMove(d, 0.55);
  EXPECT_GT(up.e_ground, plain.e_ground);
  EXPECT_NEAR(
    up.e_ground - plain.e_ground,
    drobot_test::kRoverClimbWhPerM * 0.55, kTol);
  // 등반 중에는 느려진다
  EXPECT_GT(up.time_s, plain.time_s);
}

TEST_F(EnergyModelTest, RoverClimbIsPathIndependent)
{
  // 0.55m 장애물을 한 번에 오르든 나눠 오르든 총 등반 에너지는 같아야 한다
  // (위치에너지와 같은 경로 무관 구조).
  const double d = 0.1;
  const CostAccumulator once = model_.roverClimbMove(d, 0.55);

  CostAccumulator split;
  split += model_.roverClimbMove(d, 0.30);
  split += model_.roverClimbMove(d, 0.25);

  // 거리가 2배이므로 주행분(ground_wh_per_m * d)만큼 차이가 난다
  const double drive_extra = 0.5 * d;
  EXPECT_NEAR(split.e_ground - drive_extra, once.e_ground, kTol);
}
TEST_F(EnergyModelTest, GroundMoveMatchesPython)
{
  for (int i = 0; i < drobot_test::kNumGroundCases; ++i) {
    const auto & c = drobot_test::kGroundCases[i];
    const CostAccumulator a = model_.groundMove(c.dist);

    EXPECT_NEAR(a.e_ground, c.e_wh, kTol) << "dist=" << c.dist;
    EXPECT_NEAR(a.time_s, c.time_s, kTol) << "dist=" << c.dist;
    EXPECT_NEAR(model_.cost(a), c.cost, kTol) << "dist=" << c.dist;
    // 거리도 기록되는지
    EXPECT_NEAR(a.dist_ground, c.dist, kTol);
  }
}

TEST_F(EnergyModelTest, AirMoveHorizontalMatchesPython)
{
  for (int i = 0; i < drobot_test::kNumAirCases; ++i) {
    const auto & c = drobot_test::kAirCases[i];
    const CostAccumulator a = model_.airMoveHorizontal(c.dist, c.clearance);

    EXPECT_NEAR(a.e_air_horiz, c.e_wh, kTol)
      << "dist=" << c.dist << " clearance=" << c.clearance;
    EXPECT_NEAR(a.time_s, c.time_s, kTol);
    EXPECT_NEAR(model_.cost(a), c.cost, kTol);
  }
}

TEST_F(EnergyModelTest, GroundEffectAppliesBelowThresholdOnly)
{
  // clearance 가 activation_height(0.5) 이하일 때만 배수가 붙어야 한다.
  // 경계 바로 위/아래를 확인한다 — 부등호 방향 실수를 잡기 위함.
  const CostAccumulator below = model_.airMoveHorizontal(1.0, 0.50);
  const CostAccumulator above = model_.airMoveHorizontal(1.0, 0.51);

  EXPECT_GT(below.e_air_horiz, above.e_air_horiz)
    << "clearance <= 0.5 에서 ground effect 가 적용되어야 한다";
  EXPECT_NEAR(below.e_air_horiz / above.e_air_horiz, 1.15, 1e-6);
}

TEST_F(EnergyModelTest, AirMoveVerticalMatchesPython)
{
  for (int i = 0; i < drobot_test::kNumVertCases; ++i) {
    const auto & c = drobot_test::kVertCases[i];
    const CostAccumulator a = model_.airMoveVertical(c.dz);

    EXPECT_NEAR(a.e_air_vert, c.e_wh, kTol) << "dz=" << c.dz;
    EXPECT_NEAR(a.time_s, c.time_s, kTol) << "dz=" << c.dz;
    EXPECT_NEAR(model_.cost(a), c.cost, kTol) << "dz=" << c.dz;
  }
}

TEST_F(EnergyModelTest, DescentIsCheaperThanClimb)
{
  // 하강이 상승보다 싸야 한다 (descent_energy_ratio = 0.3)
  const CostAccumulator up = model_.airMoveVertical(1.0);
  const CostAccumulator down = model_.airMoveVertical(-1.0);
  EXPECT_LT(down.e_air_vert, up.e_air_vert);
  EXPECT_NEAR(down.e_air_vert / up.e_air_vert, 0.3, 1e-9);
}

TEST_F(EnergyModelTest, TakeoffLandingMatchPython)
{
  for (int i = 0; i < drobot_test::kNumSwitchCases; ++i) {
    const auto & c = drobot_test::kSwitchCases[i];

    const CostAccumulator t = model_.takeoff(c.altitude);
    EXPECT_NEAR(t.e_switch, c.takeoff_wh, kTol) << "alt=" << c.altitude;
    EXPECT_NEAR(model_.cost(t), c.takeoff_cost, kTol) << "alt=" << c.altitude;
    EXPECT_EQ(t.n_takeoff, 1);
    EXPECT_EQ(t.n_landing, 0);

    const CostAccumulator l = model_.landing(c.altitude);
    EXPECT_NEAR(l.e_switch, c.landing_wh, kTol) << "alt=" << c.altitude;
    EXPECT_NEAR(model_.cost(l), c.landing_cost, kTol) << "alt=" << c.altitude;
    EXPECT_EQ(l.n_takeoff, 0);
    EXPECT_EQ(l.n_landing, 1);
  }
}


// ---------------------------------------------------------------------------
// 제약
// ---------------------------------------------------------------------------
TEST_F(EnergyModelTest, MaxFlightAltitudeMatchesPython)
{
  for (int i = 0; i < drobot_test::kNumMaxAltCases; ++i) {
    const auto & c = drobot_test::kMaxAltCases[i];
    EXPECT_NEAR(model_.maxFlightAltitude(c.input), c.expected, kTol)
      << "ceiling=" << c.input;
  }
}

TEST_F(EnergyModelTest, MaxDzForMatchesPython)
{
  // 격자 해상도가 아니라 각도에서 나오는 값이어야 한다.
  // 이게 틀리면 z해상도를 높일수록 해가 나빠지는 버그가 재발한다.
  for (int i = 0; i < drobot_test::kNumMaxDzCases; ++i) {
    const auto & c = drobot_test::kMaxDzCases[i];
    EXPECT_NEAR(model_.maxDzFor(c.input), c.expected, kTol)
      << "horizontal_dist=" << c.input;
  }
}

TEST_F(EnergyModelTest, MaxDzScalesLinearlyWithDistance)
{
  // 45도이므로 수평거리와 최대 고도차가 같아야 한다
  EXPECT_NEAR(model_.maxDzFor(1.0), 1.0, 1e-9);
  EXPECT_NEAR(model_.maxDzFor(2.0), 2.0, 1e-9);
}


// ---------------------------------------------------------------------------
// 복합 시나리오 — 누적 연산에서도 일치하는지
// ---------------------------------------------------------------------------
TEST_F(EnergyModelTest, FlightScenarioMatchesPython)
{
  for (int i = 0; i < drobot_test::kNumScenarioCases; ++i) {
    const auto & c = drobot_test::kScenarioCases[i];

    CostAccumulator acc;
    acc += model_.takeoff(c.altitude);
    acc += model_.airMoveHorizontal(c.dist, c.clearance);
    acc += model_.landing(c.altitude);

    EXPECT_NEAR(acc.eTotal(), c.e_total_wh, kTol)
      << "alt=" << c.altitude << " dist=" << c.dist;
    EXPECT_NEAR(acc.time_s, c.time_s, kTol);
    EXPECT_NEAR(model_.cost(acc), c.cost, kTol);
    EXPECT_EQ(acc.nSwitches(), 2);
  }
}

TEST_F(EnergyModelTest, AccumulatorAdditionIsAssociative)
{
  // (a + b) + c == a + (b + c) — 누적 순서가 결과를 바꾸면 안 된다
  const CostAccumulator a = model_.groundMove(1.0);
  const CostAccumulator b = model_.takeoff(0.8);
  const CostAccumulator c = model_.airMoveHorizontal(2.0, 0.8);

  const CostAccumulator left = (a + b) + c;
  const CostAccumulator right = a + (b + c);

  EXPECT_NEAR(model_.cost(left), model_.cost(right), kTol);
  EXPECT_NEAR(left.eTotal(), right.eTotal(), kTol);
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
