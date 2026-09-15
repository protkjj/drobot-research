// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// 자동 생성 파일 — 직접 수정하지 말 것.
// 생성: python3 benchmark/gen_cpp_fixtures.py
//
// Python 벤치마크(benchmark/cost/energy.py)의 기댓값이다.
// C++ EnergyModel 이 같은 입력에 같은 값을 내는지 대조하는 데 쓴다.
// energy_params.yaml 을 바꾸면 이 파일을 다시 생성해야 한다.

#ifndef ENERGY_FIXTURES_HPP_
#define ENERGY_FIXTURES_HPP_

namespace drobot_test
{

// 참조값 (파라미터에서 유도)
constexpr double kERef = 0.5;
constexpr double kESwitchRef = 8.0;
constexpr double kTRef = 3.3333333333333335;

// 로버 등반 파라미터
constexpr double kRoverClimbMaxH = 0.7;
constexpr double kRoverClimbWhPerM = 2.0;
constexpr double kRoverClimbSpeedFactor = 0.4;

// 등가 alpha/beta/gamma  (C = alpha*(E_motion+E_switch) + beta*n_switch + gamma*T)
constexpr double kAlpha = 1.0;
constexpr double kBeta = 0.2;
constexpr double kGamma = 0.09;

struct GroundCase
{
  double dist;
  double e_wh;
  double time_s;
  double cost;
};
constexpr GroundCase kGroundCases[] = {
  {0.05, 0.025, 0.16666666666666669, 0.04},
  {0.1, 0.05, 0.33333333333333337, 0.08},
  {0.1414213562, 0.0707106781, 0.47140452066666666, 0.11313708496},
  {1.0, 0.5, 3.3333333333333335, 0.8},
  {5.0, 2.5, 16.666666666666668, 4.0},
};
constexpr int kNumGroundCases = 5;

struct RoverClimbCase
{
  double dist;
  double delta_h;
  double e_wh;
  double time_s;
  double cost;
};
constexpr RoverClimbCase kRoverClimbCases[] = {
  {0.1, 0.55, 1.1500000000000001, 0.8333333333333334, 1.225},
  {0.1, 0.35, 0.75, 0.8333333333333334, 0.825},
  {0.1, 0.7, 1.45, 0.8333333333333334, 1.525},
  {0.1, 0.0, 0.05, 0.33333333333333337, 0.08},
  {0.5, 0.0, 0.25, 1.6666666666666667, 0.4},
  {0.1, -0.55, 0.05, 0.33333333333333337, 0.08},
  {0.1, -0.35, 0.05, 0.33333333333333337, 0.08},
  {0.1414213562373095, 0.6, 1.2707106781186548, 1.1785113019775793, 1.376776695296637},
  {1.0, 0.15, 0.8, 8.333333333333334, 1.55},
};
constexpr int kNumRoverClimbCases = 9;

struct AirCase
{
  double dist;
  double clearance;
  double e_wh;
  double time_s;
  double cost;
};
constexpr AirCase kAirCases[] = {
  {0.1, 0.8, 0.2, 0.2, 0.218},
  {0.1, 0.3, 0.22999999999999998, 0.2, 0.24799999999999997},
  {1.0, 0.5, 2.3, 2.0, 2.48},
  {1.0, 0.51, 2.0, 2.0, 2.18},
  {3.0, 0.2, 6.8999999999999995, 6.0, 7.4399999999999995},
};
constexpr int kNumAirCases = 5;

struct VertCase
{
  double dz;
  double e_wh;
  double time_s;
  double cost;
};
constexpr VertCase kVertCases[] = {
  {0.1, 0.2, 0.2, 0.218},
  {-0.1, 0.06, 0.2, 0.078},
  {0.5, 1.0, 1.0, 1.09},
  {-0.5, 0.3, 1.0, 0.39},
  {1.0, 2.0, 2.0, 2.18},
};
constexpr int kNumVertCases = 5;

struct SwitchCase
{
  double altitude;
  double takeoff_wh;
  double takeoff_cost;
  double landing_wh;
  double landing_cost;
};
constexpr SwitchCase kSwitchCases[] = {
  {0.2, 5.4, 6.050000000000001, 3.4, 3.96},
  {0.35, 5.7, 6.3500000000000005, 3.7, 4.260000000000001},
  {0.8, 6.6, 7.25, 4.6, 5.16},
  {1.2, 7.4, 8.05, 5.4, 5.960000000000001},
  {1.75, 8.5, 9.149999999999999, 6.5, 7.0600000000000005},
};
constexpr int kNumSwitchCases = 5;

struct ConstraintCase
{
  double input;
  double expected;
};
// maxFlightAltitude(ceiling)
constexpr ConstraintCase kMaxAltCases[] = {
  {2.2, 1.4500000000000002},
  {2.5, 1.75},
  {3.0, 2.25},
};
constexpr int kNumMaxAltCases = 3;

// maxDzFor(horizontal_dist)
constexpr ConstraintCase kMaxDzCases[] = {
  {0.1, 0.09999999999999999},
  {0.1414213562, 0.14142135619999996},
  {1.0, 0.9999999999999999},
};
constexpr int kNumMaxDzCases = 3;

// 복합 시나리오: 이륙 -> 수평비행 -> 착륙
struct ScenarioCase
{
  double altitude;
  double dist;
  double clearance;
  double e_total_wh;
  double time_s;
  double cost;
};
constexpr ScenarioCase kScenarioCases[] = {
  {0.35, 2.0, 0.35, 14.0, 13.0, 15.57},
  {0.8, 5.0, 0.6, 21.2, 19.0, 23.31},
  {1.2, 1.0, 0.25, 15.100000000000001, 11.0, 16.490000000000002},
};
constexpr int kNumScenarioCases = 3;

}  // namespace drobot_test

#endif  // ENERGY_FIXTURES_HPP_
