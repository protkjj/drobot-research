// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// '팔이 로버 자세인가' 판정 — ElevationLayer 가 카메라 높이를 넣을지 정한다.
//
// 2026-11-26: 드론 자세에서 몸체가 바닥에 내려앉아(−0.105 m) 카메라가 바닥을 +10 cm 로
// 쟀고, 그 경계가 가짜 턱이 되어 바퀴로 갈 수 있는 곳을 날아갔다.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "drobot_costmap_2_5d/elevation_layer.hpp"

using drobot_costmap_2_5d::maxJointDeviation;

namespace
{
const std::vector<std::string> kArms = {"lf_arm_rev", "lr_arm_rev", "rf_arm_rev", "rr_arm_rev"};
const std::vector<double> kRover = {0.0, 0.0, 0.0, 0.0};
// /joint_states 순서 그대로 — 바퀴가 섞여 있고 순서가 kArms 와 다르다
const std::vector<std::string> kNames = {
  "lf_arm_rev", "lf_wheel_con", "lr_wheel_con", "lr_arm_rev",
  "rf_arm_rev", "rf_wheel_con", "rr_wheel_con", "rr_arm_rev"};
}  // namespace

TEST(RoverPose, AtRoverPoseDeviationIsSmall)
{
  const std::vector<double> pos = {0.001, -5.8, -5.8, 0.0, 0.002, 66.9, 66.9, -0.001};
  EXPECT_NEAR(maxJointDeviation(kNames, pos, kArms, kRover), 0.002, 1e-9);
}

TEST(RoverPose, WheelAnglesAreIgnored)
{
  // 바퀴 각도(수십 rad)가 판정에 섞이면 늘 '로버 아님' 이 된다
  const std::vector<double> pos = {0.0, 97.2, 97.2, 0.0, 0.0, -19.2, -19.2, 0.0};
  EXPECT_DOUBLE_EQ(maxJointDeviation(kNames, pos, kArms, kRover), 0.0);
}

TEST(RoverPose, HalfReturnedArmsAreFarFromRover)
{
  // 시뮬 실측: 착륙 뒤 로버 변신이 timeout 으로 끝난 자세 (몸체 −0.10 m)
  const std::vector<double> pos = {0.567, -5.8, -5.8, 0.297, 0.518, 66.9, 66.9, 0.123};
  EXPECT_NEAR(maxJointDeviation(kNames, pos, kArms, kRover), 0.567, 1e-9);
}

TEST(RoverPose, DroneTargetIsMeasuredAgainstItsOwnPositions)
{
  const std::vector<double> drone = {1.5708, 1.5708, 1.5708, 1.5708};
  const std::vector<double> pos = {1.49, 0.0, 0.0, 1.44, 1.21, 0.0, 0.0, 1.32};
  EXPECT_NEAR(maxJointDeviation(kNames, pos, kArms, drone), 1.5708 - 1.21, 1e-9);
}

TEST(RoverPose, MissingJointIsNegative)
{
  // 바퀴만 담은 메시지 — 판정하지 않는다
  const std::vector<std::string> names = {"lf_wheel_con", "rf_wheel_con"};
  const std::vector<double> pos = {1.0, 2.0};
  EXPECT_LT(maxJointDeviation(names, pos, kArms, kRover), 0.0);
}

TEST(RoverPose, ShortPositionArrayIsNegative)
{
  const std::vector<double> pos = {0.0, 0.0};   // name 보다 짧다
  EXPECT_LT(maxJointDeviation(kNames, pos, kArms, kRover), 0.0);
}
