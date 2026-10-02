// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// 컨트롤러로 나가는 nav_msgs/Path 의 모양 — densifyPath.
//
// 예전에는 스무딩한 꺾임점 몇 개만, 방향은 전부 yaw 0 으로 냈다. DWB 가 점 사이
// 구간을 경로로 보지 못해 로봇이 제자리에서 좌우로 흔들렸다 (base_map_h0.5,
// 205 s 동안 13 바퀴, 회전만 76 %). 아래가 그 회귀를 막는다.

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "drobot_hybrid_planner/hybrid_astar_planner.hpp"

using drobot_hybrid_planner::AIR;
using drobot_hybrid_planner::densifyPath;
using drobot_hybrid_planner::GROUND;
using drobot_hybrid_planner::Waypoint;

namespace
{
constexpr double kStep = 0.05;

double yawOf(const geometry_msgs::msg::PoseStamped & ps)
{
  const auto & q = ps.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double gap(const geometry_msgs::msg::PoseStamped & a, const geometry_msgs::msg::PoseStamped & b)
{
  return std::hypot(b.pose.position.x - a.pose.position.x, b.pose.position.y - a.pose.position.y);
}
}  // namespace


TEST(PathMsg, ConsecutivePosesAreAtMostOneStepApart)
{
  // 꺾임점 3개 — (0,0) -> (1,0) -> (1,2)
  const std::vector<Waypoint> wps{{0, 0, 0, GROUND}, {1, 0, 0, GROUND}, {1, 2, 0, GROUND}};
  const auto msg = densifyPath(wps, kStep, std_msgs::msg::Header{});

  ASSERT_GE(msg.poses.size(), 60u);   // 3 m / 0.05
  for (size_t i = 0; i + 1 < msg.poses.size(); ++i) {
    EXPECT_LE(gap(msg.poses[i], msg.poses[i + 1]), kStep + 1e-9) << "i=" << i;
  }
  // 꺾임점과 끝점은 그대로 들어간다
  EXPECT_DOUBLE_EQ(msg.poses.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(msg.poses.back().pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(msg.poses.back().pose.position.y, 2.0);
}

TEST(PathMsg, OrientationFollowsTravelDirection)
{
  const std::vector<Waypoint> wps{{0, 0, 0, GROUND}, {1, 0, 0, GROUND}, {1, 2, 0, GROUND}};
  const auto msg = densifyPath(wps, kStep, std_msgs::msg::Header{});

  for (const auto & ps : msg.poses) {
    const double want = (ps.pose.position.y < 1e-9 && ps.pose.position.x < 1.0 - 1e-9) ?
      0.0 : M_PI / 2.0;
    EXPECT_NEAR(yawOf(ps), want, 1e-9)
      << "(" << ps.pose.position.x << ", " << ps.pose.position.y << ")";
  }
}

TEST(PathMsg, TakeoffAndLandingKeepHeadingAndAltitude)
{
  // 지상 -> 같은 자리에서 이륙 -> 0.8 m 로 1 m 비행 -> 같은 자리에서 착륙
  const std::vector<Waypoint> wps{
    {0, 0, 0.0, GROUND}, {0, 1, 0.0, GROUND}, {0, 1, 0.8, AIR},
    {0, 2, 0.8, AIR}, {0, 2, 0.0, GROUND}};
  const auto msg = densifyPath(wps, kStep, std_msgs::msg::Header{});

  bool saw_air = false;
  for (const auto & ps : msg.poses) {
    EXPECT_NEAR(yawOf(ps), M_PI / 2.0, 1e-9) << "제자리 이착륙도 진행 방향을 잇는다";
    if (ps.pose.position.y > 1.0 + 1e-9 && ps.pose.position.y < 2.0 - 1e-9) {
      EXPECT_DOUBLE_EQ(ps.pose.position.z, 0.8) << "비행 구간 고도 유지";
      saw_air = true;
    }
  }
  EXPECT_TRUE(saw_air);
  EXPECT_DOUBLE_EQ(msg.poses.back().pose.position.z, 0.0);
}

TEST(PathMsg, SinglePointAndEmpty)
{
  EXPECT_TRUE(densifyPath({}, kStep, std_msgs::msg::Header{}).poses.empty());
  const auto one = densifyPath({{1, 1, 0, GROUND}}, kStep, std_msgs::msg::Header{});
  ASSERT_EQ(one.poses.size(), 1u);
  EXPECT_DOUBLE_EQ(one.poses[0].pose.orientation.w, 1.0);
}
