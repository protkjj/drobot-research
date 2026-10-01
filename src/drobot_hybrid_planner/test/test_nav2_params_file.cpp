// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.
//
// nav2 파라미터 파일의 값이 planner_server 의 EnergyModel 에 실제로 도달하는가.
//
// 왜 이 파일이 생겼나
// -------------------
// nav2_params_hybrid*.yaml 에서 energy_model 블록이 최상위 키로 놓여 있었다.
// rcl 은 파라미터 파일의 최상위 키를 '노드 이름'으로 읽는다. 그래서 그 블록은
// /energy_model 이라는 (존재하지 않는) 노드의 섹션이 되어 조용히 버려졌고,
// planner_server 안의 플러그인은 C++ 기본값으로 계획했다.
// C++ 기본값이 default 세트와 우연히 전부 같아서 드러나지 않았고,
// energy:=derived 는 아무 효과가 없었다 (HANDOFF.md 4절).
//
// 그래서 파일 구조가 아니라 '값이 노드에 도달하는가'를 본다.
// navigation.launch.py 와 똑같이 --params-file 로 파일을 받은 'planner_server'
// 노드를 만들고, HybridAStarPlanner::configure 와 같은 호출
// (EnergyModel::configure(node, "energy_model"))로 모델을 채운 뒤,
// 모델이 실제로 쓰는 값이 파일에 적힌 값인지 확인한다.
// 노드 이름 매칭은 rcl 이 한다 — 테스트가 흉내 내지 않는다.
//
// 노드 이름이 실제 서버와 같은 planner_server 이므로, 떠 있는 시뮬레이션과
// 섞이지 않게 ROS_DOMAIN_ID 를 격리해 돌린다 (CMakeLists.txt 참고).

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <rclcpp/parameter_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "drobot_hybrid_planner/energy_model.hpp"

using drobot_hybrid_planner::EnergyModel;

namespace
{
// navigation.launch.py 가 nav2_params 를 넘기는 플래너 노드 이름
constexpr char kPlannerNode[] = "planner_server";

// 파일이 있는 디렉터리. CMakeLists.txt 가 <src>/drobot_bringup/config/navigation 을
// 넣어 준다. DROBOT_NAV_CONFIG_DIR 로 바꿔 끼울 수 있다 — 수정 전 파일을 넣고
// 이 테스트가 실제로 실패하는지 확인할 때 쓴다.
std::string configDir()
{
  const char * env = std::getenv("DROBOT_NAV_CONFIG_DIR");
  return (env && *env) ? env : DROBOT_BRINGUP_NAV_CONFIG_DIR;
}

constexpr char kDefaultFile[] = "nav2_params_hybrid.yaml";
constexpr char kDerivedFile[] = "nav2_params_hybrid_derived.yaml";

constexpr double kTol = 1e-9;

std::string configPath(const std::string & file) {return configDir() + "/" + file;}

/// 런치와 같은 경로로 파일을 받은 planner_server 노드.
rclcpp_lifecycle::LifecycleNode::SharedPtr makePlannerNode(const std::string & path)
{
  rclcpp::NodeOptions opts;
  // launch_ros 의 Node(parameters=[파일]) 은 결국 이 인자가 된다
  opts.arguments({"--ros-args", "--params-file", path});
  // 테스트 프로세스 자체의 명령행 인자가 섞이지 않게 한다
  opts.use_global_arguments(false);
  return std::make_shared<rclcpp_lifecycle::LifecycleNode>(kPlannerNode, opts);
}

/// 파일을 받은 planner_server 로 채운 EnergyModel (hybrid_astar_planner.cpp 와 같은 호출)
EnergyModel modelFrom(const std::string & path)
{
  EnergyModel m;
  m.configure(makePlannerNode(path), "energy_model");
  return m;
}

/// 파일에 적힌 /planner_server 의 energy_model.* 값 (rcl 파서가 읽은 그대로)
std::map<std::string, rclcpp::ParameterValue> fileEnergyValues(const std::string & path)
{
  std::map<std::string, rclcpp::ParameterValue> out;
  const auto pmap = rclcpp::parameter_map_from_yaml_file(path);
  const auto it = pmap.find(std::string("/") + kPlannerNode);
  if (it == pmap.end()) {return out;}
  for (const auto & p : it->second) {
    if (p.get_name().rfind("energy_model.", 0) == 0) {
      out.emplace(p.get_name(), p.get_parameter_value());
    }
  }
  return out;
}
}  // namespace


class Nav2ParamsFileTest : public ::testing::TestWithParam<const char *>
{
protected:
  void SetUp() override
  {
    // 플러그인만 따로 떼어 빌드하면 drobot_bringup 이 옆에 없다. 그때만 건너뛴다.
    if (!std::filesystem::is_directory(configDir())) {
      GTEST_SKIP() << "drobot_bringup 설정 디렉터리가 없다: " << configDir();
    }
    path_ = configPath(GetParam());
    ASSERT_TRUE(std::filesystem::exists(path_)) << path_;
  }

  std::string path_;
};


// ---------------------------------------------------------------------------
// EnergyModel 이 선언하는 energy_model.* 가 전부 파일에서 와야 한다.
//
// 값 비교가 아니라 '출처'를 본다. default 세트는 C++ 기본값과 같아서
// 값만 비교하면 파일이 무시돼도 통과한다 — 실제로 그렇게 숨어 있었다.
// ---------------------------------------------------------------------------
TEST_P(Nav2ParamsFileTest, EveryEnergyParameterComesFromTheFile)
{
  auto node = makePlannerNode(path_);
  EnergyModel model;
  model.configure(node, "energy_model");

  const auto declared = node->list_parameters({"energy_model"}, 0).names;
  ASSERT_FALSE(declared.empty()) << "EnergyModel::configure 가 아무것도 선언하지 않았다";

  const auto & overrides = node->get_node_parameters_interface()->get_parameter_overrides();
  std::vector<std::string> missing;
  for (const auto & name : declared) {
    if (overrides.count(name) == 0) {missing.push_back(name);}
  }
  EXPECT_TRUE(missing.empty())
    << missing.size() << "/" << declared.size()
    << "개가 파일에서 오지 않았다 — C++ 기본값이 쓰인다.\n"
    << "  예: " << (missing.empty() ? "" : missing.front()) << "\n"
    << "  energy_model 블록이 planner_server: ros__parameters: 아래에 있는지 확인할 것.\n"
    << "  최상위 energy_model: 은 /energy_model 노드의 섹션으로 읽혀 버려진다.";
}


// ---------------------------------------------------------------------------
// 플래너가 실제로 쓰는 값 = 파일의 값.
// 노드 파라미터가 아니라 EnergyModel 의 동작으로 확인한다.
// ---------------------------------------------------------------------------
TEST_P(Nav2ParamsFileTest, PlannerUsesTheFileValues)
{
  const auto want = fileEnergyValues(path_);
  ASSERT_FALSE(want.empty())
    << path_ << " 에 /planner_server 의 energy_model.* 가 하나도 없다";
  const auto v = [&want](const std::string & key) {
      const auto it = want.find("energy_model." + key);
      EXPECT_NE(it, want.end()) << "파일에 energy_model." << key << " 가 없다";
      return it == want.end() ? 0.0 : it->second.get<double>();
    };

  const EnergyModel m = modelFrom(path_);

  // 이착륙 — derived 세트가 바꾸는 값. 비행 선택 여부를 통째로 좌우한다.
  EXPECT_NEAR(m.takeoff(0.0).e_switch, v("mode_switch.takeoff_energy"), kTol);
  EXPECT_NEAR(m.landing(0.0).e_switch, v("mode_switch.landing_energy"), kTol);
  EXPECT_NEAR(
    m.takeoff(1.0).e_switch - m.takeoff(0.0).e_switch,
    v("mode_switch.energy_per_altitude_meter"), kTol);
  EXPECT_NEAR(m.takeoff(0.0).time_s, v("mode_switch.takeoff_time"), kTol);
  EXPECT_NEAR(m.landing(0.0).time_s, v("mode_switch.landing_time"), kTol);

  // 비행 — ground effect 가 걸리지 않게 여유를 크게 준다.
  // idle 보정이 켜져 있으면 수평 비행 에너지가 단가와 달라지므로 먼저 확인한다.
  ASSERT_NE(want.count("energy_model.idle_power.enabled"), 0u);
  ASSERT_FALSE(want.at("energy_model.idle_power.enabled").get<bool>());
  EXPECT_NEAR(m.airMoveHorizontal(1.0, 100.0).e_air_horiz, v("air_mode.energy_per_meter"), kTol);
  EXPECT_NEAR(m.airSpeed(), v("air_mode.speed"), kTol);

  // 지상 / 가중치 / 형상
  EXPECT_NEAR(m.eRef(), v("ground_mode.energy_per_meter"), kTol);
  EXPECT_NEAR(m.groundSpeed(), v("ground_mode.speed"), kTol);
  EXPECT_NEAR(m.alpha() * m.eRef(), v("cost_weights.w_energy"), kTol);
  EXPECT_NEAR(m.beta(), v("cost_weights.w_switch"), kTol);
  EXPECT_NEAR(m.gamma() * m.tRef(), v("cost_weights.w_time"), kTol);
  EXPECT_NEAR(m.roverClimbMaxH(), v("climb_mode.max_height"), kTol);
  EXPECT_NEAR(m.minFlightClearance(), v("min_flight_clearance"), kTol);
  EXPECT_NEAR(m.robotHeight(), v("robot.height"), kTol);
}


// ---------------------------------------------------------------------------
// 파일의 노드 섹션마다 그 이름의 노드가 실제로 떠야 한다.
//
// 이름이 어긋난 섹션은 에러 없이 통째로 버려진다 (energy_model 이 그랬다).
// 아래 목록은 navigation.launch.py 가 이 파일을 넘기는 서버 5개와, 그 서버가
// 안에서 만드는 costmap 노드다. 2026-09-22 에 이 파일로 서버를 실제로 띄워
// 노드 목록을 떠서 확인했다. 런치에 노드를 추가하면 여기도 추가할 것.
// ---------------------------------------------------------------------------
TEST_P(Nav2ParamsFileTest, EverySectionNamesANodeTheLaunchCreates)
{
  const std::set<std::string> launched = {
    "/bt_navigator",
    "/controller_server",
    "/local_costmap/local_costmap",     // controller_server 안
    "/planner_server",
    "/global_costmap/global_costmap",   // planner_server 안
    "/behavior_server",
    "/velocity_smoother",
  };
  for (const auto & [fqn, params] : rclcpp::parameter_map_from_yaml_file(path_)) {
    // /** 같은 와일드카드 섹션은 여러 노드에 적용되므로 대상이 아니다
    if (fqn.find('*') != std::string::npos) {continue;}
    EXPECT_EQ(launched.count(fqn), 1u)
      << fqn << " 섹션에 해당하는 노드가 없다 — 값 " << params.size() << "개가 전부 버려진다.\n"
      << "  rcl 은 최상위 키(와 ros__parameters 앞까지의 중첩 키)를 노드 이름으로 읽는다.";
  }
}

INSTANTIATE_TEST_SUITE_P(
  HybridParamFiles, Nav2ParamsFileTest,
  ::testing::Values(kDefaultFile, kDerivedFile));


// ---------------------------------------------------------------------------
// energy:=derived 가 플래너가 쓰는 값을 실제로 바꿔야 한다.
//
// 버그가 있던 동안에는 두 파일 모두 C++ 기본값으로 떨어져 이 네 값이 같았다.
// ---------------------------------------------------------------------------
TEST(Nav2ParamsDerivedSwitch, DerivedChangesWhatThePlannerUses)
{
  if (!std::filesystem::is_directory(configDir())) {
    GTEST_SKIP() << "drobot_bringup 설정 디렉터리가 없다: " << configDir();
  }
  const EnergyModel def = modelFrom(configPath(kDefaultFile));
  const EnergyModel der = modelFrom(configPath(kDerivedFile));

  EXPECT_NE(def.takeoff(0.0).e_switch, der.takeoff(0.0).e_switch) << "takeoff_energy";
  EXPECT_NE(def.landing(0.0).e_switch, der.landing(0.0).e_switch) << "landing_energy";
  EXPECT_NE(
    def.takeoff(1.0).e_switch - def.takeoff(0.0).e_switch,
    der.takeoff(1.0).e_switch - der.takeoff(0.0).e_switch) << "energy_per_altitude_meter";
  EXPECT_NE(
    def.airMoveHorizontal(1.0, 100.0).e_air_horiz,
    der.airMoveHorizontal(1.0, 100.0).e_air_horiz) << "air_mode.energy_per_meter";

  // derived 는 '이착륙이 싼 쪽' 세트다. 방향까지 뒤집히면 파일이 바뀐 것이다.
  EXPECT_LT(der.eSwitchRef(), def.eSwitchRef());
}


int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
