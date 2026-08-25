// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "drobot_hybrid_planner/energy_model.hpp"

#include <algorithm>

#include <nav2_util/node_utils.hpp>

namespace drobot_hybrid_planner
{

namespace
{
/// 파라미터를 선언하고 값을 읽는 헬퍼.
/// Nav2 플러그인은 노드가 이미 살아 있는 상태로 초기화되므로,
/// declare_parameter_if_not_declared 로 중복 선언을 피해야 한다.
double getParam(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, double fallback)
{
  nav2_util::declare_parameter_if_not_declared(
    node, name, rclcpp::ParameterValue(fallback));
  return node->get_parameter(name).as_double();
}

bool getBoolParam(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & name, bool fallback)
{
  nav2_util::declare_parameter_if_not_declared(
    node, name, rclcpp::ParameterValue(fallback));
  return node->get_parameter(name).as_bool();
}
}  // namespace


void EnergyModel::configure(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
  const std::string & prefix)
{
  const std::string p = prefix + ".";

  // 비용 가중치 (정규화)
  w_energy_ = getParam(node, p + "cost_weights.w_energy", w_energy_);
  w_switch_ = getParam(node, p + "cost_weights.w_switch", w_switch_);
  w_time_ = getParam(node, p + "cost_weights.w_time", w_time_);

  // 지상
  ground_wh_per_m_ = getParam(node, p + "ground_mode.energy_per_meter", ground_wh_per_m_);
  ground_speed_ = getParam(node, p + "ground_mode.speed", ground_speed_);

  // 공중
  air_wh_per_m_ = getParam(node, p + "air_mode.energy_per_meter", air_wh_per_m_);
  hover_power_w_ = getParam(node, p + "air_mode.hover_power", hover_power_w_);
  air_speed_ = getParam(node, p + "air_mode.speed", air_speed_);

  // 모드 전환
  // 장애물 등반 (로버가 밟고 넘기)
  rover_climb_max_h_ = getParam(node, p + "climb_mode.max_height", rover_climb_max_h_);
  rover_climb_wh_per_m_ =
    getParam(node, p + "climb_mode.energy_per_height_m", rover_climb_wh_per_m_);
  rover_climb_speed_factor_ =
    getParam(node, p + "climb_mode.speed_factor", rover_climb_speed_factor_);

  takeoff_wh_ = getParam(node, p + "mode_switch.takeoff_energy", takeoff_wh_);
  takeoff_s_ = getParam(node, p + "mode_switch.takeoff_time", takeoff_s_);
  landing_wh_ = getParam(node, p + "mode_switch.landing_energy", landing_wh_);
  landing_s_ = getParam(node, p + "mode_switch.landing_time", landing_s_);
  wh_per_altitude_m_ =
    getParam(node, p + "mode_switch.energy_per_altitude_meter", wh_per_altitude_m_);

  // ground effect
  ge_enabled_ = getBoolParam(node, p + "ground_effect.enabled", ge_enabled_);
  ge_activation_height_ =
    getParam(node, p + "ground_effect.activation_height", ge_activation_height_);
  ge_power_multiplier_ =
    getParam(node, p + "ground_effect.power_multiplier", ge_power_multiplier_);

  // 수직 이동 / 형상 / 제약
  climb_wh_per_m_ = getParam(node, p + "vertical_motion.climb_energy_per_meter", climb_wh_per_m_);
  descent_ratio_ = getParam(node, p + "vertical_motion.descent_energy_ratio", descent_ratio_);
  climb_speed_ = getParam(node, p + "vertical_motion.climb_speed", climb_speed_);
  descent_speed_ = getParam(node, p + "vertical_motion.descent_speed", descent_speed_);

  altitude_cost_on_landing_ =
    getBoolParam(node, p + "apply_altitude_cost_to_landing", altitude_cost_on_landing_);
  robot_height_ = getParam(node, p + "robot.height", robot_height_);
  ceiling_margin_ = getParam(node, p + "robot.ceiling_margin", ceiling_margin_);
  min_flight_clearance_ = getParam(node, p + "min_flight_clearance", min_flight_clearance_);
  max_climb_angle_deg_ = getParam(node, p + "max_climb_angle_deg", max_climb_angle_deg_);

  // idle power 보정
  idle_enabled_ = getBoolParam(node, p + "idle_power.enabled", idle_enabled_);
  idle_ground_w_ = getParam(node, p + "idle_power.ground_w", idle_ground_w_);
  idle_air_w_ = getParam(node, p + "idle_power.air_w", idle_air_w_);

  RCLCPP_INFO(
    node->get_logger(),
    "EnergyModel: wE=%.3f wS=%.3f wT=%.3f | E_ref=%.3fWh T_ref=%.3fs"
    " | 등가 alpha=%.4f beta=%.4f gamma=%.4f"
    " | 로버 등반: 한계 %.2fm, %.2fWh/m, 속도배수 %.2f",
    w_energy_, w_switch_, w_time_, eRef(), tRef(),
    alpha(), beta(), gamma(),
    rover_climb_max_h_, rover_climb_wh_per_m_, rover_climb_speed_factor_);

  // 설정 충돌 경고 — 벤치마크에서 실제로 문제가 됐던 조합이다.
  // flight_clearance 는 플래너 쪽 파라미터라 여기서는 값을 모르지만,
  // ground effect가 죽는 조건을 로그로 남겨 둔다.
  if (ge_enabled_) {
    RCLCPP_INFO(
      node->get_logger(),
      "EnergyModel: ground effect는 clearance <= %.2fm 에서만 발동한다. "
      "3D 상태공간에서 flight_clearance가 이보다 크면 영원히 적용되지 않는다.",
      ge_activation_height_);
  }
}


CostAccumulator EnergyModel::groundMove(double dist) const
{
  CostAccumulator a;
  a.e_ground = ground_wh_per_m_ * dist;
  a.time_s = dist / ground_speed_;
  a.dist_ground = dist;

  // idle 보정: 측정 에너지에 포함된 대기 전력을 빼서
  // 시간 항(wT·T)과 이중 계상되지 않게 한다.
  if (idle_enabled_) {
    a.e_ground = std::max(0.0, a.e_ground - idle_ground_w_ * a.time_s / 3600.0);
  }
  return a;
}


CostAccumulator EnergyModel::roverClimbMove(double dist, double delta_h) const
{
  CostAccumulator a;
  a.e_ground = ground_wh_per_m_ * dist;
  a.dist_ground = dist;

  if (delta_h > 0.0) {
    // 올라가는 스텝에만 등반 비용이 붙는다. 평지 이동이나 하강은
    // 평지 주행과 같다 (groundMove 와 동일한 결과가 나와야 한다).
    a.e_ground += rover_climb_wh_per_m_ * delta_h;
    a.time_s = dist / (ground_speed_ * rover_climb_speed_factor_);
  } else {
    a.time_s = dist / ground_speed_;
  }

  if (idle_enabled_) {
    a.e_ground = std::max(0.0, a.e_ground - idle_ground_w_ * a.time_s / 3600.0);
  }
  return a;
}


CostAccumulator EnergyModel::airMoveHorizontal(double dist, double clearance) const
{
  double mult = 1.0;
  if (ge_enabled_ && clearance <= ge_activation_height_) {
    mult = ge_power_multiplier_;
  }
  CostAccumulator a;
  a.e_air_horiz = air_wh_per_m_ * dist * mult;
  a.time_s = dist / air_speed_;
  a.dist_air = dist;

  if (idle_enabled_) {
    a.e_air_horiz = std::max(0.0, a.e_air_horiz - idle_air_w_ * a.time_s / 3600.0);
  }
  return a;
}


CostAccumulator EnergyModel::airMoveVertical(double dz) const
{
  CostAccumulator a;
  if (dz >= 0.0) {
    a.e_air_vert = climb_wh_per_m_ * dz;
    a.time_s = dz / climb_speed_;
  } else {
    const double drop = -dz;
    a.e_air_vert = climb_wh_per_m_ * descent_ratio_ * drop;
    a.time_s = drop / descent_speed_;
  }
  return a;
}


CostAccumulator EnergyModel::takeoff(double altitude) const
{
  CostAccumulator a;
  a.e_switch = takeoff_wh_ + wh_per_altitude_m_ * altitude;
  a.time_s = takeoff_s_;
  a.n_takeoff = 1;
  return a;
}


CostAccumulator EnergyModel::landing(double altitude) const
{
  CostAccumulator a;
  const double extra = altitude_cost_on_landing_ ? wh_per_altitude_m_ * altitude : 0.0;
  a.e_switch = landing_wh_ + extra;
  a.time_s = landing_s_;
  a.n_landing = 1;
  return a;
}


double EnergyModel::cost(const CostAccumulator & acc) const
{
  // C = wE·(E_motion + E_switch)/E_ref + wS·n_switch + wT·T/T_ref
  //
  // 전환 에너지를 E_motion 과 같은 참조값으로 나누는 이유:
  //   이전에는 E_switch 를 eSwitchRef()(8.0 Wh)로 따로 나눴다. 그러면
  //   같은 1 Wh 라도 주행이면 wE/0.5 = 1.000, 전환이면 wS/8.0 = 0.025 로
  //   40배 차이가 났다. 배터리에서 빠지는 1 Wh 는 어디서 쓰든 1 Wh 인데도.
  //   그 결과 총 52.9 Wh 쓰는 경로가 14.1 Wh 쓰는 경로를 이기고
  //   '최적'으로 뽑히는 일이 생겼다.
  //
  // 전환 항이 '횟수'로 남은 이유:
  //   전환에는 에너지 외의 비용도 있다 — 착륙 실패 위험, 자세 재수립,
  //   제어 복잡도. Wh 로 환산되지 않으므로 횟수 페널티로 둔다.
  //   nSwitches() 는 이륙과 착륙을 각각 세므로 비행 한 구간이면 2·wS 다.
  return w_energy_ * (acc.eMotion() + acc.e_switch) / eRef() +
         w_switch_ * static_cast<double>(acc.nSwitches()) +
         w_time_ * acc.time_s / tRef();
}


double EnergyModel::maxFlightAltitude(double ceiling_height) const
{
  return ceiling_height - robot_height_ - ceiling_margin_;
}


double EnergyModel::minAltitudeOver(double terrain_height) const
{
  return terrain_height + min_flight_clearance_;
}


double EnergyModel::maxDzFor(double horizontal_dist) const
{
  return horizontal_dist * std::tan(max_climb_angle_deg_ * M_PI / 180.0);
}

}  // namespace drobot_hybrid_planner
