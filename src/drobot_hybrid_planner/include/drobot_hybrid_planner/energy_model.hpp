// Copyright 2026 leo11dk
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// 에너지 비용 모델
//
// benchmark/cost/energy.py 를 C++로 이식한 것이다.
// Python 벤치마크에서 검증된 것과 '같은' 비용 함수여야 하므로,
// 수식과 파라미터 이름을 그대로 유지했다.
//
// 비용 함수 (정규화 형태)
//     C = wE·E_motion/E_ref + wS·E_switch/E_switch_ref + wT·T/T_ref
//
// 세 항이 모두 무차원이다. 이전 형태(α·E + β·E_switch + γ·T)와 동등하며
//     α = wE/E_ref,  β = wS/E_switch_ref,  γ = wT/T_ref
// 로 변환된다.
//
// 단위
//     에너지: Wh,  시간: s,  거리: m
//     고도 z: m (지면 기준 절대 고도. 지형 높이가 h인 곳 위를 고도 z로 날면
//                여유(clearance)는 z - h)

#ifndef DROBOT_HYBRID_PLANNER__ENERGY_MODEL_HPP_
#define DROBOT_HYBRID_PLANNER__ENERGY_MODEL_HPP_

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace drobot_hybrid_planner
{

/// 경로를 따라가며 비용을 항목별로 쌓아두는 그릇.
///
/// 스칼라 하나로 쌓지 않고 항목을 나눠 두는 이유:
///   1) 가중치가 항목마다 달라서, 나중에 가중치를 바꿔 재계산하려면
///      원자료가 남아 있어야 한다.
///   2) drobot_experiments 의 trial_summary.csv 가
///      flight_energy_wh / ground_energy_wh 를 따로 요구한다.
struct CostAccumulator
{
  double e_ground = 0.0;      ///< 지상 주행 에너지 (Wh)
  double e_air_horiz = 0.0;   ///< 수평 비행 에너지 (Wh)
  double e_air_vert = 0.0;    ///< 수직 이동 에너지 (Wh)
  double e_switch = 0.0;      ///< 모드 전환 에너지 (Wh)
  double time_s = 0.0;        ///< 총 소요 시간 (s)

  double dist_ground = 0.0;   ///< 지상 주행 거리 (m)
  double dist_air = 0.0;      ///< 비행 거리 (m, 수평 성분)
  int n_takeoff = 0;
  int n_landing = 0;

  /// 이동 에너지 = 지상 + 비행(수평+수직). 전환 에너지는 제외.
  double eMotion() const {return e_ground + e_air_horiz + e_air_vert;}
  double eAir() const {return e_air_horiz + e_air_vert;}
  /// 실제로 배터리에서 빠지는 총 에너지 (가중치 없음)
  double eTotal() const {return eMotion() + e_switch;}
  int nSwitches() const {return n_takeoff + n_landing;}

  CostAccumulator operator+(const CostAccumulator & o) const
  {
    CostAccumulator r;
    r.e_ground = e_ground + o.e_ground;
    r.e_air_horiz = e_air_horiz + o.e_air_horiz;
    r.e_air_vert = e_air_vert + o.e_air_vert;
    r.e_switch = e_switch + o.e_switch;
    r.time_s = time_s + o.time_s;
    r.dist_ground = dist_ground + o.dist_ground;
    r.dist_air = dist_air + o.dist_air;
    r.n_takeoff = n_takeoff + o.n_takeoff;
    r.n_landing = n_landing + o.n_landing;
    return r;
  }

  CostAccumulator & operator+=(const CostAccumulator & o)
  {
    *this = *this + o;
    return *this;
  }
};


/// energy_params.yaml 을 담고 비용을 계산한다.
class EnergyModel
{
public:
  EnergyModel() = default;

  /// ROS 파라미터에서 값을 읽어 초기화한다.
  /// prefix 아래에 energy_params.yaml 의 항목들이 선언되어 있어야 한다.
  void configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
    const std::string & prefix);

  // ---- 정규화 참조값 -------------------------------------------------
  // 파라미터에서 유도한다. 실측으로 파라미터가 바뀌어도 가중치의 의미가
  // 유지된다 (w_energy=0.5는 늘 '평지 1m 주행 대비' 기준).
  double eRef() const {return ground_wh_per_m_;}
  double tRef() const {return 1.0 / ground_speed_;}

  /// 모드 전환 1회(이륙+착륙) 에너지. 비용 함수에는 쓰지 않고
  /// 분석·보고용으로만 남겨둔다 (cost() 주석 참고).
  double eSwitchRef() const {return takeoff_wh_ + landing_wh_;}

  // ---- 등가 α/β/γ (논문에서 두 형태를 연결할 때) ----------------------
  // C = α·(E_motion + E_switch) + β·n_switch + γ·T
  /// 에너지 1 Wh 당 비용 — 주행이든 전환이든 동일하게 적용된다.
  double alpha() const {return w_energy_ / eRef();}
  /// 모드 전환 1회당 페널티 (무차원). 에너지가 아니라 '횟수'에 붙는다.
  /// 이륙과 착륙을 각각 세므로 비행 한 구간이면 2·β 가 붙는다.
  double beta() const {return w_switch_;}
  double gamma() const {return w_time_ / tRef();}

  // ---- 원자 연산 -----------------------------------------------------
  // 아래 5개가 경로를 구성하는 최소 단위. 모든 경로 비용은 이들의 합이다.

  /// 지상 주행 dist 미터
  CostAccumulator groundMove(double dist) const;

  /// 장애물을 밟고 올라가며 dist 미터 이동. delta_h 는 고도 상승분 (m).
  ///
  /// 높이 '차이'에만 부과한다. 목적지 높이에 비례해 부과하면 장애물 위를
  /// 여러 셀 지나갈 때 같은 값이 중복 계상되기 때문이다. 상승분에만 붙이면
  /// 올라갈 때 한 번만 들고, 위를 지나는 동안은 평지와 같으며,
  /// 내려올 때는 공짜다 — 위치에너지와 같은 경로 무관 구조가 된다.
  ///
  /// 주의: 계수는 INA226 실측 전 추정값이다. 위치에너지 공식을 그대로 쓰면
  ///       (2.723kg, h=0.7m, 효율 0.4) 0.013 Wh 로 평지 2.6cm 에 불과해
  ///       사실상 공짜가 된다. 실제 등반은 모터 토크 급증·슬립·저속 때문에
  ///       훨씬 크다.
  CostAccumulator roverClimbMove(double dist, double delta_h) const;

  /// 수평 비행 dist 미터.
  /// clearance: 바로 아래 지형 상단으로부터의 여유 높이 (= z - h).
  ///            activation_height 이하면 ground effect로 전력이 증가한다.
  CostAccumulator airMoveHorizontal(double dist, double clearance) const;

  /// 비행 중 고도 변경. dz > 0 상승, < 0 하강.
  CostAccumulator airMoveVertical(double dz) const;

  /// 이륙. altitude = 도달할 고도 (이륙 지점 지면 기준)
  CostAccumulator takeoff(double altitude) const;

  /// 착륙. altitude = 착륙 시작 고도
  CostAccumulator landing(double altitude) const;

  // ---- 최종 비용 -----------------------------------------------------
  double cost(const CostAccumulator & acc) const;

  // ---- 제약 ----------------------------------------------------------
  /// 천장 제약으로부터 허용되는 최대 비행 고도.
  ///   z + robot_height + ceiling_margin <= ceiling_height
  double maxFlightAltitude(double ceiling_height) const;

  /// 지형 높이 위를 날 때의 최소 허용 고도
  double minAltitudeOver(double terrain_height) const;

  /// 수평으로 horizontal_dist 이동하는 동안 바꿀 수 있는 최대 고도차.
  /// 최대 상승각에서 역산한다 — 격자 해상도에 의존하지 않는 게 핵심이다.
  /// (격자 칸수로 제한하면 해상도가 촘촘할수록 경사가 얕아지는 버그가 생긴다)
  double maxDzFor(double horizontal_dist) const;

  // ---- 접근자 --------------------------------------------------------
  double groundSpeed() const {return ground_speed_;}
  double airSpeed() const {return air_speed_;}
  double minFlightClearance() const {return min_flight_clearance_;}
  double robotHeight() const {return robot_height_;}
  /// 로버가 밟고 넘을 수 있는 최대 장애물 높이 (m)
  double roverClimbMaxH() const {return rover_climb_max_h_;}

private:
  // 정규화 가중치
  double w_energy_ = 0.5;
  double w_switch_ = 0.2;
  double w_time_ = 0.3;

  // 지상
  double ground_wh_per_m_ = 0.5;
  double ground_speed_ = 0.3;

  // 공중
  double air_wh_per_m_ = 2.0;
  double hover_power_w_ = 50.0;
  double air_speed_ = 0.5;

  // 모드 전환
  double takeoff_wh_ = 5.0;
  double takeoff_s_ = 5.0;
  double landing_wh_ = 3.0;
  double landing_s_ = 4.0;
  double wh_per_altitude_m_ = 2.0;

  // ground effect
  bool ge_enabled_ = true;
  double ge_activation_height_ = 0.5;
  double ge_power_multiplier_ = 1.15;

  // 장애물 등반 (로버가 밟고 넘기) — 아래 climb_* 와 혼동 주의.
  // 이쪽은 '지상 로버가 장애물 위로 타고 넘는' 동작이고,
  // 아래 climb_* 는 '비행 중 수직 상승' 이다.
  double rover_climb_max_h_ = 0.7;
  double rover_climb_wh_per_m_ = 2.0;
  double rover_climb_speed_factor_ = 0.4;

  // 비행 중 수직 이동 (energy_params.yaml 에 없는 값 — 벤치마크 가정에서 가져옴)
  double climb_wh_per_m_ = 2.0;
  double descent_ratio_ = 0.3;
  double climb_speed_ = 0.5;
  double descent_speed_ = 0.5;

  // 형상 / 제약
  bool altitude_cost_on_landing_ = true;
  double robot_height_ = 0.25;
  double ceiling_margin_ = 0.5;
  double min_flight_clearance_ = 0.2;
  double max_climb_angle_deg_ = 45.0;

  // idle power 보정 (실측 후 사용)
  bool idle_enabled_ = false;
  double idle_ground_w_ = 0.0;
  double idle_air_w_ = 0.0;
};

}  // namespace drobot_hybrid_planner

#endif  // DROBOT_HYBRID_PLANNER__ENERGY_MODEL_HPP_
