"""
에너지 비용 모델.

drobot_hybrid_planner/config/energy_params.yaml 을 그대로 읽어서
"경로 한 조각의 비용"을 계산하는 함수들을 제공한다.

이 모듈이 하는 일은 딱 하나 — 비용 함수
    C = alpha * E_motion + beta * E_switch + gamma * T
를 계산 가능한 형태로 만드는 것.
A*든 RRT*든 이 모듈만 쓴다. 두 플래너가 서로 다른 비용 함수를 쓰면
비교 자체가 무의미해지므로, 비용 계산은 반드시 한 곳에만 존재해야 한다.

단위
  에너지: Wh
  시간:   s
  거리:   m
  고도:   m (지면 기준 절대 고도. AGL = Above Ground Level 이 아님에 주의 —
             지형 높이가 h인 곳 위를 고도 z로 날면 여유(clearance)는 z - h)
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

# 이 파일 기준 경로. benchmark/cost/energy.py -> 레포 루트는 두 단계 위
_REPO_ROOT = Path(__file__).resolve().parents[2]
_ENERGY_YAML = _REPO_ROOT / "src/drobot_hybrid_planner/config/energy_params.yaml"
_ASSUM_YAML = _REPO_ROOT / "benchmark/config/benchmark_assumptions.yaml"


# ---------------------------------------------------------------------------
# 비용 누적기
# ---------------------------------------------------------------------------
@dataclass
class CostAccumulator:
    """경로를 따라가며 비용을 항목별로 쌓아두는 그릇.

    왜 그냥 스칼라 하나로 안 쌓고 항목을 나눠 두느냐면:
      1) C = a*E_motion + b*E_switch + g*T 에서 가중치가 항목마다 다르다.
         나중에 alpha/beta/gamma를 바꿔가며 재계산하려면 원자료가 남아 있어야 한다.
      2) logging_config.yaml의 trial_summary.csv가
         flight_energy_wh / ground_energy_wh 를 따로 요구한다.
    """

    e_ground: float = 0.0      # 지상 주행 에너지 (Wh)
    e_air_horiz: float = 0.0   # 수평 비행 에너지 (Wh)
    e_air_vert: float = 0.0    # 수직 이동 에너지 (Wh, 비행 중 상승/하강)
    e_switch: float = 0.0      # 모드 전환 에너지 (Wh, 이륙+착륙)
    time_s: float = 0.0        # 총 소요 시간 (s)

    dist_ground: float = 0.0   # 지상 주행 거리 (m)
    dist_air: float = 0.0      # 비행 거리 (m, 수평 성분)
    n_takeoff: int = 0
    n_landing: int = 0

    @property
    def e_motion(self) -> float:
        """이동 에너지 = 지상 + 비행(수평+수직). 전환 에너지는 제외."""
        return self.e_ground + self.e_air_horiz + self.e_air_vert

    @property
    def e_air(self) -> float:
        return self.e_air_horiz + self.e_air_vert

    @property
    def e_total(self) -> float:
        """실제로 배터리에서 빠지는 총 에너지 (가중치 없음)."""
        return self.e_motion + self.e_switch

    @property
    def n_switches(self) -> int:
        return self.n_takeoff + self.n_landing

    def __add__(self, other: "CostAccumulator") -> "CostAccumulator":
        return CostAccumulator(
            e_ground=self.e_ground + other.e_ground,
            e_air_horiz=self.e_air_horiz + other.e_air_horiz,
            e_air_vert=self.e_air_vert + other.e_air_vert,
            e_switch=self.e_switch + other.e_switch,
            time_s=self.time_s + other.time_s,
            dist_ground=self.dist_ground + other.dist_ground,
            dist_air=self.dist_air + other.dist_air,
            n_takeoff=self.n_takeoff + other.n_takeoff,
            n_landing=self.n_landing + other.n_landing,
        )


# ---------------------------------------------------------------------------
# 에너지 모델
# ---------------------------------------------------------------------------
@dataclass
class EnergyModel:
    """energy_params.yaml + benchmark_assumptions.yaml 을 담고 비용을 계산."""

    # --- 비용 함수 가중치 (정규화 형태) ---
    # C = wE*E_motion/E_ref + wS*E_switch/E_switch_ref + wT*T/T_ref
    # 세 항이 모두 무차원이라 단위가 섞이지 않는다.
    w_energy: float
    w_switch: float
    w_time: float

    # --- 지상 ---
    ground_wh_per_m: float
    ground_speed: float

    # --- 공중 ---
    air_wh_per_m: float
    hover_power_w: float
    air_speed: float

    # --- 모드 전환 ---
    takeoff_wh: float
    takeoff_s: float
    landing_wh: float
    landing_s: float
    wh_per_altitude_m: float

    # --- 장애물 등반 (밟고 넘기) ---
    rover_climb_max_h: float          # 밟고 넘을 수 있는 최대 장애물 높이 (m)
    rover_climb_wh_per_m: float       # 등반 시 높이 1m 당 추가 에너지 (Wh/m)
    rover_climb_speed_factor: float   # 등반 중 속도 배수 (평지 대비)

    # --- ground effect ---
    ge_enabled: bool
    ge_activation_height: float
    ge_power_multiplier: float

    # --- 벤치마크 추가 가정 ---
    climb_wh_per_m: float
    descent_ratio: float
    climb_speed: float
    descent_speed: float
    altitude_cost_on_landing: bool
    robot_height: float
    ceiling_margin: float
    min_flight_clearance: float
    max_climb_angle_deg: float

    # 원본 dict 보관 (디버깅/기록용)
    raw: dict = field(default_factory=dict, repr=False)

    # -- 생성 ------------------------------------------------------------
    @classmethod
    def from_yaml(
        cls,
        energy_yaml: Path | str = _ENERGY_YAML,
        assumptions_yaml: Path | str = _ASSUM_YAML,
    ) -> "EnergyModel":
        with open(energy_yaml) as f:
            raw_e = yaml.safe_load(f)
        with open(assumptions_yaml) as f:
            raw_a = yaml.safe_load(f)

        # ROS 파라미터 yaml은 <노드이름>/ros__parameters/ 아래에 값이 들어간다
        p = raw_e["energy_model"]["ros__parameters"]
        a = raw_a["assumptions"]

        w = p["cost_weights"]
        g = p["ground_mode"]

        # 정규화 가중치를 읽되, 구형식(alpha/beta/gamma)도 받아들인다.
        # 벤치마크 이전 결과와 대조하려면 구형식을 그대로 돌릴 수 있어야 한다.
        if "w_energy" in w:
            w_e, w_s, w_t = (float(w["w_energy"]), float(w["w_switch"]),
                             float(w["w_time"]))
        else:
            # 구형식 -> 정규화 환산:  wE = alpha*E_ref 등
            _e_ref = float(g["energy_per_meter"])
            _es_ref = float(ms["takeoff_energy"]) + float(ms["landing_energy"])
            _t_ref = 1.0 / float(g["speed"])
            w_e = float(w["alpha"]) * _e_ref
            w_s = float(w["beta"]) * _es_ref
            w_t = float(w["gamma"]) * _t_ref
        air = p["air_mode"]
        ms = p["mode_switch"]
        ge = p["ground_effect"]
        cm = p.get("climb_mode", {})
        vm = a["vertical_motion"]

        return cls(
            w_energy=w_e,
            w_switch=w_s,
            w_time=w_t,
            ground_wh_per_m=float(g["energy_per_meter"]),
            ground_speed=float(g["speed"]),
            air_wh_per_m=float(air["energy_per_meter"]),
            hover_power_w=float(air["hover_power"]),
            air_speed=float(air["speed"]),
            takeoff_wh=float(ms["takeoff_energy"]),
            takeoff_s=float(ms["takeoff_time"]),
            landing_wh=float(ms["landing_energy"]),
            landing_s=float(ms["landing_time"]),
            wh_per_altitude_m=float(ms["energy_per_altitude_meter"]),
            rover_climb_max_h=float(cm.get("max_height", 0.7)),
            rover_climb_wh_per_m=float(cm.get("energy_per_height_m", 2.0)),
            rover_climb_speed_factor=float(cm.get("speed_factor", 0.4)),
            ge_enabled=bool(ge["enabled"]),
            ge_activation_height=float(ge["activation_height"]),
            ge_power_multiplier=float(ge["power_multiplier"]),
            climb_wh_per_m=float(vm["climb_energy_per_meter"]),
            descent_ratio=float(vm["descent_energy_ratio"]),
            climb_speed=float(vm["climb_speed"]),
            descent_speed=float(vm["descent_speed"]),
            altitude_cost_on_landing=bool(a["apply_altitude_cost_to_landing"]),
            robot_height=float(a["robot"]["height"]),
            ceiling_margin=float(a["robot"]["ceiling_margin"]),
            min_flight_clearance=float(a["min_flight_clearance"]),
            max_climb_angle_deg=float(a["max_climb_angle_deg"]),
            raw={"energy_params": raw_e, "assumptions": raw_a},
        )

    # -- 원자 연산 --------------------------------------------------------
    # 아래 5개가 경로를 구성하는 최소 단위. 모든 경로 비용은 이들의 합.

    def ground_move(self, dist: float) -> CostAccumulator:
        """지상 주행 dist 미터."""
        return CostAccumulator(
            e_ground=self.ground_wh_per_m * dist,
            time_s=dist / self.ground_speed,
            dist_ground=dist,
        )

    def climb_move(self, dist: float, delta_h: float) -> CostAccumulator:
        """장애물을 밟고 올라가며 dist 미터 이동. delta_h 는 고도 상승분(m).

        높이 '차이'에만 부과하는 이유:
            목적지 높이에 비례해 부과하면, 장애물 위를 여러 셀 지나갈 때
            매 셀마다 같은 값이 중복 계상된다.
            상승분에만 부과하면 올라갈 때 한 번만 들고, 위를 지나는 동안은
            평지 주행과 같으며, 내려올 때는 공짜다 —
            위치에너지와 같은 경로 무관(path-independent) 구조가 된다.

        위치에너지 공식(m·g·h)을 그대로 쓰지 않는 이유:
            로봇 2.72kg, h=0.7m, 효율 0.4 로 계산하면 0.013 Wh 인데
            이는 평지 주행 2.6cm 에 해당해 사실상 공짜가 된다.
            실제 등반은 모터 토크 급증·슬립·저속 때문에 훨씬 크다.
            정확한 값은 INA226 실측이 필요하며, 그 전까지는
            energy_per_height_m 을 sweep 해서 민감도를 본다.
        """
        a = CostAccumulator()
        a.e_ground = self.ground_wh_per_m * dist
        a.dist_ground = dist
        if delta_h > 0.0:
            a.e_ground += self.rover_climb_wh_per_m * delta_h
            a.time_s = dist / (self.ground_speed * self.rover_climb_speed_factor)
        else:
            a.time_s = dist / self.ground_speed
        return a

    def air_move_horizontal(self, dist: float, clearance: float) -> CostAccumulator:
        """수평 비행 dist 미터.

        clearance: 바로 아래 지형 상단으로부터의 여유 높이 (= z - h).
                   이 값이 activation_height 이하면 ground effect로 전력이 증가한다.
        """
        mult = 1.0
        if self.ge_enabled and clearance <= self.ge_activation_height:
            mult = self.ge_power_multiplier
        return CostAccumulator(
            e_air_horiz=self.air_wh_per_m * dist * mult,
            time_s=dist / self.air_speed,
            dist_air=dist,
        )

    def air_move_vertical(self, dz: float) -> CostAccumulator:
        """비행 중 고도 변경. dz > 0 이면 상승, < 0 이면 하강.

        주의: 이 항목은 energy_params.yaml에 정의가 없는 '가정'이다.
              benchmark_assumptions.yaml 참고.
        """
        if dz >= 0.0:
            return CostAccumulator(
                e_air_vert=self.climb_wh_per_m * dz,
                time_s=dz / self.climb_speed,
            )
        drop = -dz
        return CostAccumulator(
            e_air_vert=self.climb_wh_per_m * self.descent_ratio * drop,
            time_s=drop / self.descent_speed,
        )

    def takeoff(self, altitude: float) -> CostAccumulator:
        """이륙. altitude = 도달할 고도 (이륙 지점 지면 기준)."""
        return CostAccumulator(
            e_switch=self.takeoff_wh + self.wh_per_altitude_m * altitude,
            time_s=self.takeoff_s,
            n_takeoff=1,
        )

    def landing(self, altitude: float) -> CostAccumulator:
        """착륙. altitude = 착륙 시작 고도 (착지 지점 지면 기준)."""
        extra = self.wh_per_altitude_m * altitude if self.altitude_cost_on_landing else 0.0
        return CostAccumulator(
            e_switch=self.landing_wh + extra,
            time_s=self.landing_s,
            n_landing=1,
        )

    # -- 정규화 참조값 -----------------------------------------------------
    # 파라미터에서 유도한다. 실측으로 파라미터가 바뀌어도 가중치의 의미가
    # 유지된다 (w_energy=0.5는 늘 '평지 1m 주행 대비' 기준).
    @property
    def e_ref(self) -> float:
        """평지 1m 주행 에너지 (Wh)."""
        return self.ground_wh_per_m

    @property
    def e_switch_ref(self) -> float:
        """모드 전환 1회(이륙+착륙) 에너지 (Wh).

        비용 함수에는 더 이상 쓰지 않는다 (아래 cost() 주석 참고).
        분석·보고용으로만 남겨둔다.
        """
        return self.takeoff_wh + self.landing_wh

    @property
    def t_ref(self) -> float:
        """평지 1m 주행 시간 (s)."""
        return 1.0 / self.ground_speed

    # -- 등가 alpha/beta/gamma (논문에서 두 형태를 연결할 때 쓴다) ----------
    # C = alpha*(E_motion + E_switch) + beta*n_switch + gamma*T
    @property
    def alpha(self) -> float:
        """에너지 1 Wh 당 비용 — 주행이든 전환이든 동일하게 적용된다."""
        return self.w_energy / self.e_ref

    @property
    def beta(self) -> float:
        """모드 전환 1회당 페널티 (무차원).

        에너지가 아니라 '횟수'에 붙는 계수다. 정규화 형태에서 전환 항이
        wS*n_switch 이므로 beta = wS 가 된다.
        이륙과 착륙을 각각 1회로 세므로 비행 한 구간이면 2*beta 가 붙는다.
        """
        return self.w_switch

    @property
    def gamma(self) -> float:
        return self.w_time / self.t_ref

    # -- 최종 비용 --------------------------------------------------------
    def cost(self, acc: CostAccumulator) -> float:
        """C = wE*(E_motion + E_switch)/E_ref + wS*n_switch + wT*T/T_ref

        세 항이 모두 무차원이다.
        등가 형태: C = alpha*(E_motion + E_switch) + beta*n_switch + gamma*T

        왜 전환 에너지를 E_motion 과 같은 참조값으로 나누는가:
            이전에는 E_switch 를 E_switch_ref(8.0 Wh)로, E_motion 을
            E_ref(0.5 Wh)로 나눴다. 그러면 같은 1 Wh 라도
                주행으로 쓰면  wE * 1/0.5 = 1.000
                전환으로 쓰면  wS * 1/8.0 = 0.025
            로 40배 차이가 났다. 배터리 입장에서 1 Wh 는 어디서 쓰든
            1 Wh 인데도 그랬다.

            그 결과 medium_open 에서 총 에너지 52.9 Wh 인 하이브리드 경로가
            14.1 Wh 인 밟고넘기 경로를 이기고 '최적'으로 뽑혔다.
            에너지 효율 경로를 찾는다는 목적과 정면으로 어긋난다.

        왜 전환 항이 '횟수'로 남았는가:
            전환에는 에너지 외의 비용도 있다 — 착륙 실패 위험, 자세 재수립,
            제어 복잡도. 이건 Wh 로 환산되지 않으므로 횟수 페널티로 둔다.
            이렇게 하면 wS 의 의미도 명확해진다:
            'wS=0.2' 는 전환 1회를 평지 주행 0.2m 와 같게 본다는 뜻이다.

        주의: n_switches = n_takeoff + n_landing 이라 이륙과 착륙을 따로 센다.
              즉 비행 한 구간(이륙 -> 비행 -> 착륙)의 페널티는 2*wS 다.
        """
        return (
            self.w_energy * (acc.e_motion + acc.e_switch) / self.e_ref
            + self.w_switch * acc.n_switches
            + self.w_time * acc.time_s / self.t_ref
        )

    # -- 제약 -------------------------------------------------------------
    def max_flight_altitude(self, ceiling_height: float) -> float:
        """천장 제약으로부터 허용되는 최대 비행 고도.

        elevation_params.yaml 문언:
          장애물높이 + 로봇 비행고도 + 천장여유(0.5) > ceiling_height -> Impassable
        를 고도 상한으로 옮기면:
          z + robot_height + ceiling_margin <= ceiling_height
        """
        return ceiling_height - self.robot_height - self.ceiling_margin

    def min_altitude_over(self, terrain_height: float) -> float:
        """지형 높이 terrain_height 위를 날 때의 최소 허용 고도."""
        return terrain_height + self.min_flight_clearance

    def max_dz_for(self, horizontal_dist: float) -> float:
        """수평으로 horizontal_dist 이동하는 동안 바꿀 수 있는 최대 고도차.

        최대 상승각으로부터 역산한다. 격자 해상도에 의존하지 않는 게 핵심 —
        예전 구현은 '격자 한 칸'으로 제한해서 z해상도가 촘촘할수록
        경사가 얕아지는 버그가 있었다.
        """
        import math as _m
        return horizontal_dist * _m.tan(_m.radians(self.max_climb_angle_deg))
