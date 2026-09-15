"""Python↔C++ 동등성 테스트용 기댓값 생성.

C++ 플래너는 benchmark/cost/energy.py 를 이식한 것이다.
이식 과정에서 수식이 미묘하게 달라지면, Python 벤치마크에서 얻은 결론
(A* 채택 근거)을 C++ 구현에 그대로 인용할 수 없게 된다.

그래서 Python 쪽에서 테스트 케이스와 기댓값을 뽑아 C++ 헤더로 내보내고,
C++ gtest 가 같은 입력에 같은 값이 나오는지 대조한다.

실행
    python3 benchmark/gen_cpp_fixtures.py
    -> src/drobot_hybrid_planner/test/energy_fixtures.hpp 생성

주의: energy_params.yaml 을 바꾸면 이 스크립트를 다시 돌려야 한다.
      (기댓값이 파라미터에 의존하므로)
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cost.energy import EnergyModel, CostAccumulator  # noqa: E402

OUT = (Path(__file__).resolve().parents[1]
       / "src/drobot_hybrid_planner/test/energy_fixtures.hpp")

m = EnergyModel.from_yaml()

# ---------------------------------------------------------------------------
# 테스트 케이스 — 원자 연산별로 대표값을 뽑는다
# ---------------------------------------------------------------------------
ground_cases = [0.05, 0.1, 0.1414213562, 1.0, 5.0]
air_cases = [
    # (거리, clearance)  — clearance가 activation_height 위/아래를 모두 포함
    (0.1, 0.8), (0.1, 0.3), (1.0, 0.5), (1.0, 0.51), (3.0, 0.2),
]
vert_cases = [0.1, -0.1, 0.5, -0.5, 1.0]
alt_cases = [0.2, 0.35, 0.8, 1.2, 1.75]

lines = []
w = lines.append

# 라이선스 헤더 — ament_copyright 린터가 요구한다.
# 생성 파일도 검사 대상이므로 여기서 함께 넣는다.
# ament_copyright 의 mit_header_0 템플릿과 정확히 일치해야 인식된다.
w("// Copyright 2026 leo11dk")
w("//")
w("// Use of this source code is governed by an MIT-style")
w("// license that can be found in the LICENSE file or at")
w("// https://opensource.org/licenses/MIT.")
w("")
w("// 자동 생성 파일 — 직접 수정하지 말 것.")
w("// 생성: python3 benchmark/gen_cpp_fixtures.py")
w("//")
w("// Python 벤치마크(benchmark/cost/energy.py)의 기댓값이다.")
w("// C++ EnergyModel 이 같은 입력에 같은 값을 내는지 대조하는 데 쓴다.")
w("// energy_params.yaml 을 바꾸면 이 파일을 다시 생성해야 한다.")
w("")
w("#ifndef ENERGY_FIXTURES_HPP_")
w("#define ENERGY_FIXTURES_HPP_")
w("")
w("namespace drobot_test")
w("{")
w("")
w("// 참조값 (파라미터에서 유도)")
w(f"constexpr double kERef = {m.e_ref!r};")
w(f"constexpr double kESwitchRef = {m.e_switch_ref!r};")
w(f"constexpr double kTRef = {m.t_ref!r};")
w("")
w("// 로버 등반 파라미터")
w(f"constexpr double kRoverClimbMaxH = {m.rover_climb_max_h!r};")
w(f"constexpr double kRoverClimbWhPerM = {m.rover_climb_wh_per_m!r};")
w(f"constexpr double kRoverClimbSpeedFactor = {m.rover_climb_speed_factor!r};")
w("")
w("// 등가 alpha/beta/gamma  (C = alpha*(E_motion+E_switch) + beta*n_switch + gamma*T)")
w(f"constexpr double kAlpha = {m.alpha!r};")
w(f"constexpr double kBeta = {m.beta!r};")
w(f"constexpr double kGamma = {m.gamma!r};")
w("")

# 지상 이동
w("struct GroundCase")
w("{")
w("  double dist;")
w("  double e_wh;")
w("  double time_s;")
w("  double cost;")
w("};")
w("constexpr GroundCase kGroundCases[] = {")
for d in ground_cases:
    a = m.ground_move(d)
    w(f"  {{{d!r}, {a.e_ground!r}, {a.time_s!r}, {m.cost(a)!r}}},")
w("};")
w(f"constexpr int kNumGroundCases = {len(ground_cases)};")
w("")

# 장애물 등반 — (거리, 높이차) 쌍.
# 상승(dh>0) / 평지(dh=0) / 하강(dh<0) 세 경우를 모두 넣어
# '상승분에만 부과' 규칙이 C++ 에서도 같은지 확인한다.
climb_cases = [
    (0.1, 0.55), (0.1, 0.35), (0.1, 0.70),      # 올라가기
    (0.1, 0.0), (0.5, 0.0),                      # 장애물 위 평지 주행
    (0.1, -0.55), (0.1, -0.35),                  # 내려오기
    (0.1414213562373095, 0.60),                  # 대각 진입
    (1.0, 0.15),
]
w("struct RoverClimbCase")
w("{")
w("  double dist;")
w("  double delta_h;")
w("  double e_wh;")
w("  double time_s;")
w("  double cost;")
w("};")
w("constexpr RoverClimbCase kRoverClimbCases[] = {")
for d, dh in climb_cases:
    a = m.climb_move(d, dh)
    w(f"  {{{d!r}, {dh!r}, {a.e_ground!r}, {a.time_s!r}, {m.cost(a)!r}}},")
w("};")
w(f"constexpr int kNumRoverClimbCases = {len(climb_cases)};")
w("")

# 수평 비행
w("struct AirCase")
w("{")
w("  double dist;")
w("  double clearance;")
w("  double e_wh;")
w("  double time_s;")
w("  double cost;")
w("};")
w("constexpr AirCase kAirCases[] = {")
for d, c in air_cases:
    a = m.air_move_horizontal(d, clearance=c)
    w(f"  {{{d!r}, {c!r}, {a.e_air_horiz!r}, {a.time_s!r}, {m.cost(a)!r}}},")
w("};")
w(f"constexpr int kNumAirCases = {len(air_cases)};")
w("")

# 수직 이동
w("struct VertCase")
w("{")
w("  double dz;")
w("  double e_wh;")
w("  double time_s;")
w("  double cost;")
w("};")
w("constexpr VertCase kVertCases[] = {")
for dz in vert_cases:
    a = m.air_move_vertical(dz)
    w(f"  {{{dz!r}, {a.e_air_vert!r}, {a.time_s!r}, {m.cost(a)!r}}},")
w("};")
w(f"constexpr int kNumVertCases = {len(vert_cases)};")
w("")

# 이착륙
w("struct SwitchCase")
w("{")
w("  double altitude;")
w("  double takeoff_wh;")
w("  double takeoff_cost;")
w("  double landing_wh;")
w("  double landing_cost;")
w("};")
w("constexpr SwitchCase kSwitchCases[] = {")
for z in alt_cases:
    t = m.takeoff(z)
    l = m.landing(z)
    w(f"  {{{z!r}, {t.e_switch!r}, {m.cost(t)!r}, "
      f"{l.e_switch!r}, {m.cost(l)!r}}},")
w("};")
w(f"constexpr int kNumSwitchCases = {len(alt_cases)};")
w("")

# 제약
w("struct ConstraintCase")
w("{")
w("  double input;")
w("  double expected;")
w("};")
w("// maxFlightAltitude(ceiling)")
w("constexpr ConstraintCase kMaxAltCases[] = {")
for ceil in (2.2, 2.5, 3.0):
    w(f"  {{{ceil!r}, {m.max_flight_altitude(ceil)!r}}},")
w("};")
w("constexpr int kNumMaxAltCases = 3;")
w("")
w("// maxDzFor(horizontal_dist)")
w("constexpr ConstraintCase kMaxDzCases[] = {")
for d in (0.1, 0.1414213562, 1.0):
    w(f"  {{{d!r}, {m.max_dz_for(d)!r}}},")
w("};")
w("constexpr int kNumMaxDzCases = 3;")
w("")

# 복합 시나리오 — 원자 연산을 조합했을 때도 일치하는지
w("// 복합 시나리오: 이륙 -> 수평비행 -> 착륙")
w("struct ScenarioCase")
w("{")
w("  double altitude;")
w("  double dist;")
w("  double clearance;")
w("  double e_total_wh;")
w("  double time_s;")
w("  double cost;")
w("};")
w("constexpr ScenarioCase kScenarioCases[] = {")
for z, d, c in [(0.35, 2.0, 0.35), (0.8, 5.0, 0.6), (1.2, 1.0, 0.25)]:
    acc = CostAccumulator()
    acc = acc + m.takeoff(z)
    acc = acc + m.air_move_horizontal(d, clearance=c)
    acc = acc + m.landing(z)
    w(f"  {{{z!r}, {d!r}, {c!r}, {acc.e_total!r}, {acc.time_s!r}, {m.cost(acc)!r}}},")
w("};")
w("constexpr int kNumScenarioCases = 3;")
w("")
w("}  // namespace drobot_test")
w("")
w("#endif  // ENERGY_FIXTURES_HPP_")

OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text("\n".join(lines) + "\n")
print(f"생성: {OUT}")
print(f"  케이스 수: ground={len(ground_cases)} air={len(air_cases)} "
      f"vert={len(vert_cases)} switch={len(alt_cases)} scenario=3")
print()
print("참조값 확인:")
print(f"  E_ref={m.e_ref}  E_switch_ref={m.e_switch_ref}  T_ref={m.t_ref:.6f}")
print(f"  alpha={m.alpha:.6f}  beta={m.beta:.6f}  gamma={m.gamma:.6f}")
print(f"  로버 등반: 한계 {m.rover_climb_max_h}m, "
      f"{m.rover_climb_wh_per_m} Wh/m, 속도배수 {m.rover_climb_speed_factor}")
