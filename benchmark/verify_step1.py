"""1단계 검증 — 에너지 모델이 config와 일치하는지, 그리고
앞서 손계산으로 주장한 두 가지가 실제로 성립하는지 확인한다.

검증 항목
  V1. yaml 로딩이 원본 값과 일치하는가
  V2. 3D 형식(고도 = h + 0.8 고정)에서 ground effect가 정말 안 켜지는가
  V3. 장애물 높이가 일정하면 최적 고도가 정말 '경계값'인가
  V4. 장애물 높이가 변하면 최적 고도 프로파일이 비자명해지는가

V3/V4가 핵심이다. V3가 참이면 "높이가 일정한 맵으로는 4D 실험이 무의미"하고,
V4가 참이면 "높이를 변화시킨 맵에서는 진짜 최적화 문제가 된다"는 뜻.
2단계 맵 설계가 여기 결과에 달려 있다.
"""
import sys
from pathlib import Path

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cost.energy import EnergyModel, CostAccumulator  # noqa: E402

m = EnergyModel.from_yaml()
SEP = "=" * 74
CEILING = 2.5  # elevation_params.yaml: ceiling_height


# ---------------------------------------------------------------- V1
print(SEP)
print("V1. config 로딩 검증 — 코드가 yaml 값을 그대로 읽었는가")
print(SEP)
p = m.raw["energy_params"]["energy_model"]["ros__parameters"]
checks = [
    ("w_energy", m.w_energy, p["cost_weights"]["w_energy"]),
    ("w_switch", m.w_switch, p["cost_weights"]["w_switch"]),
    ("w_time", m.w_time, p["cost_weights"]["w_time"]),
    ("등반 한계높이", m.rover_climb_max_h, p["climb_mode"]["max_height"]),
    ("등반 Wh/m", m.rover_climb_wh_per_m, p["climb_mode"]["energy_per_height_m"]),
    ("등반 속도배수", m.rover_climb_speed_factor, p["climb_mode"]["speed_factor"]),
    ("ground Wh/m", m.ground_wh_per_m, p["ground_mode"]["energy_per_meter"]),
    ("ground speed", m.ground_speed, p["ground_mode"]["speed"]),
    ("air Wh/m", m.air_wh_per_m, p["air_mode"]["energy_per_meter"]),
    ("air speed", m.air_speed, p["air_mode"]["speed"]),
    ("takeoff Wh", m.takeoff_wh, p["mode_switch"]["takeoff_energy"]),
    ("landing Wh", m.landing_wh, p["mode_switch"]["landing_energy"]),
    ("Wh/alt-m", m.wh_per_altitude_m, p["mode_switch"]["energy_per_altitude_meter"]),
    ("GE act.height", m.ge_activation_height, p["ground_effect"]["activation_height"]),
    ("GE multiplier", m.ge_power_multiplier, p["ground_effect"]["power_multiplier"]),
]
all_ok = True
for name, got, want in checks:
    good = abs(got - want) < 1e-12
    all_ok &= good
    print(f"  {'PASS' if good else 'FAIL'}  {name:15s} = {got!r:<8} (yaml: {want!r})")
print(f"  => {'전부 일치' if all_ok else '!!! 불일치 있음 !!!'}")


# ---------------------------------------------------------------- V2
print()
print(SEP)
print("V2. 3D 형식에서 ground effect가 발동하는가")
print(SEP)
_planner_yaml = (
    Path(__file__).resolve().parents[1]
    / "src/drobot_hybrid_planner/config/hybrid_astar_params.yaml"
)
_pl = yaml.safe_load(open(_planner_yaml))["planner_server"]["ros__parameters"]["GridBased"]
FLIGHT_CLEARANCE = float(_pl["flight_clearance"])

print(f"  hybrid_astar_params.yaml flight_clearance  = {FLIGHT_CLEARANCE}")
print(f"  energy_params.yaml       activation_height = {m.ge_activation_height}")
print(f"  ground_effect.enabled                      = {m.ge_enabled}")
print()
print(f"  3D에서는 비행고도 = 장애물높이 + {FLIGHT_CLEARANCE}")
print(f"  따라서 clearance(= z - h)는 항상 {FLIGHT_CLEARANCE} 로 고정된다.")

d_test = 3.0
e_3d = m.air_move_horizontal(d_test, clearance=FLIGHT_CLEARANCE).e_air_horiz
e_nominal = m.air_wh_per_m * d_test
mult = e_3d / e_nominal
print()
print(f"  검산: {d_test}m 비행 -> {e_3d:.4f} Wh, 배수 없는 값 {e_nominal:.4f} Wh")
print(f"        실제 적용된 배수 = {mult:.4f}")
print(f"  => ground effect 발동: {mult > 1.0 + 1e-9}"
      f"  ({'죽은 파라미터 확인' if abs(mult - 1) < 1e-9 else '발동함'})")

# 반대 방향 확인: clearance를 낮추면 정말 켜지는가 (모델이 망가진 게 아님을 확인)
e_low = m.air_move_horizontal(d_test, clearance=0.3).e_air_horiz
print(f"  반대 검증: clearance=0.3 이면 배수 = {e_low / e_nominal:.4f}"
      f"  (기대값 {m.ge_power_multiplier}) -> "
      f"{'모델 정상' if abs(e_low/e_nominal - m.ge_power_multiplier) < 1e-9 else '모델 이상'}")


# ---------------------------------------------------------------- V3
print()
print(SEP)
print("V3. 장애물 높이가 일정할 때 — 최적 고도가 경계값인가")
print(SEP)
print("  주장: E(z)는 z에 단조증가(+4 Wh/m)이고 z=h+0.5에서 불연속 감소하므로,")
print("        최적해는 항상 z_min 또는 h+0.5 둘 중 하나다 (내부 최적점 없음).")
print()


def segment_cost_uniform(z, h, d, model):
    """높이 h 장애물 위를 거리 d, 고도 z로 비행하는 1회 비행 구간 총 비용."""
    acc = CostAccumulator()
    acc = acc + model.takeoff(z)
    acc = acc + model.air_move_horizontal(d, clearance=z - h)
    acc = acc + model.landing(z)
    return model.cost(acc)


v3_all_boundary = True
for h, d in [(0.5, 2.0), (0.5, 8.0), (1.2, 3.0), (0.8, 15.0)]:
    z_lo = m.min_altitude_over(h)
    z_hi = m.max_flight_altitude(CEILING)
    if z_hi <= z_lo:
        print(f"  h={h} d={d}: 천장 제약으로 비행 불가 (z_hi={z_hi:.2f} <= z_lo={z_lo:.2f})")
        continue

    zs = np.linspace(z_lo, z_hi, 40001)
    cs = np.array([segment_cost_uniform(z, h, d, m) for z in zs])
    i = int(np.argmin(cs))
    z_opt, c_opt = zs[i], cs[i]

    # 이론이 예측하는 후보들
    cands = {"z_min": z_lo}
    z_ge = h + m.ge_activation_height
    if z_lo < z_ge <= z_hi:
        cands["h+0.5"] = min(z_ge + 1e-9, z_hi)

    nearest = min(cands, key=lambda k: abs(cands[k] - z_opt))
    is_boundary = abs(cands[nearest] - z_opt) < 5e-3
    v3_all_boundary &= is_boundary

    cand_str = ", ".join(f"{k}={v:.3f}" for k, v in cands.items())
    print(f"  h={h:4.1f} d={d:5.1f} | z∈[{z_lo:.2f},{z_hi:.2f}] "
          f"| 최적 z={z_opt:.4f}  C={c_opt:.3f}")
    print(f"       후보 {{{cand_str}}} → 최근접 '{nearest}' "
          f"| 경계값인가: {'YES' if is_boundary else 'NO'}")

print()
print(f"  => 모든 케이스에서 경계값: {v3_all_boundary}")
if v3_all_boundary:
    print("     ⇒ 높이 일정 맵에서는 탐색 없이 손으로 풀린다. 4D 비교가 무의미해짐.")


# ---------------------------------------------------------------- V4
print()
print(SEP)
print("V4. 장애물 높이가 변할 때 — 고도 프로파일이 비자명해지는가")
print(SEP)
print("  시나리오: 높이 h1 구간(길이 d1) → 높이 h2 구간(길이 d2) 를 한 번의 비행으로 통과")
print("    A안(고도 고정): 두 구간을 같은 고도로 통과")
print("    B안(고도 가변): 구간마다 고도 변경 (수직이동 비용 발생)")
print()


def two_segment_cost(z1, z2, h1, d1, h2, d2, model):
    acc = CostAccumulator()
    acc = acc + model.takeoff(z1)
    acc = acc + model.air_move_horizontal(d1, clearance=z1 - h1)
    acc = acc + model.air_move_vertical(z2 - z1)
    acc = acc + model.air_move_horizontal(d2, clearance=z2 - h2)
    acc = acc + model.landing(z2)
    return model.cost(acc)


scenarios = [
    (1.2, 2.0, 0.2, 10.0, "높은 장애물 짧게 → 낮은 장애물 길게"),
    (1.2, 10.0, 0.2, 2.0, "높은 장애물 길게 → 낮은 장애물 짧게"),
    (0.2, 6.0, 1.2, 6.0, "낮은 → 높은 (대칭)"),
    (1.5, 3.0, 0.1, 12.0, "매우 높은 장애물 → 거의 평지 (극단)"),
]
z_hi = m.max_flight_altitude(CEILING)
v4_any_gain = False

for h1, d1, h2, d2, desc in scenarios:
    z1_lo = m.min_altitude_over(h1)
    z2_lo = m.min_altitude_over(h2)
    if z_hi <= max(z1_lo, z2_lo):
        print(f"  [{desc}] 천장 제약으로 불가")
        continue

    # B안: (z1, z2) 격자 탐색
    g1 = np.linspace(z1_lo, z_hi, 500)
    g2 = np.linspace(z2_lo, z_hi, 500)
    best, arg = np.inf, None
    for z1 in g1:
        for z2 in g2:
            c = two_segment_cost(z1, z2, h1, d1, h2, d2, m)
            if c < best:
                best, arg = c, (z1, z2)

    # A안: z1 == z2 제약 하 최선
    z_flat = np.linspace(max(z1_lo, z2_lo), z_hi, 8000)
    cf = np.array([two_segment_cost(z, z, h1, d1, h2, d2, m) for z in z_flat])
    j = int(np.argmin(cf))

    gain = cf[j] - best
    rel = 100 * gain / cf[j]
    v4_any_gain |= gain > 1e-6

    print(f"  [{desc}]")
    print(f"     h1={h1} d1={d1} / h2={h2} d2={d2}")
    print(f"     A안(고정): z={z_flat[j]:.3f}                  C={cf[j]:.4f}")
    print(f"     B안(가변): z1={arg[0]:.3f} z2={arg[1]:.3f}   C={best:.4f}")
    print(f"     => 가변이 {gain:+.4f} 유리 (상대 {rel:+.2f}%)"
          f"  {'← 비자명' if gain > 1e-6 else '← 차이 없음(자명)'}")
    print()

print(f"  => 고도 가변이 유리한 케이스 존재: {v4_any_gain}")


# ---------------------------------------------------------------- 요약
print()
print(SEP)
print("요약 — 2단계 맵 설계에 주는 함의")
print(SEP)
print(f"  V1 config 일치           : {'OK' if all_ok else 'FAIL'}")
print(f"  V2 3D에서 GE 죽음        : {abs(mult - 1) < 1e-9}")
print(f"  V3 높이일정→경계값       : {v3_all_boundary}")
print(f"  V4 높이변화→비자명       : {v4_any_gain}")
print()
if v3_all_boundary and v4_any_gain:
    print("  결론: 맵의 각 비행구간에 '서로 다른 높이의 장애물'이 2개 이상 있어야")
    print("        4D 탐색이 의미를 갖는다. 2단계 맵을 그렇게 설계한다.")
else:
    print("  결론: 예상과 다름. 위 수치를 다시 검토할 것.")
