"""1단계 추가 검증 — 3D 형식의 '통과 가능 높이 한계'

V4에서 4D의 에너지 이득이 0.5~0.7%로 미미하다는 게 확인됐다.
그렇다면 4D를 하는 이유가 약해진다. 다른 근거를 찾아본다.

가설: 3D는 비행고도가 h+0.8로 고정이므로, 천장 제약 때문에
      넘을 수 있는 장애물 높이의 상한이 4D보다 훨씬 낮다.
      이건 '효율' 차이가 아니라 '가능/불가능' 차이라 훨씬 강한 근거다.

검증 항목
  V5. 3D / 4D 각각의 통과 가능 장애물 높이 상한
  V6. 그 구간(3D는 못 넘고 4D는 넘는 높이)에서 실제로 무슨 일이 일어나는가
  V7. 천장 높이를 바꿔가며 한계가 어떻게 이동하는가 (민감도)
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cost.energy import EnergyModel, CostAccumulator  # noqa: E402

m = EnergyModel.from_yaml()
SEP = "=" * 74
CEILING = 2.5           # elevation_params.yaml: ceiling_height
FLIGHT_CLEARANCE = 0.8  # hybrid_rrt_params.yaml: flight_clearance


print(SEP)
print("V5. 3D vs 4D — 통과 가능한 장애물 높이 상한")
print(SEP)

z_max = m.max_flight_altitude(CEILING)
print(f"  천장 제약: z + robot_height({m.robot_height}) + margin({m.ceiling_margin})"
      f" <= ceiling({CEILING})")
print(f"           => 최대 비행고도 z_max = {z_max:.2f} m")
print()

# 3D: 고도가 h + flight_clearance 로 강제됨
h_max_3d = z_max - FLIGHT_CLEARANCE
print(f"  [3D] 고도 = h + {FLIGHT_CLEARANCE} (고정)")
print(f"       h + {FLIGHT_CLEARANCE} <= {z_max:.2f}  =>  h <= {h_max_3d:.2f} m")

# 4D: 고도가 자유, 하한은 min_flight_clearance
h_max_4d = z_max - m.min_flight_clearance
print(f"  [4D] 고도 자유, 하한 = h + {m.min_flight_clearance}")
print(f"       h + {m.min_flight_clearance} <= {z_max:.2f}  =>  h <= {h_max_4d:.2f} m")
print()
print(f"  => 3D가 못 넘고 4D는 넘는 높이 구간: ({h_max_3d:.2f}, {h_max_4d:.2f}] m")
print(f"     구간 폭 = {h_max_4d - h_max_3d:.2f} m")
print(f"     4D의 통과가능 높이는 3D 대비 {100*(h_max_4d/h_max_3d - 1):.0f}% 넓음")


print()
print(SEP)
print("V6. 문제의 높이 구간에서 실제로 무슨 일이 일어나는가")
print(SEP)


def can_fly_3d(h):
    """3D 형식에서 높이 h 장애물 위를 날 수 있는가."""
    return h + FLIGHT_CLEARANCE <= z_max


def can_fly_4d(h):
    """4D 형식에서 높이 h 장애물 위를 날 수 있는가."""
    return h + m.min_flight_clearance <= z_max


def best_cost_4d(h, d):
    """4D에서 높이 h 장애물 위를 거리 d 비행할 때의 최소 비용."""
    z_lo = m.min_altitude_over(h)
    if z_lo > z_max:
        return None, None
    zs = np.linspace(z_lo, z_max, 4001)
    best, z_best = np.inf, None
    for z in zs:
        acc = CostAccumulator()
        acc = acc + m.takeoff(z)
        acc = acc + m.air_move_horizontal(d, clearance=z - h)
        acc = acc + m.landing(z)
        c = m.cost(acc)
        if c < best:
            best, z_best = c, z
    return best, z_best


print(f"  {'높이 h':>8} | {'3D 통과':>8} | {'4D 통과':>8} | {'4D 최적고도':>11} | {'4D 비용':>9}")
print(f"  {'-'*8}-+-{'-'*8}-+-{'-'*8}-+-{'-'*11}-+-{'-'*9}")
for h in [0.5, 0.8, 0.90, 0.95, 1.0, 1.2, 1.4, 1.55, 1.6]:
    ok3 = can_fly_3d(h)
    ok4 = can_fly_4d(h)
    c4, z4 = best_cost_4d(h, d=5.0)
    c4s = f"{c4:.2f}" if c4 is not None else "-"
    z4s = f"{z4:.3f}" if z4 is not None else "-"
    mark = "  <-- 3D만 실패" if (ok4 and not ok3) else ""
    print(f"  {h:8.2f} | {str(ok3):>8} | {str(ok4):>8} | {z4s:>11} | {c4s:>9}{mark}")

print()
print("  ※ 3D가 실패한다는 것은 '그 장애물을 비행으로 넘는 선택지가 사라진다'는 뜻.")
print("     플래너는 우회 주행하거나, 우회로가 없으면 경로 자체를 못 찾는다.")


print()
print(SEP)
print("V7. 천장 높이 민감도 — 이 결론이 ceiling=2.5 에만 의존하는가")
print(SEP)
print(f"  {'천장':>6} | {'z_max':>6} | {'3D 한계':>8} | {'4D 한계':>8} | {'격차':>6}")
print(f"  {'-'*6}-+-{'-'*6}-+-{'-'*8}-+-{'-'*8}-+-{'-'*6}")
for ceil in [2.2, 2.4, 2.5, 2.7, 3.0, 3.5]:
    zm = m.max_flight_altitude(ceil)
    h3 = zm - FLIGHT_CLEARANCE
    h4 = zm - m.min_flight_clearance
    print(f"  {ceil:6.1f} | {zm:6.2f} | {h3:8.2f} | {h4:8.2f} | {h4-h3:6.2f}")

print()
print(f"  => 격차는 항상 {FLIGHT_CLEARANCE} - {m.min_flight_clearance} = "
      f"{FLIGHT_CLEARANCE - m.min_flight_clearance:.1f} m 로 일정.")
print("     천장이 낮을수록 이 격차가 '전체 통과가능 범위'에서 차지하는 비중이 커진다.")
for ceil in [2.2, 2.5, 3.0]:
    zm = m.max_flight_altitude(ceil)
    h3, h4 = zm - FLIGHT_CLEARANCE, zm - m.min_flight_clearance
    print(f"       천장 {ceil}m: 4D가 3D 대비 통과범위 {100*(h4/h3-1):5.1f}% 넓음")


print()
print(SEP)
print("결론")
print(SEP)
print(f"  4D를 채택하는 근거는 '에너지 0.5% 절감'이 아니라 다음 둘이다:")
print(f"    (1) 통과 가능 장애물 높이가 {h_max_3d:.2f}m -> {h_max_4d:.2f}m 로 확장 "
      f"({100*(h_max_4d/h_max_3d-1):.0f}% 증가)")
print(f"        = 실내 천장({CEILING}m) 아래에서 3D는 {h_max_3d:.2f}m 넘는 장애물을 "
      f"비행으로 넘지 못한다.")
print(f"    (2) 상태공간이 z축 방향으로 확장되어 A*의 격자 탐색이 급격히 비싸진다")
print(f"        (이게 원래 목적인 A* vs RRT* 비교의 무대)")
print()
print("  => 2단계 맵은 '3D로는 못 넘고 4D로는 넘는' 높이")
print(f"     ({h_max_3d:.2f} ~ {h_max_4d:.2f}m) 장애물을 반드시 포함해야 한다.")
