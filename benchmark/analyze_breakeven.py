"""비행 손익분기 분석 — 언제 비행이 주행보다 싼가.

3-4단계 검증에서 최적해가 한 번도 비행하지 않는다는 게 드러났다.
맵을 고치기 전에, 어떤 조건에서 비행이 이기는지 정확히 계산한다.
감으로 맵을 만들면 또 같은 실패를 반복하게 된다.

모델
    비행: 장애물(폭 w, 높이 h)을 넘어간다
        C_fly = alpha*(이륙E + 수평E + 착륙E) + beta*(전환E) + gamma*(총시간)
    주행: 장애물을 우회한다 (우회 추가거리 L)
        C_ground = alpha*(0.5*L) + gamma*(L/0.3)

    두 값이 같아지는 L을 구하면 그게 손익분기 우회거리다.
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from cost.energy import CostAccumulator, EnergyModel  # noqa: E402

m = EnergyModel.from_yaml()
SEP = "=" * 78

CEILING = 2.5
z_max = m.max_flight_altitude(CEILING)


def cost_fly(w: float, h: float, z: float | None = None) -> tuple[float, float]:
    """폭 w, 높이 h 장애물을 비행으로 넘는 최소 비용과 그때의 고도."""
    z_lo = m.min_altitude_over(h)
    if z_lo > z_max:
        return float("inf"), float("nan")
    zs = [z] if z is not None else np.linspace(z_lo, z_max, 2001)
    best, z_best = float("inf"), float("nan")
    for zz in zs:
        acc = CostAccumulator()
        acc = acc + m.takeoff(zz)
        acc = acc + m.air_move_horizontal(w, clearance=zz - h)
        acc = acc + m.landing(zz)
        c = m.cost(acc)
        if c < best:
            best, z_best = c, zz
    return best, z_best


def cost_ground(dist: float) -> float:
    """지상 주행 dist 미터의 비용."""
    return m.cost(m.ground_move(dist))


def unit_ground() -> float:
    """지상 주행 1m당 비용."""
    return cost_ground(1.0)


print(SEP)
print("비행 손익분기 분석")
print(SEP)
print()
print(f"  지상 주행 단가 : {unit_ground():.4f} / m")
print(f"    = alpha*{m.ground_wh_per_m} + gamma/{m.ground_speed}"
      f" = {m.alpha}*{m.ground_wh_per_m} + {m.gamma}/{m.ground_speed}")
print()
print(f"  비행 고정비 (이착륙, z=0.35 기준):")
acc = CostAccumulator() + m.takeoff(0.35) + m.landing(0.35)
print(f"    에너지 {acc.e_switch:.2f} Wh + 시간 {acc.time_s:.1f}s"
      f" -> 비용 {m.cost(acc):.3f}")
print(f"    이것만으로 지상 주행 {m.cost(acc)/unit_ground():.2f} m 에 해당")


# ---------------------------------------------------------------- 표 1
print()
print(SEP)
print("표1. 장애물 폭/높이별 — 비행이 이기려면 우회거리가 얼마여야 하는가")
print(SEP)
print("  (우회거리 = 장애물을 피해 돌아가는 '추가' 거리)")
print()
hdr = f"  {'폭 w':>6} |" + "".join(f" h={h:<5.2f}" for h in (0.30, 0.60, 0.90, 1.20, 1.50))
print(hdr)
print("  " + "-" * (len(hdr) - 2))
for w in (0.5, 1.0, 2.0, 3.0, 5.0, 8.0):
    row = f"  {w:6.1f} |"
    for h in (0.30, 0.60, 0.90, 1.20, 1.50):
        cf, _ = cost_fly(w, h)
        if not np.isfinite(cf):
            row += "     -   "
            continue
        L = cf / unit_ground()
        row += f" {L:7.2f} "
    print(row)
print()
print("  읽는 법: 폭 1.0m 높이 0.60m 장애물이면, 우회거리가 이 값(m)보다")
print("           길 때만 비행이 이긴다.")


# ---------------------------------------------------------------- 표 2
print()
print(SEP)
print("표2. 좁고 긴 벽 — 비행에 가장 유리한 구조")
print(SEP)
print("  폭이 좁을수록 비행 비용이 작고, 벽이 길수록 우회거리가 크다.")
print("  벽 길이 Lw, 폭 w 인 벽을 만나면:")
print("    - 비행: 폭 w 만 건너면 됨")
print("    - 우회: 벽 끝까지 갔다 와야 하므로 대략 Lw 만큼 추가")
print()
print(f"  {'폭 w':>6} {'높이 h':>7} {'비행비용':>9} {'등가 주행거리':>13} "
      f"{'필요 벽길이':>12}")
print("  " + "-" * 56)
for w, h in [(0.4, 0.30), (0.4, 0.60), (0.6, 0.60), (0.6, 0.90),
             (1.0, 0.60), (1.0, 1.20), (2.0, 0.60)]:
    cf, zb = cost_fly(w, h)
    if not np.isfinite(cf):
        continue
    L = cf / unit_ground()
    print(f"  {w:6.1f} {h:7.2f} {cf:9.3f} {L:12.2f}m {L:11.1f}m+")


# ---------------------------------------------------------------- 표 3
print()
print(SEP)
print("표3. 완전 차단 시나리오 — 우회로가 아예 없으면?")
print(SEP)
print("  벽이 통로를 완전히 막으면 지상 주행은 '실패'다.")
print("  이 경우 비행 비용과 무관하게 비행이 유일한 해가 된다.")
print("  -> hard_corridor 가 이 케이스. 다만 이러면 '선택'이 아니라 '강제'라")
print("     에너지 비교 실험으로서의 가치는 떨어진다.")
print()
print("  가장 좋은 맵은 '비행과 주행이 비슷한 비용이라 진짜 고민되는' 맵이다.")
print("  즉 우회거리를 손익분기 근처로 맞춰야 한다.")


# ---------------------------------------------------------------- 권고
print()
print(SEP)
print("맵 재설계 권고")
print(SEP)

# 대표 케이스로 목표 우회거리 계산
targets = []
for w, h in [(0.6, 0.60), (0.6, 1.10), (1.0, 0.80)]:
    cf, zb = cost_fly(w, h)
    L = cf / unit_ground()
    targets.append((w, h, L))
    print(f"  폭 {w}m 높이 {h}m 장애물 -> 손익분기 우회거리 {L:.1f}m "
          f"(최적고도 {zb:.2f}m)")

print()
print("  설계 지침:")
print("   1) 장애물은 '좁고 긴 벽' 형태로 (폭 0.4~1.0m, 길이 6~14m)")
print("      폭이 좁아야 비행 비용이 작고, 길어야 우회 비용이 크다.")
print("   2) 우회거리를 손익분기의 0.7배 ~ 1.5배 범위로 배치")
print("      -> 어떤 벽은 넘는 게 이득, 어떤 벽은 도는 게 이득 (진짜 선택 발생)")
print("   3) 맵 크기를 키워야 한다. 현재 12~16m로는 8~14m 우회거리를 못 만든다.")
print("      권고: 20x14m ~ 24x18m")
print("   4) hard_corridor 처럼 완전 차단하는 맵도 1개는 유지 (실패 케이스 확인용)")
