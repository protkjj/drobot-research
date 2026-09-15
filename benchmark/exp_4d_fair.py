"""4D 공정 비교 — 양쪽 모두 동일한 시간 제약(2초).

앞선 exp_4d_zres.py 는 A*에 60초, RRT*에 2초를 줬다.
그 조건에서 A*가 이긴 건 당연하고 공정하지 않다.

hybrid_rrt_params.yaml 의 timeout: 2.0 이 실제 운용 제약이므로,
양쪽 모두 2초로 묶고 다시 잰다. 이게 실전에서 의미 있는 비교다.

추가로 확인할 것
  F1. 2초 제약에서 4D A*가 해를 내는가
  F2. 3D A*(2초 내 완료) vs 4D RRT*(2초) 비교
      -> "4D가 필요하면 RRT*, 3D면 A*" 인지 확인
  F3. medium_corridor 에서 4D가 3D보다 나빴던 이유 (z 이산화 오차 의심)
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cost.energy import EnergyModel  # noqa: E402
from envs.heightmap import build_all  # noqa: E402
from planners.state_space import ProblemSpec  # noqa: E402
from planners.grid_search import astar  # noqa: E402
from planners.rrt_star import rrt_star  # noqa: E402
from planners.smoothing import smooth_result  # noqa: E402

SEP = "=" * 84
model = EnergyModel.from_yaml()
maps = build_all(res=0.1)

BUDGET = 2.0     # hybrid_rrt_params.yaml: timeout
Z_LIST = [0.5, 0.25, 0.1]
TEST = ["easy_open", "medium_open", "medium_corridor", "hard_open", "hard_corridor"]
N_SEEDS = 5


def run_a(spec, budget):
    t0 = time.perf_counter()
    r = astar(spec, timeout=budget)
    if not r.found:
        return None, time.perf_counter() - t0, r
    c, _, _ = smooth_result(spec, r, is_grid=True)
    return c, time.perf_counter() - t0, r


def run_r(spec, budget, n=N_SEEDS):
    out = []
    for s in range(n):
        r = rrt_star(spec, seed=s, max_samples=10 ** 7, timeout=budget)
        if r.found:
            c, _, _ = smooth_result(spec, r, is_grid=False)
            out.append(c)
    return out


# ---------------------------------------------------------------- F1
print(SEP)
print(f"F1. 4D — 양쪽 모두 {BUDGET}초 제약")
print(SEP)
print()
print(f"  {'맵':<17} {'z해상도':>8} {'A*':>10} {'A*상태':>9} | "
      f"{'RRT*최선':>10} {'성공':>6} | {'승자':>7}")
print("  " + "-" * 76)

f1 = {}
for name in TEST:
    for zr in Z_LIST:
        spec = ProblemSpec(hm=maps[name], model=model, dims=4, z_res=zr)
        ca, ta, ra = run_a(spec, BUDGET)
        rs = run_r(spec, BUDGET)
        cr = min(rs) if rs else None

        a_state = "timeout" if (ra.timed_out and not ra.found) else (
            "OK" if ra.found else "해없음")
        a_s = f"{ca:>10.3f}" if ca is not None else f"{'-':>10}"
        r_s = f"{cr:>10.3f}" if cr is not None else f"{'-':>10}"

        if ca is not None and cr is not None:
            w = "A*" if ca <= cr else "RRT*"
        elif ca is not None:
            w = "A*"
        elif cr is not None:
            w = "RRT*"
        else:
            w = "둘다실패"

        f1[(name, zr)] = {"a": ca, "r": cr, "winner": w}
        print(f"  {name:<17} {zr:>7.2f}m {a_s} {a_state:>9} | "
              f"{r_s} {len(rs):>3}/{N_SEEDS} | {w:>7}")
    print()


# ---------------------------------------------------------------- F2
print(SEP)
print(f"F2. 실전 선택지 비교 — 전부 {BUDGET}초 제약")
print(SEP)
print("  실제로 kj가 고를 수 있는 선택지는 이 셋이다:")
print("    (a) 3D + A*      : 현재 hybrid_rrt_params.yaml 설계 + A*")
print("    (b) 3D + RRT*    : 현재 설계 그대로")
print("    (c) 4D + RRT*    : 고도 최적화 + RRT* (4D A*는 시간 초과)")
print()
print(f"  {'맵':<17} {'(a) 3D+A*':>11} {'(b) 3D+RRT*':>12} "
      f"{'(c) 4D+RRT*':>12} {'최선':>10}")
print("  " + "-" * 68)

for name in TEST:
    spec3 = ProblemSpec(hm=maps[name], model=model, dims=3)
    ca3, _, _ = run_a(spec3, BUDGET)
    rs3 = run_r(spec3, BUDGET)
    cr3 = min(rs3) if rs3 else None

    spec4 = ProblemSpec(hm=maps[name], model=model, dims=4, z_res=0.25)
    rs4 = run_r(spec4, BUDGET)
    cr4 = min(rs4) if rs4 else None

    opts = {"(a) 3D+A*": ca3, "(b) 3D+RRT*": cr3, "(c) 4D+RRT*": cr4}
    valid = {k: v for k, v in opts.items() if v is not None}
    best = min(valid, key=valid.get) if valid else "없음"

    def fmt(v):
        return f"{v:>11.3f}" if v is not None else f"{'해없음':>11}"

    print(f"  {name:<17} {fmt(ca3)} {fmt(cr3):>12} {fmt(cr4):>12} {best:>10}")


# ---------------------------------------------------------------- F3
print()
print(SEP)
print("F3. medium_corridor 에서 4D가 3D보다 나빴던 이유")
print(SEP)
print("  4D는 3D의 상위집합이어야 한다 (3D가 쓰는 고도 h+0.8을 4D도 선택 가능).")
print("  그런데 4D(0.25m)가 3D보다 0.79% 나빴다. z격자 이산화 오차를 의심한다.")
print()

name = "medium_corridor"
spec3 = ProblemSpec(hm=maps[name], model=model, dims=3)
c3, t3, _ = run_a(spec3, 120.0)
print(f"  3D A* (제한 없음): {c3:.4f}  ({t3:.2f}s)")
print()
print(f"  {'z해상도':>8} {'4D A*':>10} {'3D 대비':>10} {'z격자가 h+0.8을 표현 가능?':>28}")
print("  " + "-" * 60)

for zr in [0.5, 0.25, 0.2, 0.1, 0.05]:
    spec4 = ProblemSpec(hm=maps[name], model=model, dims=4, z_res=zr)
    c4, t4, r4 = run_a(spec4, 180.0)
    if c4 is None:
        print(f"  {zr:>7.2f}m {'timeout':>10}")
        continue
    rel = 100 * (c3 - c4) / c3
    # 이 맵의 장애물 높이들에 대해 h + 0.8 이 z격자에 정확히 떨어지는가
    hs = sorted(set(np.round(maps[name].grid[
        (maps[name].grid > 0.15) & (maps[name].grid <= 1.55)], 2).tolist()))
    exact = all(abs((h + 0.8) / zr - round((h + 0.8) / zr)) < 1e-9 for h in hs)
    print(f"  {zr:>7.2f}m {c4:>10.4f} {rel:>+9.2f}% {str(exact):>28}")

print()
print("  => z격자가 3D의 고도(h+0.8)를 정확히 표현하지 못하면,")
print("     4D는 그보다 높거나 낮은 격자점만 쓸 수 있어 손해를 볼 수 있다.")
print("     해상도를 높이면 이 손해가 사라지는지로 확인된다.")
