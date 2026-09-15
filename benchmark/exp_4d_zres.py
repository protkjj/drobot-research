"""4D (x,y,z,m) + z해상도 sweep — A*가 무너지는 경계 찾기.

3D에서는 A*가 우위였다. 4D는 상태공간이 z축만큼 커지므로 A*에 불리하다.
어느 해상도에서 역전되는지 찾으면 "실용 해상도에서는 A*, 극단 해상도에서는 RRT*"
라는 조건부 결론을 데이터로 만들 수 있다.

무조건적 주장("A*가 항상 좋다")보다 조건부 주장이 방어력이 높다.

측정
  Z1. z해상도별 상태공간 크기와 A* 성능
  Z2. 같은 조건에서 RRT* 성능
  Z3. 역전 지점
  Z4. 4D가 3D보다 실제로 나은 해를 주는가 (4D를 하는 이유 재확인)
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

# hard_corridor는 3D에서 해가 없다 -> 4D의 가치를 보여주는 맵이므로 포함
TEST = ["easy_open", "medium_open", "medium_corridor", "hard_open", "hard_corridor"]
Z_RESOLUTIONS = [0.5, 0.25, 0.1, 0.05]
A_TIMEOUT = 60.0
RRT_BUDGET = 2.0
N_SEEDS = 5


def run_astar_4d(spec, timeout=A_TIMEOUT):
    t0 = time.perf_counter()
    r = astar(spec, timeout=timeout)
    if not r.found:
        return None, time.perf_counter() - t0, r
    c, _, _ = smooth_result(spec, r, is_grid=True)
    return c, time.perf_counter() - t0, r


def run_rrt_4d(spec, budget=RRT_BUDGET, n_seeds=N_SEEDS):
    costs = []
    for s in range(n_seeds):
        r = rrt_star(spec, seed=s, max_samples=10**7, timeout=budget)
        if r.found:
            c, _, _ = smooth_result(spec, r, is_grid=False)
            costs.append(c)
    return costs


# ---------------------------------------------------------------- Z1, Z2, Z3
print(SEP)
print("Z1-Z3. z해상도별 A* vs RRT* (4D 상태공간)")
print(SEP)
print(f"  A* timeout {A_TIMEOUT}s | RRT* 예산 {RRT_BUDGET}s x 시드 {N_SEEDS}개 중 최선")
print()

summary = {}
for name in TEST:
    hm = maps[name]
    print(f"  [{name}]")
    print(f"     {'z해상도':>8} {'상태수':>12} {'A*비용':>9} {'A*시간':>8} "
          f"{'RRT*최선':>10} {'승자':>8}")
    print("     " + "-" * 62)
    summary[name] = {}

    for zr in Z_RESOLUTIONS:
        spec = ProblemSpec(hm=hm, model=model, dims=4, z_res=zr)
        n_states = spec.summary()["n_states"]

        ca, ta, ra = run_astar_4d(spec)
        rrt_costs = run_rrt_4d(spec)
        cr = min(rrt_costs) if rrt_costs else None

        if ca is None:
            a_str = "timeout" if ra.timed_out else "해없음"
            a_cost_s = f"{a_str:>9}"
        else:
            a_cost_s = f"{ca:>9.3f}"

        cr_s = f"{cr:>10.3f}" if cr is not None else f"{'해없음':>10}"

        if ca is not None and cr is not None:
            winner = "A*" if ca <= cr else "RRT*"
        elif ca is not None:
            winner = "A*"
        elif cr is not None:
            winner = "RRT*"
        else:
            winner = "-"

        summary[name][zr] = {"astar": ca, "astar_t": ta, "rrt": cr,
                             "n_states": n_states, "winner": winner}
        print(f"     {zr:>7.2f}m {n_states:>12,} {a_cost_s} {ta:>7.2f}s "
              f"{cr_s} {winner:>8}")
    print()


# ---------------------------------------------------------------- 역전 지점
print(SEP)
print("Z3. 역전 지점 요약")
print(SEP)
print(f"  {'맵':<17} " + " ".join(f"{z:>8.2f}m" for z in Z_RESOLUTIONS))
print("  " + "-" * (18 + 10 * len(Z_RESOLUTIONS)))
for name in TEST:
    row = f"  {name:<17} "
    for zr in Z_RESOLUTIONS:
        row += f"{summary[name][zr]['winner']:>9}"
    print(row)

print()
print("  A* 실행시간 (초)")
print(f"  {'맵':<17} " + " ".join(f"{z:>8.2f}m" for z in Z_RESOLUTIONS))
print("  " + "-" * (18 + 10 * len(Z_RESOLUTIONS)))
for name in TEST:
    row = f"  {name:<17} "
    for zr in Z_RESOLUTIONS:
        t = summary[name][zr]["astar_t"]
        row += f"{t:>9.2f}"
    print(row)


# ---------------------------------------------------------------- Z4
print()
print(SEP)
print("Z4. 4D가 3D보다 나은 해를 주는가 (4D를 하는 이유 재확인)")
print(SEP)
print(f"  {'맵':<17} {'3D A*':>10} {'4D A*(0.25m)':>13} {'개선':>9} {'비고':>20}")
print("  " + "-" * 74)
for name in TEST:
    spec3 = ProblemSpec(hm=maps[name], model=model, dims=3)
    c3, _, r3 = run_astar_4d(spec3)
    d4 = summary[name].get(0.25, {})
    c4 = d4.get("astar")

    if c3 is None and c4 is not None:
        print(f"  {name:<17} {'해없음':>10} {c4:>13.3f} {'':>9} "
              f"{'3D 불가, 4D 가능':>20}")
    elif c3 is not None and c4 is not None:
        imp = 100 * (c3 - c4) / c3
        print(f"  {name:<17} {c3:>10.3f} {c4:>13.3f} {imp:>+8.2f}% {'':>20}")
    elif c3 is not None:
        print(f"  {name:<17} {c3:>10.3f} {'해없음':>13} {'':>9} {'4D timeout':>20}")
    else:
        print(f"  {name:<17} {'해없음':>10} {'해없음':>13}")

print()
print("  ※ hard_corridor는 3D에서 통로(높이 1.10m)를 못 넘어 해가 없다.")
print("    4D는 고도를 자유롭게 잡아 통과 가능 — 이게 4D의 존재 이유.")
