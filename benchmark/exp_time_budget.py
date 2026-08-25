"""동일 시간 예산 비교 — A* vs RRT*.

지금까지는 RRT*에 샘플 2만 개(약 15초)를 주고 비교했다.
하지만 실제 운용 조건은 다르다:

    hybrid_rrt_params.yaml:  timeout: 2.0   # Nav2 글로벌 플래너 응답 제약

같은 시간 예산에서 누가 더 좋은 해를 내는가 — 이게 실전에서 의미 있는 질문이다.

측정
  T1. 시간 예산별 해 품질 (0.1 / 0.25 / 0.5 / 1.0 / 2.0 / 5.0 초)
  T2. RRT*가 A*의 해 품질에 도달하는 데 걸리는 시간
  T3. 2초 예산에서의 성공률 (시드 20개)
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

SEP = "=" * 82
model = EnergyModel.from_yaml()
maps = build_all(res=0.1)
TEST = ["easy_open", "easy_corridor", "medium_open", "medium_corridor", "hard_open"]

BUDGETS = [0.1, 0.25, 0.5, 1.0, 2.0, 5.0]
N_SEEDS = 5


def run_astar(spec):
    t0 = time.perf_counter()
    r = astar(spec, timeout=60.0)
    if not r.found:
        return None, None
    c, acc, _ = smooth_result(spec, r, is_grid=True)
    return c, time.perf_counter() - t0


def run_rrt(spec, seed, budget):
    r = rrt_star(spec, seed=seed, max_samples=10**7, timeout=budget)
    if not r.found:
        return None
    c, _, _ = smooth_result(spec, r, is_grid=False)
    return c


# ---------------------------------------------------------------- T1
print(SEP)
print("T1. 동일 시간 예산에서의 해 품질")
print(SEP)
print(f"  RRT*는 시드 {N_SEEDS}개의 '최선값' (RRT*에게 유리한 조건)")
print("  A*는 결정론적이라 1회면 충분")
print()

results = {}
for name in TEST:
    spec = ProblemSpec(hm=maps[name], model=model, dims=3)
    ca, ta = run_astar(spec)
    if ca is None:
        print(f"  [{name}] A* 해 없음")
        continue
    results[name] = {"astar": ca, "astar_t": ta, "rrt": {}}

    print(f"  [{name}]  A* = {ca:.3f}  ({ta:.3f}s, 스무딩 포함)")
    print(f"     {'예산':>7} {'RRT*최선':>10} {'vs A*':>9} {'성공시드':>9}")
    print("     " + "-" * 40)
    for b in BUDGETS:
        costs = []
        for s in range(N_SEEDS):
            c = run_rrt(spec, s, b)
            if c is not None:
                costs.append(c)
        if not costs:
            print(f"     {b:>6.2f}s {'해없음':>10} {'':>9} {0:>4}/{N_SEEDS}")
            results[name]["rrt"][b] = None
            continue
        best = min(costs)
        rel = 100 * (best - ca) / ca
        results[name]["rrt"][b] = best
        mark = "  <-- A* 추월" if best < ca else ""
        print(f"     {b:>6.2f}s {best:>10.3f} {rel:>+8.2f}% "
              f"{len(costs):>4}/{N_SEEDS}{mark}")
    print()


# ---------------------------------------------------------------- T2
print(SEP)
print("T2. RRT*가 A* 수준에 도달하는 데 걸리는 시간")
print(SEP)
print(f"  {'맵':<17} {'A* 시간':>9} {'RRT* 도달시간':>14} {'배수':>9}")
print("  " + "-" * 54)
for name in TEST:
    if name not in results:
        continue
    r = results[name]
    ca, ta = r["astar"], r["astar_t"]
    reach = None
    for b in BUDGETS:
        c = r["rrt"].get(b)
        if c is not None and c <= ca:
            reach = b
            break
    if reach is None:
        print(f"  {name:<17} {ta:>8.3f}s {'>5.0s 미도달':>14} {'':>9}")
    else:
        print(f"  {name:<17} {ta:>8.3f}s {reach:>13.2f}s {reach/ta:>8.1f}x")


# ---------------------------------------------------------------- T3
print()
print(SEP)
print("T3. 실제 운용 예산(2.0초)에서의 신뢰성 — 시드 20개")
print(SEP)
print("  hybrid_rrt_params.yaml 의 timeout: 2.0 이 실제 조건이다.")
print()
print(f"  {'맵':<17} {'A*':>9} | {'RRT*평균':>9} {'표준편차':>9} {'최선':>9} "
      f"{'최악':>9} {'실패':>6}")
print("  " + "-" * 76)
N20 = 20
for name in TEST:
    if name not in results:
        continue
    spec = ProblemSpec(hm=maps[name], model=model, dims=3)
    ca = results[name]["astar"]
    cs, fails = [], 0
    for s in range(N20):
        c = run_rrt(spec, s, 2.0)
        if c is None:
            fails += 1
        else:
            cs.append(c)
    if cs:
        a = np.array(cs)
        print(f"  {name:<17} {ca:>9.3f} | {a.mean():>9.3f} {a.std():>9.3f} "
              f"{a.min():>9.3f} {a.max():>9.3f} {fails:>3}/{N20}")
    else:
        print(f"  {name:<17} {ca:>9.3f} | {'전부 실패':>9} {'':>9} {'':>9} "
              f"{'':>9} {fails:>3}/{N20}")

print()
print("  ※ A*는 표준편차 0, 실패 0 (결정론적).")
print("    RRT*의 표준편차와 최악값이 '재현성' 논거의 근거가 된다.")
