"""모드 전환이 필요한 통로 — A* 우위 논거 4의 정확한 형태.

앞선 narrow passage 실험에서 RRT*는 통로 폭 0.4m 까지도 20/20 성공했다.
"좁으면 샘플링이 실패한다"는 가설은 그 조건에서 성립하지 않았다.

그런데 hard_corridor 에서는 RRT*가 20회 전부 실패했다.
그 맵의 통로는 폭 2m로 넓다. 대신 통로 안에 높이 1.10m 장애물이 있어
'이륙 -> 통과 -> 착륙' 시퀀스를 거쳐야만 지날 수 있다.

가설 수정:
    샘플링 기반의 약점은 '통로가 좁아서'가 아니라
    '모드 전환 시퀀스를 우연히 맞춰야 해서'다.

    A*는 이륙/착륙을 명시적 엣지로 갖고 있어 체계적으로 탐색한다.
    RRT*는 (a) 통로 근처에서 air 모드를 샘플하고
           (b) 그 지점에서 이륙 노드를 만들고
           (c) 반대편까지 날아가고
           (d) 착륙 가능한 지점을 찾아야 한다.
    네 가지가 우연히 맞아야 한다.

측정
  S1. 통로 장애물 높이를 바꿔가며 (넘을 필요 없음 -> 넘어야 함) 성공률
  S2. 필요한 모드 전환 횟수(통로 개수)를 늘려가며 성공률
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cost.energy import EnergyModel  # noqa: E402
from envs.heightmap import (  # noqa: E402
    HeightMap, WALL_HEIGHT, _blank, _border_walls, _rect,
)
from planners.state_space import ProblemSpec  # noqa: E402
from planners.grid_search import astar  # noqa: E402
from planners.rrt_star import rrt_star  # noqa: E402
from planners.smoothing import smooth_result  # noqa: E402

SEP = "=" * 84
model = EnergyModel.from_yaml()

N_SEEDS = 20
RRT_BUDGET = 2.0
GAP_W = 2.0          # 통로 폭은 넉넉하게 고정 — '좁음'이 아니라 '모드전환'만 보기 위해


def make_switch_map(obstacle_h: float, n_walls: int = 2,
                    res: float = 0.1) -> HeightMap:
    """통로에 높이 obstacle_h 장애물이 있는 맵.

    obstacle_h <= 0.15 이면 로버가 그냥 지나간다 (모드 전환 불필요).
    그보다 높으면 반드시 비행해야 한다.
    """
    W, H = 18.0, 12.0
    g = _blank(int(W / res), int(H / res))
    _border_walls(g, res)

    xs = np.linspace(5.0, 13.0, n_walls)
    for i, wx in enumerate(xs):
        gy = 3.0 if i % 2 == 0 else 9.0        # 통로를 엇갈리게
        _rect(g, wx, 0.0, wx + 0.6, H, WALL_HEIGHT, res)
        _rect(g, wx, gy - GAP_W / 2, wx + 0.6, gy + GAP_W / 2,
              obstacle_h, res)                  # 통로 = 장애물 높이

    return HeightMap(
        grid=g, resolution=res, name=f"switch_h{obstacle_h:.2f}_n{n_walls}",
        start=(1.5, 6.0), goal=(16.5, 6.0),
        description=f"통로 장애물 {obstacle_h}m, 격벽 {n_walls}개",
    )


def evaluate(hm):
    spec = ProblemSpec(hm=hm, model=model, dims=3)

    t0 = time.perf_counter()
    ra = astar(spec, timeout=60.0)
    ta = time.perf_counter() - t0
    if ra.found:
        ca, acc_a, _ = smooth_result(spec, ra, is_grid=True)
        a_sw = acc_a.n_switches
    else:
        ca, a_sw = None, None

    succ, firsts, costs, sws = 0, [], [], []
    for s in range(N_SEEDS):
        r = rrt_star(spec, seed=s, max_samples=10 ** 7, timeout=RRT_BUDGET)
        if r.found:
            succ += 1
            if r.first_solution_s is not None:
                firsts.append(r.first_solution_s)
            c, acc_r, _ = smooth_result(spec, r, is_grid=False)
            costs.append(c)
            sws.append(acc_r.n_switches)

    return {
        "a_found": ra.found, "a_cost": ca, "a_time": ta, "a_switches": a_sw,
        "r_rate": 100.0 * succ / N_SEEDS,
        "r_first": np.mean(firsts) if firsts else None,
        "r_cost": np.mean(costs) if costs else None,
        "r_switches": np.mean(sws) if sws else None,
    }


# ---------------------------------------------------------------- S1
print(SEP)
print("S1. 통로 장애물 높이별 — 모드 전환 필요성이 성공률에 미치는 영향")
print(SEP)
print(f"  통로 폭은 {GAP_W}m 로 고정 (좁음이 아니라 모드전환만 보기 위해)")
print(f"  격벽 2개, RRT* 예산 {RRT_BUDGET}초 x 시드 {N_SEEDS}개")
print()
print(f"  {'장애물h':>8} {'통과방법':>10} | {'A*':>9} {'A*전환':>7} {'A*시간':>8} "
      f"| {'RRT*성공':>9} {'RRT*비용':>10} {'RRT*전환':>9} {'첫해':>8}")
print("  " + "-" * 82)

HEIGHTS = [0.0, 0.10, 0.30, 0.60, 0.90, 1.20, 1.50]
s1 = []
for h in HEIGHTS:
    hm = make_switch_map(h, n_walls=2)
    r = evaluate(hm)
    s1.append((h, r))

    method = "주행" if h <= 0.15 else ("비행" if h <= 1.55 else "불가")
    a_s = f"{r['a_cost']:>9.3f}" if r["a_cost"] is not None else f"{'해없음':>9}"
    asw = f"{r['a_switches']:>7}" if r["a_switches"] is not None else f"{'-':>7}"
    rc = f"{r['r_cost']:>10.3f}" if r["r_cost"] is not None else f"{'-':>10}"
    rsw = f"{r['r_switches']:>9.1f}" if r["r_switches"] is not None else f"{'-':>9}"
    rf = f"{r['r_first']:>7.3f}s" if r["r_first"] is not None else f"{'-':>8}"
    print(f"  {h:>7.2f}m {method:>10} | {a_s} {asw} {r['a_time']:>7.3f}s "
          f"| {r['r_rate']:>8.0f}% {rc} {rsw} {rf}")


# ---------------------------------------------------------------- S2
print()
print(SEP)
print("S2. 필요한 모드 전환 횟수별 — 전환이 늘수록 샘플링이 불리해지는가")
print(SEP)
print(f"  통로 장애물 0.60m (반드시 비행), 폭 {GAP_W}m")
print()
print(f"  {'격벽수':>7} {'필요전환':>9} | {'A*':>9} {'A*시간':>8} "
      f"| {'RRT*성공':>9} {'RRT*비용':>10} {'첫해':>8} {'vs A*':>8}")
print("  " + "-" * 76)

for n in (1, 2, 3, 4):
    hm = make_switch_map(0.60, n_walls=n)
    r = evaluate(hm)
    a_s = f"{r['a_cost']:>9.3f}" if r["a_cost"] is not None else f"{'해없음':>9}"
    rc = f"{r['r_cost']:>10.3f}" if r["r_cost"] is not None else f"{'-':>10}"
    rf = f"{r['r_first']:>7.3f}s" if r["r_first"] is not None else f"{'-':>8}"
    if r["a_cost"] and r["r_cost"]:
        rel = f"{100*(r['r_cost']-r['a_cost'])/r['a_cost']:>+7.2f}%"
    else:
        rel = f"{'-':>8}"
    need = r["a_switches"] if r["a_switches"] is not None else "-"
    print(f"  {n:>7} {need:>9} | {a_s} {r['a_time']:>7.3f}s "
          f"| {r['r_rate']:>8.0f}% {rc} {rf} {rel}")


# ---------------------------------------------------------------- 해석
print()
print(SEP)
print("해석")
print(SEP)
need_fly = [(h, r) for h, r in s1 if h > 0.15 and h <= 1.55]
no_fly = [(h, r) for h, r in s1 if h <= 0.15]

if no_fly:
    m = np.mean([r["r_rate"] for _, r in no_fly])
    print(f"  주행으로 통과 가능한 경우 RRT* 평균 성공률: {m:.0f}%")
if need_fly:
    m = np.mean([r["r_rate"] for _, r in need_fly])
    print(f"  비행이 필요한 경우   RRT* 평균 성공률: {m:.0f}%")
    drop = [(h, r["r_rate"]) for h, r in need_fly if r["r_rate"] < 100]
    if drop:
        print(f"  성공률 100% 미만: {', '.join(f'{h}m={r:.0f}%' for h, r in drop)}")
        print("  => 모드 전환이 필요할 때 샘플링 기반이 불리하다는 근거가 된다.")
    else:
        print("  => 비행이 필요해도 RRT*가 전부 성공했다.")
        print("     논거 4는 이 실험으로도 뒷받침되지 않는다. 정직하게 보고할 것.")
