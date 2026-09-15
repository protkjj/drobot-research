"""narrow passage 강건성 — A* 우위 논거 4번의 정량화.

주장: 실내 환경의 좁은 통로는 샘플링 기반 플래너의 고전적 약점이다.
      통로가 좁을수록 랜덤 샘플이 그 안에 떨어질 확률이 급격히 낮아진다.
      격자 탐색은 통로 폭과 무관하게 (해상도 이상이면) 항상 찾는다.

이걸 통제된 실험으로 확인한다:
    통로 폭만 바꾼 맵을 여러 개 만들고, 두 플래너의 성공률/시간을 측정.

측정
  N1. 통로 폭별 성공률 (시드 20개)
  N2. 통로 폭별 첫 해 도달 시간
  N3. A*의 통로 폭 의존성 (없어야 정상)
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cost.energy import EnergyModel  # noqa: E402
from envs.heightmap import HeightMap, WALL_HEIGHT, _blank, _border_walls, _rect  # noqa: E402
from planners.state_space import ProblemSpec  # noqa: E402
from planners.grid_search import astar  # noqa: E402
from planners.rrt_star import rrt_star  # noqa: E402
from planners.smoothing import smooth_result  # noqa: E402

SEP = "=" * 80
model = EnergyModel.from_yaml()


def make_narrow_map(gap_w: float, res: float = 0.1) -> HeightMap:
    """통로 폭만 다른 맵. 다른 조건은 전부 동일하게 통제한다.

    구조: 세로 격벽 3개. 통로를 서로 '엇갈리게' 배치해 지그재그를 강요한다.

    1차 설계 실패: 통로를 전부 y=6.0(= start/goal 과 같은 높이)에 두었더니
    두 플래너 모두 직선으로 통과해버려서 통로 폭이 결과에 영향을 주지 않았다
    (모든 폭에서 비용 28.167로 동일, A* 0.002초).
    통로가 직선상에 있으면 goal bias 만으로도 쉽게 찾히므로
    narrow passage 의 어려움이 전혀 재현되지 않는다.
    """
    W, H = 18.0, 12.0
    g = _blank(int(W / res), int(H / res))
    _border_walls(g, res)

    # (벽 x위치, 통로 중심 y) — y를 위/아래로 엇갈리게
    for wx, gy in ((5.0, 2.5), (9.5, 9.5), (14.0, 2.5)):
        _rect(g, wx, 0.0, wx + 0.5, H, WALL_HEIGHT, res)          # 격벽
        _rect(g, wx, gy - gap_w / 2, wx + gap_w * 0 + 0.5,         # 통로 (뚫림)
              gy + gap_w / 2, 0.0, res)

    return HeightMap(
        grid=g, resolution=res, name=f"narrow_{gap_w:.1f}m",
        start=(1.5, 6.0), goal=(16.5, 6.0),
        description=f"통로 폭 {gap_w}m (엇갈림 배치)",
    )


GAPS = [3.0, 2.0, 1.5, 1.0, 0.8, 0.6, 0.4]
N_SEEDS = 20
RRT_BUDGET = 2.0

print(SEP)
print("narrow passage 실험 — 통로 폭만 바꿔가며 성공률 측정")
print(SEP)
print(f"  맵 16x12m, 격벽 2개. 통로 외에는 통과 불가.")
print(f"  RRT* 예산 {RRT_BUDGET}초, 시드 {N_SEEDS}개")
print()
print(f"  {'통로폭':>7} {'통로셀':>7} | {'A*성공':>7} {'A*시간':>8} {'A*비용':>9} "
      f"| {'RRT*성공':>9} {'첫해시간':>10} {'RRT*평균비용':>12}")
print("  " + "-" * 84)

rows = []
for gap in GAPS:
    hm = make_narrow_map(gap)
    spec = ProblemSpec(hm=hm, model=model, dims=3)
    n_cells = int(round(gap / hm.resolution))

    # --- A* (결정론적이라 1회) ---
    t0 = time.perf_counter()
    ra = astar(spec, timeout=60.0)
    ta = time.perf_counter() - t0
    if ra.found:
        ca, _, _ = smooth_result(spec, ra, is_grid=True)
        a_ok = True
    else:
        ca, a_ok = float("nan"), False

    # --- RRT* (시드 N개) ---
    # 시간은 runtime_s 가 아니라 first_solution_s 를 쓴다.
    # RRT*는 timeout까지 해를 계속 개선하므로 runtime_s는 늘 예산과 같아서
    # '얼마나 빨리 찾았나'를 전혀 보여주지 못한다.
    succ, times, costs = 0, [], []
    for s in range(N_SEEDS):
        r = rrt_star(spec, seed=s, max_samples=10**7, timeout=RRT_BUDGET)
        if r.found:
            succ += 1
            if r.first_solution_s is not None:
                times.append(r.first_solution_s)
            c, _, _ = smooth_result(spec, r, is_grid=False)
            costs.append(c)

    rrt_rate = 100.0 * succ / N_SEEDS
    tm = np.mean(times) if times else float("nan")
    cm = np.mean(costs) if costs else float("nan")

    rows.append((gap, n_cells, a_ok, ta, ca, rrt_rate, tm, cm))
    a_s = "OK" if a_ok else "실패"
    ca_s = f"{ca:>9.3f}" if a_ok else f"{'-':>9}"
    tm_s = f"{tm:>9.3f}s" if times else f"{'-':>10}"
    cm_s = f"{cm:>12.3f}" if costs else f"{'-':>12}"
    print(f"  {gap:>6.1f}m {n_cells:>7} | {a_s:>7} {ta:>7.3f}s {ca_s} "
          f"| {rrt_rate:>8.0f}% {tm_s} {cm_s}")

print()
print(SEP)
print("해석")
print(SEP)

a_all_ok = all(r[2] for r in rows)
print(f"  A* 전 구간 성공: {a_all_ok}")
if a_all_ok:
    a_times = [r[3] for r in rows]
    print(f"     시간 범위 {min(a_times):.3f}s ~ {max(a_times):.3f}s "
          f"(변동 {max(a_times)/min(a_times):.1f}배)")
    print("     => A*는 통로 폭에 거의 영향받지 않는다. 격자 해상도(0.1m) 이상이면 찾는다.")

print()
degraded = [r for r in rows if r[5] < 100.0]
if degraded:
    print("  RRT* 성공률이 100% 미만인 구간:")
    for gap, ncell, _, _, _, rate, _, _ in degraded:
        print(f"     통로 {gap}m ({ncell}셀): {rate:.0f}%")
    worst = min(rows, key=lambda r: r[5])
    print(f"     => 최저 성공률 {worst[5]:.0f}% (통로 {worst[0]}m)")
else:
    print("  RRT*도 전 구간 100% 성공 — 통로가 충분히 넓거나 예산이 충분했다.")
    print("     (이 경우 논거 4는 이 실험으로 뒷받침되지 않는다. 정직하게 보고할 것)")
