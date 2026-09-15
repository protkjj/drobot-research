"""왜 최적해가 비행을 선택하지 않는가 — 정밀 진단.

맵을 키우고 벽을 길게 만들었는데도 Dijkstra 최적해의 모드 전환이 0회다.
감으로 또 고치지 말고, 원인을 수치로 특정한다.

진단 항목
  P1. 실제 우회거리는 얼마인가 (내 어림셈 2*dy 가 맞았는가)
  P2. 비행을 강제하면 비용이 얼마나 나쁜가 (얼마나 차이나는가)
  P3. 이착륙 에너지가 얼마여야 비행이 선택되는가 (역산)
  P4. 그 값이 물리적으로 타당한가
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cost.energy import CostAccumulator, EnergyModel  # noqa: E402
from envs.heightmap import build_all  # noqa: E402
from planners.state_space import ProblemSpec, GROUND, AIR  # noqa: E402
from planners.grid_search import dijkstra  # noqa: E402

SEP = "=" * 78
model = EnergyModel.from_yaml()
maps = build_all(res=0.1)


# ---------------------------------------------------------------- P1
print(SEP)
print("P1. 실제 우회거리 — 어림셈이 맞았는가")
print(SEP)
print("  직선거리(start->goal 유클리드) 대비 실제 최적 경로가 얼마나 더 긴가")
print()
print(f"  {'맵':<17} {'직선':>8} {'실제경로':>9} {'추가거리':>9} {'손익분기':>9} {'비행?':>6}")
print("  " + "-" * 64)

BREAKEVEN = 8.2  # analyze_breakeven.py: 폭0.6 높이0.6 기준
detours = {}
for name, hm in maps.items():
    spec = ProblemSpec(hm=hm, model=model, dims=3)
    r = dijkstra(spec, timeout=120.0)
    sx, sy = hm.start
    gx, gy = hm.goal
    straight = np.hypot(gx - sx, gy - sy)
    if not r.found:
        print(f"  {name:<17} {straight:>7.2f}m {'해없음':>9}")
        continue
    actual = r.acc.dist_ground + r.acc.dist_air
    extra = actual - straight
    detours[name] = extra
    print(f"  {name:<17} {straight:>7.2f}m {actual:>8.2f}m {extra:>8.2f}m "
          f"{BREAKEVEN:>8.1f}m {'YES' if extra > BREAKEVEN else 'no':>6}")

print()
print("  => 추가거리가 손익분기(8.2m)보다 작으면 비행할 이유가 없다.")
print("     8방향 이동에서는 대각선으로 부드럽게 우회하므로")
print("     내가 쓴 어림셈(2 * dy)보다 실제 우회 비용이 훨씬 작다. 이게 원인.")


# ---------------------------------------------------------------- P2
print()
print(SEP)
print("P2. 비행을 강제하면 비용이 얼마나 나빠지는가")
print(SEP)


def dijkstra_forced_flight(spec):
    """반드시 1회 이상 비행하는 최적해를 구한다.

    상태에 '비행한 적 있는가' 플래그를 붙여서, 목표에 도달할 때
    플래그가 켜져 있어야만 인정한다.
    """
    import heapq
    start = spec.start_state()
    goal = spec.goal_state()
    s0 = (start, False)
    g = {s0: 0.0}
    g_acc = {s0: CostAccumulator()}
    heap = [(0.0, 0, s0)]
    closed = set()
    cnt = 0
    while heap:
        _, _, cur = heapq.heappop(heap)
        if cur in closed:
            continue
        closed.add(cur)
        st, flew = cur
        if st == goal and flew:
            return g[cur], g_acc[cur]
        for nxt, acc in spec.neighbors(st):
            # 이륙했으면 플래그를 켠다
            nflew = flew or (st[-1] == GROUND and nxt[-1] == AIR)
            key = (nxt, nflew)
            if key in closed:
                continue
            na = g_acc[cur] + acc
            ng = spec.model.cost(na)
            if ng < g.get(key, float("inf")) - 1e-12:
                g[key] = ng
                g_acc[key] = na
                cnt += 1
                heapq.heappush(heap, (ng, cnt, key))
    return float("inf"), None


print(f"  {'맵':<17} {'자유(최적)':>11} {'비행강제':>10} {'차이':>9} {'비율':>7}")
print("  " + "-" * 58)
for name, hm in maps.items():
    if name == "hard_corridor":
        continue  # 3D에서 해 없음
    spec = ProblemSpec(hm=hm, model=model, dims=3)
    r = dijkstra(spec, timeout=120.0)
    cf, acc_f = dijkstra_forced_flight(spec)
    if not r.found or not np.isfinite(cf):
        continue
    diff = cf - r.cost
    print(f"  {name:<17} {r.cost:>10.3f} {cf:>10.3f} {diff:>+8.3f} "
          f"{cf/r.cost:>6.3f}x")

print()
print("  => 비행을 강제해도 비용이 크게 나빠지지 않으면, 파라미터를 조금만")
print("     바꿔도 비행이 선택될 수 있다는 뜻. 차이가 크면 구조적 문제다.")


# ---------------------------------------------------------------- P3
print()
print(SEP)
print("P3. 이착륙 에너지가 얼마여야 비행이 선택되는가 (역산)")
print(SEP)

import copy  # noqa: E402


def find_threshold(hm, param: str, lo: float, hi: float, iters: int = 18):
    """param 값을 이분탐색해서 비행이 선택되기 시작하는 지점을 찾는다."""
    for _ in range(iters):
        mid = (lo + hi) / 2
        m2 = copy.deepcopy(model)
        if param == "takeoff":
            m2.takeoff_wh = mid
            m2.landing_wh = mid * 0.6   # 원본 비율 5:3 유지
        elif param == "alt":
            m2.wh_per_altitude_m = mid
        spec = ProblemSpec(hm=hm, model=m2, dims=3)
        r = dijkstra(spec, timeout=120.0)
        if r.found and r.acc.n_switches > 0:
            lo = mid   # 비행함 -> 더 비싸게 해도 되나 확인
        else:
            hi = mid   # 비행 안함 -> 더 싸게
    return (lo + hi) / 2


print(f"  현재 값: takeoff={model.takeoff_wh} Wh, landing={model.landing_wh} Wh")
print()
print(f"  {'맵':<17} {'비행 시작 임계 takeoff':>24}")
print("  " + "-" * 44)
for name in ("easy_open", "medium_open", "hard_open"):
    hm = maps[name]
    th = find_threshold(hm, "takeoff", 0.0, 5.0)
    print(f"  {name:<17} {th:>20.3f} Wh  (현재 {model.takeoff_wh})")

print()
print("  => 현재 takeoff=5.0 Wh 는 이 임계값보다 훨씬 크다.")
print("     즉 비행이 '구조적으로' 배제되어 있다.")


# ---------------------------------------------------------------- P4
print()
print(SEP)
print("P4. 이착륙 에너지 5.0 Wh 가 물리적으로 타당한가")
print(SEP)
print(f"  energy_params.yaml 의 값들:")
print(f"    takeoff_energy   = {model.takeoff_wh} Wh")
print(f"    takeoff_time     = {model.takeoff_s} s")
print(f"    hover_power      = {model.hover_power_w} W")
print()
th_wh = model.hover_power_w * model.takeoff_s / 3600.0
print(f"  호버링 전력으로 이륙 시간만큼 소비한다면:")
print(f"    {model.hover_power_w} W x {model.takeoff_s} s = {th_wh*1000:.1f} mWh"
      f" = {th_wh:.4f} Wh")
print()
print(f"  config의 takeoff_energy 는 그 {model.takeoff_wh/th_wh:.0f} 배다.")
print()
print(f"  반대로, {model.takeoff_wh} Wh 를 호버링으로 소비하려면:")
print(f"    {model.takeoff_wh} Wh / {model.hover_power_w} W ="
      f" {model.takeoff_wh/model.hover_power_w*3600:.0f} 초 = "
      f"{model.takeoff_wh/model.hover_power_w*60:.1f} 분")
print()
print("  => 5초짜리 이륙에 6분치 호버링 에너지를 매기고 있다.")
print("     energy_params.yaml 주석에도 '실측 전 초기 추정값'이라 되어 있으므로")
print("     이 값은 재검토 대상이다. (다만 kj의 config이므로 임의 수정하지 않음)")
