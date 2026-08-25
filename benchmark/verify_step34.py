"""3-4단계 검증 — Dijkstra(참조 최적해) 와 A*.

검증 항목
  D1. Dijkstra가 해를 찾는가 / 못 찾아야 할 때 못 찾는가
  D2. 보고된 비용이 경로를 재계산한 값과 일치하는가 (플래너 버그 탐지)
  A1. A*가 Dijkstra와 '같은 비용'의 해를 내는가  <- 휴리스틱 admissible 검증
  A2. A*가 Dijkstra보다 적게 확장하는가 (휴리스틱이 실제로 작동하는가)
  A3. h(n)=0 으로 두면 A*가 Dijkstra와 완전히 동일해지는가 (구현 일치 확인)

A1이 핵심이다. 여기서 A*가 더 싼 해를 내면 Dijkstra에 버그가 있고,
더 비싼 해를 내면 휴리스틱이 과대평가(inadmissible)라는 뜻이다.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cost.energy import EnergyModel  # noqa: E402
from envs.heightmap import build_all  # noqa: E402
from planners.state_space import ProblemSpec, GROUND, AIR  # noqa: E402
from planners.grid_search import dijkstra, astar, _search  # noqa: E402

SEP = "=" * 78
model = EnergyModel.from_yaml()
maps = build_all(res=0.1)

print(SEP)
print("3-4단계 검증 — Dijkstra / A* (3D 상태공간)")
print(SEP)
print()

results = {}
issues = []

for name, hm in maps.items():
    spec = ProblemSpec(hm=hm, model=model, dims=3)
    s = spec.summary()

    print(f"[{name}]  상태수 {s['n_states']:,}  z_max={s['z_max']}  "
          f"통과가능 h<={s['h_limit']}")

    rd = dijkstra(spec, timeout=120.0)
    ra = astar(spec, timeout=120.0)
    results[name] = (rd, ra)

    print(f"   {rd.brief()}")
    print(f"   {ra.brief()}")

    # -- D2: 비용 재계산 일치 --
    for r in (rd, ra):
        if r.found:
            recomputed = spec.path_cost(r.path)
            c_re = spec.model.cost(recomputed)
            if abs(c_re - r.cost) > 1e-6:
                issues.append(f"{name}/{r.planner}: 보고 비용 {r.cost:.6f} != "
                              f"재계산 {c_re:.6f}")

    # -- A1: 같은 비용인가 --
    if rd.found != ra.found:
        issues.append(f"{name}: Dijkstra found={rd.found} 인데 A* found={ra.found}")
    elif rd.found and abs(rd.cost - ra.cost) > 1e-6:
        issues.append(f"{name}: 비용 불일치 Dijkstra={rd.cost:.6f} A*={ra.cost:.6f} "
                      f"(차이 {ra.cost - rd.cost:+.6f})")

    # -- A2: 확장 수 비교 --
    if rd.found and ra.found:
        ratio = ra.n_expanded / rd.n_expanded if rd.n_expanded else float("nan")
        speedup = rd.runtime_s / ra.runtime_s if ra.runtime_s > 0 else float("nan")
        print(f"   확장 비율 A*/Dijkstra = {ratio:.3f}  "
              f"(A*가 {100*(1-ratio):.1f}% 적게 확장)   속도 {speedup:.2f}배")
    print()


# ---------------------------------------------------------------- A3
print(SEP)
print("A3. h(n)=0 이면 A* == Dijkstra 인가 (구현 일치 확인)")
print(SEP)
hm = maps["easy_open"]
spec = ProblemSpec(hm=hm, model=model, dims=3)
r_dij = dijkstra(spec)
r_a0 = _search(spec, use_heuristic=False, name="A*(h=0)")
same = (r_dij.cost == r_a0.cost and r_dij.n_expanded == r_a0.n_expanded)
print(f"  Dijkstra   : C={r_dij.cost:.6f}  확장={r_dij.n_expanded:,}")
print(f"  A*(h=0)    : C={r_a0.cost:.6f}  확장={r_a0.n_expanded:,}")
print(f"  => {'동일 (구현 일치)' if same else '불일치 !!'}")
if not same:
    issues.append("h=0 A*가 Dijkstra와 다름")


# ---------------------------------------------------------------- 경로 내용
print()
print(SEP)
print("찾은 경로의 성격 — 모드 전환이 실제로 일어나는가")
print(SEP)
print(f"  {'맵':<17} {'전환':>5} {'지상거리':>9} {'비행거리':>9} "
      f"{'지상E':>8} {'비행E':>8} {'전환E':>8}")
print("  " + "-" * 70)
for name, (rd, _) in results.items():
    if not rd.found:
        print(f"  {name:<17} (해 없음)")
        continue
    a = rd.acc
    print(f"  {name:<17} {a.n_switches:>5} {a.dist_ground:>8.2f}m "
          f"{a.dist_air:>8.2f}m {a.e_ground:>7.2f}W {a.e_air:>7.2f}W "
          f"{a.e_switch:>7.2f}W")


# ---------------------------------------------------------------- 요약
print()
print(SEP)
print("검증 요약")
print(SEP)
if issues:
    print("  !!! 문제 발견 !!!")
    for i in issues:
        print(f"    - {i}")
else:
    print("  D2 비용 재계산 일치     : PASS")
    print("  A1 A* == Dijkstra 비용  : PASS")
    print("  A3 h=0 구현 일치        : PASS")
    print()
    print("  => A*의 휴리스틱이 admissible 하고, 두 구현이 같은 문제를 풀고 있다.")
