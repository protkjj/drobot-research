"""5단계 검증 — RRT* 구현이 올바른가.

목적이 'A* 채택 근거 확보'이므로 이 검증이 특히 중요하다.
RRT*가 잘못 구현돼 있으면 A*의 승리는 무의미하다.
"약한 상대를 이겼다"는 반박에 답할 수 없기 때문.

검증 항목
  R1. 해를 찾는가
  R2. 보고한 비용이 경로를 재계산한 값과 일치하는가 (비용 회계 버그 탐지)
  R3. 예산(샘플 수)을 늘리면 비용이 단조 감소하며 A* 최적해에 수렴하는가
      <- 이게 핵심. RRT*의 asymptotic optimality 가 작동하는지 확인
  R4. Informed 샘플링이 실제로 도움이 되는가
  R5. 시드마다 결과가 얼마나 다른가 (A*의 '결정론' 논거를 뒷받침할 데이터)
"""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cost.energy import EnergyModel  # noqa: E402
from envs.heightmap import build_all  # noqa: E402
from planners.state_space import ProblemSpec  # noqa: E402
from planners.grid_search import astar  # noqa: E402
from planners.rrt_star import rrt_star, HybridRRTStar  # noqa: E402

SEP = "=" * 78
model = EnergyModel.from_yaml()
maps = build_all(res=0.1)

TEST_MAPS = ["easy_open", "medium_open", "hard_open"]


# ---------------------------------------------------------------- R1, R2
print(SEP)
print("R1/R2. RRT*가 해를 찾는가 / 비용 회계가 맞는가")
print(SEP)
print()

issues = []
for name in TEST_MAPS:
    spec = ProblemSpec(hm=maps[name], model=model, dims=3)
    r = rrt_star(spec, seed=0, max_samples=8000, timeout=None)
    print(f"  [{name}]")
    print(f"     {r.brief()}")
    if not r.found:
        issues.append(f"{name}: RRT*가 해를 못 찾음 (샘플 8000)")
        continue

    # 경로 좌표로부터 비용을 다시 쌓아본다
    planner = HybridRRTStar(spec, seed=0)
    planner.nodes = []
    from planners.rrt_star import Node  # noqa: E402
    from cost.energy import CostAccumulator  # noqa: E402
    recomputed = CostAccumulator()
    ok = True
    for a, b in zip(r.path, r.path[1:]):
        na = Node(x=a[0], y=a[1], z=a[2], mode=a[3])
        na.acc = recomputed
        e = planner._edge_cost(na, b[0], b[1], b[2], b[3])
        if e is None:
            ok = False
            issues.append(f"{name}: 경로에 유효하지 않은 전이 {a} -> {b}")
            break
        recomputed = recomputed + e
    if ok:
        c_re = model.cost(recomputed)
        diff = abs(c_re - r.cost)
        status = "일치" if diff < 1e-6 else f"불일치 {diff:.6f}"
        print(f"     비용 재계산: {c_re:.4f} (보고 {r.cost:.4f}) -> {status}")
        if diff > 1e-6:
            issues.append(f"{name}: 비용 회계 불일치 {diff:.6f}")
    print()


# ---------------------------------------------------------------- R3
print(SEP)
print("R3. 예산을 늘리면 A* 최적해에 수렴하는가  <- 가장 중요")
print(SEP)
print("  RRT*는 샘플이 무한대로 가면 최적해에 수렴해야 한다(asymptotic optimality).")
print("  수렴하지 않으면 구현에 문제가 있는 것이고, 그 상태의 비교는 무의미하다.")
print()

BUDGETS = [500, 1000, 2000, 5000, 10000, 20000, 40000]
converge_ok = True

for name in TEST_MAPS:
    spec = ProblemSpec(hm=maps[name], model=model, dims=3)
    ra = astar(spec, timeout=120.0)
    opt = ra.cost
    print(f"  [{name}]  A* 최적해 C = {opt:.3f}")
    print(f"     {'샘플':>7} {'RRT* 최선':>11} {'갭':>9} {'상대갭':>8} {'시간':>8}")
    print("     " + "-" * 48)

    prev = float("inf")
    monotone = True
    last_gap = None
    for bud in BUDGETS:
        # 시드 3개 중 최선 (RRT*에게 유리하게)
        best = None
        total_t = 0.0
        for s in range(3):
            r = rrt_star(spec, seed=s, max_samples=bud, timeout=None)
            total_t += r.runtime_s
            if r.found and (best is None or r.cost < best.cost):
                best = r
        if best is None:
            print(f"     {bud:>7} {'해없음':>11}")
            continue
        gap = best.cost - opt
        rel = 100 * gap / opt
        last_gap = rel
        if best.cost > prev + 1e-9:
            monotone = False
        prev = min(prev, best.cost)
        print(f"     {bud:>7} {best.cost:>11.3f} {gap:>+9.3f} {rel:>7.2f}% "
              f"{total_t/3:>7.3f}s")

    print(f"     => 단조 개선: {monotone},  최종 상대갭: "
          f"{last_gap:.2f}%" if last_gap is not None else "")
    if last_gap is not None and last_gap > 15.0:
        converge_ok = False
        issues.append(f"{name}: 4만 샘플에도 갭이 {last_gap:.1f}% — 수렴 의심")
    print()


# ---------------------------------------------------------------- R4
print(SEP)
print("R4. Informed 샘플링이 실제로 도움이 되는가")
print(SEP)
print(f"  {'맵':<15} {'예산':>7} {'informed=OFF':>13} {'informed=ON':>12} {'개선':>8}")
print("  " + "-" * 60)
for name in TEST_MAPS:
    spec = ProblemSpec(hm=maps[name], model=model, dims=3)
    for bud in (2000, 10000):
        offs, ons = [], []
        for s in range(3):
            r0 = rrt_star(spec, seed=s, max_samples=bud, timeout=None, informed=False)
            r1 = rrt_star(spec, seed=s, max_samples=bud, timeout=None, informed=True)
            if r0.found:
                offs.append(r0.cost)
            if r1.found:
                ons.append(r1.cost)
        if not offs or not ons:
            continue
        c0, c1 = min(offs), min(ons)
        imp = 100 * (c0 - c1) / c0
        print(f"  {name:<15} {bud:>7} {c0:>13.3f} {c1:>12.3f} {imp:>+7.2f}%")


# ---------------------------------------------------------------- R5
print()
print(SEP)
print("R5. 시드에 따른 결과 변동 — A*의 '결정론' 논거용 데이터")
print(SEP)
N_SEEDS = 12
BUD = 5000
print(f"  샘플 {BUD}, 시드 {N_SEEDS}개")
print()
print(f"  {'맵':<15} {'A*(최적)':>10} {'RRT*평균':>10} {'표준편차':>9} "
      f"{'최선':>9} {'최악':>9} {'실패':>5}")
print("  " + "-" * 72)
for name in TEST_MAPS:
    spec = ProblemSpec(hm=maps[name], model=model, dims=3)
    ra = astar(spec, timeout=120.0)
    costs = []
    fails = 0
    for s in range(N_SEEDS):
        r = rrt_star(spec, seed=s, max_samples=BUD, timeout=None)
        if r.found:
            costs.append(r.cost)
        else:
            fails += 1
    if costs:
        arr = np.array(costs)
        print(f"  {name:<15} {ra.cost:>10.3f} {arr.mean():>10.3f} "
              f"{arr.std():>9.3f} {arr.min():>9.3f} {arr.max():>9.3f} {fails:>5}")
    else:
        print(f"  {name:<15} {ra.cost:>10.3f} {'전부 실패':>10} {'':>9} "
              f"{'':>9} {'':>9} {fails:>5}")

print()
print("  => A*는 표준편차 0 (항상 같은 답). RRT*의 표준편차가 이 논거의 근거가 된다.")


# ---------------------------------------------------------------- 요약
print()
print(SEP)
print("검증 요약")
print(SEP)
if issues:
    print("  !!! 문제 발견 — 이 상태로 비교하면 안 된다 !!!")
    for i in issues:
        print(f"    - {i}")
else:
    print("  R1 해 탐색        : PASS")
    print("  R2 비용 회계      : PASS")
    print("  R3 최적해 수렴    : PASS")
    print()
    print("  => RRT* 구현이 정상이다. 공정한 비교가 가능하다.")
