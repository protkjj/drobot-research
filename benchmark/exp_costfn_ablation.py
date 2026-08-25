"""비용함수 형태 전후 비교 — 에너지 모델링 적용의 효과.

두 형태를 같은 조건에서 재측정한다.

  (전) C = alpha*E_motion + beta*E_switch + gamma*T
       alpha=1.0, beta=1.0, gamma=0.5
       에너지(Wh)와 시간(s)을 그대로 더해 단위가 섞이고,
       가중치가 단위 변환을 겸한다.

  (후) C = wE*E/E_ref + wS*E_switch/E_switch_ref + wT*T/T_ref
       wE=0.5, wS=0.2, wT=0.3
       세 항이 모두 무차원. 참조값은 파라미터에서 유도한다.

두 형태는 alpha = wE/E_ref 로 정확히 동등하므로, 같은 비용함수를
다르게 '표현'한 것이다. 그런데 표현을 바꾸니 기존 가중치가
모드 전환에 79% 를 주고 있었다는 게 드러났고, 그래서 최적해가
비행을 한 번도 선택하지 않았다.

측정
  A. 각 형태에서 가중치가 실제로 어떻게 배분되는가
  B. 각 형태에서 최적해가 비행을 선택하는가
  C. 각 형태에서 A* vs RRT* 승패가 어떻게 갈리는가

실행: python3 benchmark/exp_costfn_ablation.py
"""
from __future__ import annotations

import copy
import json
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

RESULTS = Path(__file__).resolve().parent / "results"
RESULTS.mkdir(parents=True, exist_ok=True)

SEP = "=" * 88
MAPS = ["easy_open", "easy_corridor", "medium_open", "medium_corridor", "hard_open"]
N_SEEDS = 20
BUDGET = 2.0


def make_model(kind: str) -> EnergyModel:
    """비용함수 형태별 모델을 만든다.

    파라미터(에너지 값)는 동일하고 '가중치 표현'만 다르다.
    """
    m = copy.deepcopy(EnergyModel.from_yaml())
    if kind == "before":
        # 이전 설정 alpha=1.0, beta=1.0, gamma=0.5 를 정규화 가중치로 환산.
        # (energy.py 는 이제 정규화 형태만 계산하므로 등가 가중치를 넣는다)
        m.w_energy = 1.0 * m.e_ref            # alpha * E_ref
        m.w_switch = 1.0 * m.e_switch_ref     # beta  * E_switch_ref
        m.w_time = 0.5 * m.t_ref              # gamma * T_ref
    else:
        m.w_energy, m.w_switch, m.w_time = 0.5, 0.2, 0.3
    return m


def weight_share(m: EnergyModel) -> dict:
    tot = m.w_energy + m.w_switch + m.w_time
    return {
        "w_energy": m.w_energy, "w_switch": m.w_switch, "w_time": m.w_time,
        "share_energy": 100 * m.w_energy / tot,
        "share_switch": 100 * m.w_switch / tot,
        "share_time": 100 * m.w_time / tot,
        "alpha": m.alpha, "beta": m.beta, "gamma": m.gamma,
    }


def run_condition(kind: str, maps: dict) -> dict:
    m = make_model(kind)
    out = {"kind": kind, "weights": weight_share(m), "rows": []}

    for name in MAPS:
        spec = ProblemSpec(hm=maps[name], model=m, dims=3)

        t0 = time.perf_counter()
        ra = astar(spec, timeout=60.0)
        if ra.found:
            ca, acc_a, _ = smooth_result(spec, ra, is_grid=True)
            a_sw = acc_a.n_switches
            a_energy = acc_a.e_total
        else:
            ca, a_sw, a_energy = None, None, None
        ta = time.perf_counter() - t0

        costs, sws, fails = [], [], 0
        for s in range(N_SEEDS):
            r = rrt_star(spec, seed=s, max_samples=10 ** 7, timeout=BUDGET)
            if r.found:
                c, acc_r, _ = smooth_result(spec, r, is_grid=False)
                costs.append(c)
                sws.append(acc_r.n_switches)
            else:
                fails += 1

        row = {
            "map": name,
            "astar_cost": ca, "astar_time_s": round(ta, 4),
            "astar_switches": a_sw, "astar_energy_wh": a_energy,
            "rrt_mean": float(np.mean(costs)) if costs else None,
            "rrt_std": float(np.std(costs)) if costs else None,
            "rrt_best": float(np.min(costs)) if costs else None,
            "rrt_switches_mean": float(np.mean(sws)) if sws else None,
            "rrt_fails": fails,
        }
        if ca is not None and costs:
            row["astar_advantage_pct"] = 100 * (np.mean(costs) - ca) / ca
        out["rows"].append(row)
    return out


def main() -> None:
    maps = build_all(res=0.1)

    print(SEP)
    print("비용함수 형태 전후 비교")
    print(SEP)

    results = {}
    for kind, label in [("before", "전 — alpha/beta/gamma"),
                        ("after", "후 — 정규화")]:
        print()
        print(f"[{label}]")
        r = run_condition(kind, maps)
        results[kind] = r

        w = r["weights"]
        print(f"  가중치 배분:  에너지 {w['share_energy']:5.1f}%  "
              f"전환 {w['share_switch']:5.1f}%  시간 {w['share_time']:5.1f}%")
        print(f"  등가 계수:    alpha={w['alpha']:.4f}  beta={w['beta']:.4f}  "
              f"gamma={w['gamma']:.4f}")
        print()
        print(f"  {'맵':<17} {'A*':>9} {'전환':>5} {'RRT*평균':>9} {'전환':>5} "
              f"{'A*우위':>8}")
        print("  " + "-" * 60)
        for row in r["rows"]:
            adv = row.get("astar_advantage_pct")
            adv_s = f"{adv:>+7.2f}%" if adv is not None else f"{'-':>8}"
            print(f"  {row['map']:<17} {row['astar_cost']:>9.3f} "
                  f"{row['astar_switches']:>5} {row['rrt_mean']:>9.3f} "
                  f"{row['rrt_switches_mean']:>5.1f} {adv_s}")

    # ---- 요약 ----
    print()
    print(SEP)
    print("요약")
    print(SEP)
    for kind, label in [("before", "전"), ("after", "후")]:
        rows = results[kind]["rows"]
        wins = sum(1 for r in rows
                   if r.get("astar_advantage_pct") is not None
                   and r["astar_advantage_pct"] >= 0)
        tot = sum(1 for r in rows if r.get("astar_advantage_pct") is not None)
        fly = sum(r["astar_switches"] for r in rows if r["astar_switches"])
        advs = [r["astar_advantage_pct"] for r in rows
                if r.get("astar_advantage_pct") is not None]
        print(f"  [{label}] A* 우위 {wins}/{tot} 맵 | "
              f"A* 총 모드전환 {fly}회 | "
              f"평균 우위 {np.mean(advs):+.2f}% (범위 {min(advs):+.2f}~{max(advs):+.2f}%)")

    out = RESULTS / "benchmark_costfn_ablation.json"
    with open(out, "w") as f:
        json.dump(results, f, indent=1, ensure_ascii=False)
    print()
    print(f"저장: {out.name}")


if __name__ == "__main__":
    main()
