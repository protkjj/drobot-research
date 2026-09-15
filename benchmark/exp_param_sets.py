"""파라미터 세트 두 벌 비교 — 이착륙 에너지가 결론을 어떻게 바꾸는가.

왜 이 실험이 필요한가
    이착륙 에너지는 INA226 실측 전이라 확정값이 없다. 그런데 이 값 하나가
    비행 선택 여부를 통째로 좌우한다. 전환 1회 고정비를 평지 주행으로 환산하면

        default   이륙 5.0 + 착륙 3.0 = 8.0 Wh  ->  16.0 m
        derived   이륙 0.5 + 착륙 0.3 = 0.8 Wh  ->   1.6 m

    벤치마크 맵의 우회거리가 8~12 m 다. default 에서는 비행이 절대 이길 수
    없고 derived 에서는 거의 항상 이긴다.

    실측 전에 한쪽을 고르면 그 선택이 결론을 만든다. 그래서 고르지 않고
    두 세트를 나란히 돌려 "무엇이 파라미터에 의존하고 무엇이 안 하는지"를 본다.

비교 기준을 둘 다 쓰는 이유
    연구일지 1·2 는 RRT* '평균' 과 비교했다. 그런데 '최선값'(시드 5개 중
    제일 좋은 것)으로 비교하면 승패가 뒤집히는 맵이 있다.
    어느 한쪽만 쓰면 유리한 쪽을 고른 셈이 되므로 둘 다 보고한다.

실행: python3 benchmark/exp_param_sets.py
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

from cost.energy import EnergyModel  # noqa: E402
from envs.heightmap import build_all  # noqa: E402
from planners.state_space import ProblemSpec  # noqa: E402
from planners.grid_search import astar  # noqa: E402
from planners.rrt_star import rrt_star  # noqa: E402
from planners.smoothing import smooth_result  # noqa: E402

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
RESULTS.mkdir(parents=True, exist_ok=True)
CFG = HERE.parent / "src/drobot_hybrid_planner/config"

MAPS = ["easy_open", "easy_corridor", "medium_open", "medium_corridor", "hard_open"]
SETS = [("default", CFG / "energy_params.yaml"),
        ("derived", CFG / "energy_params_derived.yaml")]
N_SEEDS = 5
RRT_BUDGET = 2.0
SEP = "=" * 86


def run(hm, model):
    spec = ProblemSpec(hm=hm, model=model, dims=3)

    t0 = time.perf_counter()
    ra = astar(spec, timeout=120.0)
    a_time = time.perf_counter() - t0
    a_cost, a_acc, a_path = smooth_result(spec, ra, is_grid=True)

    runs = []
    for s in range(N_SEEDS):
        r = rrt_star(spec, seed=s, max_samples=10 ** 7, timeout=RRT_BUDGET)
        if not r.found:
            continue
        c, acc, pth = smooth_result(spec, r, is_grid=False)
        runs.append({"cost": c, "switches": acc.n_switches,
                     "energy_wh": acc.e_total, "path": pth})

    costs = [r["cost"] for r in runs]
    best = min(runs, key=lambda r: r["cost"]) if runs else None
    return {
        "astar": {"cost": a_cost, "switches": a_acc.n_switches,
                  "energy_wh": a_acc.e_total, "time_s": a_time,
                  "path": a_path},
        "rrt_best": best,
        "rrt_mean": float(np.mean(costs)) if costs else float("nan"),
        "rrt_std": float(np.std(costs)) if costs else float("nan"),
        "rrt_switch_mean": float(np.mean([r["switches"] for r in runs])) if runs else 0.0,
        "n_found": len(runs),
    }


def main():
    maps = build_all(res=0.1)
    out = {"maps": MAPS, "n_seeds": N_SEEDS, "rrt_budget_s": RRT_BUDGET, "sets": {}}

    for label, path in SETS:
        model = EnergyModel.from_yaml(path)
        sw_ref = model.takeoff_wh + model.landing_wh
        print(f"\n{SEP}\n파라미터 세트 {label}  —  전환 1회 {sw_ref} Wh "
              f"= 평지 {sw_ref / model.ground_wh_per_m:.1f} m\n{SEP}")
        print(f"  {'맵':<17}{'A*':>9}{'전환':>5}{'RRT*최선':>10}{'전환':>5}"
              f"{'RRT*평균':>10}{'σ':>7}{'vs최선':>9}{'vs평균':>9}")
        print("  " + "-" * 81)

        rows = {}
        for n in MAPS:
            d = run(maps[n], model)
            a = d["astar"]["cost"]
            rb = d["rrt_best"]["cost"] if d["rrt_best"] else float("nan")
            rm = d["rrt_mean"]
            g_best = 100 * (rb - a) / rb
            g_mean = 100 * (rm - a) / rm
            rows[n] = {**d, "gain_vs_best": g_best, "gain_vs_mean": g_mean}
            print(f"  {n:<17}{a:>9.3f}{d['astar']['switches']:>5}"
                  f"{rb:>10.3f}{d['rrt_best']['switches'] if d['rrt_best'] else 0:>5}"
                  f"{rm:>10.3f}{d['rrt_std']:>7.3f}{g_best:>+8.2f}%{g_mean:>+8.2f}%")

        w_best = sum(1 for r in rows.values() if r["gain_vs_best"] > 0)
        w_mean = sum(1 for r in rows.values() if r["gain_vs_mean"] > 0)
        n_fly = sum(1 for r in rows.values() if r["astar"]["switches"] > 0)
        print(f"  → A* 우위  최선기준 {w_best}/5 · 평균기준 {w_mean}/5"
              f"   |  A* 가 비행을 쓴 맵 {n_fly}/5")

        # 경로는 JSON 에 넣지 않는다 (용량) — 그림은 필요할 때 따로 뽑는다
        slim = {n: {k: v for k, v in r.items() if k != "astar"} | {
            "astar": {k: v for k, v in r["astar"].items() if k != "path"}}
            for n, r in rows.items()}
        for n in slim:
            if slim[n]["rrt_best"]:
                slim[n]["rrt_best"] = {k: v for k, v in slim[n]["rrt_best"].items()
                                       if k != "path"}
        out["sets"][label] = {
            "config": str(path.relative_to(HERE.parent)),
            "switch_wh": sw_ref,
            "wins_vs_best": w_best, "wins_vs_mean": w_mean,
            "maps_with_flight": n_fly,
            "rows": slim,
        }

    f = RESULTS / "benchmark_param_sets.json"
    json.dump(out, open(f, "w"), indent=1, ensure_ascii=False)
    print(f"\n저장 {f.relative_to(HERE.parent)}")

    # ---- 무엇이 파라미터에 의존하는가 ----
    print(f"\n{SEP}\n요약 — 파라미터에 의존하는 것과 아닌 것\n{SEP}")
    d, v = out["sets"]["default"], out["sets"]["derived"]
    print(f"  A* 가 비행을 쓴 맵 수      default {d['maps_with_flight']}/5"
          f"   derived {v['maps_with_flight']}/5     <- 크게 바뀜")
    print(f"  A* 우위 (평균 기준)        default {d['wins_vs_mean']}/5"
          f"   derived {v['wins_vs_mean']}/5")
    print(f"  A* 우위 (최선 기준)        default {d['wins_vs_best']}/5"
          f"   derived {v['wins_vs_best']}/5")


if __name__ == "__main__":
    main()
