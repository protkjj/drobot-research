"""통합 벤치마크 러너 — 모든 실험을 돌리고 결과를 JSON으로 저장한다.

지금까지의 실험 스크립트는 결과를 stdout으로만 냈다.
그러면 재분석도 시각화도 안 되고, 숫자를 인용할 때마다 다시 돌려야 한다.
여기서 한 번 돌려 results/*.json 에 남기고, 분석/시각화는 그 파일을 읽는다.

실행
    python3 benchmark/run_benchmark.py --quick     # 축소 실행 (동작 확인용)
    python3 benchmark/run_benchmark.py             # 전체 실행

저장 형식 (results/benchmark_<실험명>.json)
    {"meta": {...}, "rows": [{...}, ...]}
    rows 는 그대로 pandas.DataFrame 으로 읽힌다.
"""
from __future__ import annotations

import argparse
import json
import platform
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cost.energy import EnergyModel  # noqa: E402
from envs.heightmap import build_all  # noqa: E402
from planners.state_space import ProblemSpec  # noqa: E402
from planners.grid_search import astar, dijkstra  # noqa: E402
from planners.rrt_star import rrt_star  # noqa: E402
from planners.smoothing import smooth_result  # noqa: E402

RESULTS = Path(__file__).resolve().parent / "results"
RESULTS.mkdir(parents=True, exist_ok=True)


def _save(name: str, rows: list[dict], meta: dict) -> Path:
    p = RESULTS / f"benchmark_{name}.json"
    with open(p, "w") as f:
        json.dump({"meta": meta, "rows": rows}, f, indent=1, ensure_ascii=False)
    print(f"    -> 저장 {p.name} ({len(rows)}행)")
    return p


def _env_meta() -> dict:
    return {
        "python": platform.python_version(),
        "platform": platform.platform(),
        "machine": platform.machine(),
    }


def _acc_dict(acc) -> dict:
    if acc is None:
        return {}
    return {
        "e_total_wh": round(acc.e_total, 6),
        "e_ground_wh": round(acc.e_ground, 6),
        "e_air_wh": round(acc.e_air, 6),
        "e_switch_wh": round(acc.e_switch, 6),
        "time_s": round(acc.time_s, 6),
        "dist_ground_m": round(acc.dist_ground, 6),
        "dist_air_m": round(acc.dist_air, 6),
        "n_switches": acc.n_switches,
    }


# ---------------------------------------------------------------------------
# 실험 1: 시간 예산별 비교 (3D)
# ---------------------------------------------------------------------------
def exp_time_budget(model, maps, quick: bool) -> None:
    print("[1/4] 시간 예산 비교 (3D)")
    budgets = [0.25, 1.0, 2.0] if quick else [0.1, 0.25, 0.5, 1.0, 2.0, 5.0]
    n_seeds = 5 if quick else 20
    names = list(maps)

    rows = []
    for name in names:
        spec = ProblemSpec(hm=maps[name], model=model, dims=3)

        # A* — 결정론적이므로 1회
        t0 = time.perf_counter()
        ra = astar(spec, timeout=120.0)
        ta_raw = time.perf_counter() - t0
        if ra.found:
            ca, acc_a, _ = smooth_result(spec, ra, is_grid=True)
            ta = time.perf_counter() - t0
        else:
            ca, acc_a, ta = None, None, ta_raw

        rows.append({
            "map": name, "dims": 3, "planner": "astar", "budget_s": None,
            "seed": None, "found": ra.found, "cost": ca,
            "runtime_s": round(ta, 6), "n_expanded": ra.n_expanded,
            **_acc_dict(acc_a),
        })

        # RRT* — 예산 x 시드
        for b in budgets:
            for s in range(n_seeds):
                r = rrt_star(spec, seed=s, max_samples=10 ** 7, timeout=b)
                if r.found:
                    c, acc_r, _ = smooth_result(spec, r, is_grid=False)
                else:
                    c, acc_r = None, None
                rows.append({
                    "map": name, "dims": 3, "planner": "rrt_star",
                    "budget_s": b, "seed": s, "found": r.found, "cost": c,
                    "runtime_s": round(r.runtime_s, 6),
                    "n_expanded": r.n_expanded, **_acc_dict(acc_r),
                })
        print(f"    {name} 완료")

    _save("time_budget", rows, {
        "experiment": "동일 시간 예산에서 A* vs RRT* (3D)",
        "budgets_s": budgets, "n_seeds": n_seeds, "quick": quick,
        **_env_meta(),
    })


# ---------------------------------------------------------------------------
# 실험 2: 4D z해상도 sweep
# ---------------------------------------------------------------------------
def exp_4d(model, maps, quick: bool) -> None:
    print("[2/4] 4D z해상도 sweep")
    z_list = [0.5, 0.25] if quick else [0.5, 0.25, 0.1, 0.05]
    n_seeds = 3 if quick else 5
    a_timeout = 30.0 if quick else 120.0
    names = ["easy_open", "medium_open", "hard_open", "hard_corridor"]

    rows = []
    for name in names:
        for zr in z_list:
            spec = ProblemSpec(hm=maps[name], model=model, dims=4, z_res=zr)
            n_states = spec.summary()["n_states"]

            t0 = time.perf_counter()
            ra = astar(spec, timeout=a_timeout)
            if ra.found:
                ca, acc_a, _ = smooth_result(spec, ra, is_grid=True)
            else:
                ca, acc_a = None, None
            ta = time.perf_counter() - t0

            rows.append({
                "map": name, "dims": 4, "z_res": zr, "n_states": n_states,
                "planner": "astar", "seed": None, "found": ra.found,
                "timed_out": ra.timed_out, "cost": ca,
                "runtime_s": round(ta, 6), "n_expanded": ra.n_expanded,
                **_acc_dict(acc_a),
            })

            for s in range(n_seeds):
                r = rrt_star(spec, seed=s, max_samples=10 ** 7, timeout=2.0)
                if r.found:
                    c, acc_r, _ = smooth_result(spec, r, is_grid=False)
                else:
                    c, acc_r = None, None
                rows.append({
                    "map": name, "dims": 4, "z_res": zr, "n_states": n_states,
                    "planner": "rrt_star", "seed": s, "found": r.found,
                    "timed_out": False, "cost": c,
                    "runtime_s": round(r.runtime_s, 6),
                    "n_expanded": r.n_expanded, **_acc_dict(acc_r),
                })
            print(f"    {name} z={zr} 완료")

    _save("z_resolution", rows, {
        "experiment": "4D 상태공간에서 z해상도별 A* vs RRT*",
        "z_resolutions": z_list, "n_seeds": n_seeds,
        "astar_timeout_s": a_timeout, "rrt_budget_s": 2.0, "quick": quick,
        **_env_meta(),
    })


# ---------------------------------------------------------------------------
# 실험 3: 이착륙 에너지 파라미터 sweep
# ---------------------------------------------------------------------------
def exp_param_sweep(model, maps, quick: bool) -> None:
    print("[3/4] 이착륙 에너지 파라미터 sweep")
    import copy
    takeoffs = [0.3, 1.5, 5.0] if quick else [0.1, 0.3, 0.7, 1.5, 3.0, 5.0]
    n_seeds = 3 if quick else 10
    names = ["medium_open", "medium_corridor", "hard_open"]

    rows = []
    for tk in takeoffs:
        m2 = copy.deepcopy(model)
        m2.takeoff_wh = tk
        m2.landing_wh = tk * 0.6      # 원본 5:3 비율 유지
        for name in names:
            spec = ProblemSpec(hm=maps[name], model=m2, dims=3)

            t0 = time.perf_counter()
            ra = astar(spec, timeout=120.0)
            if ra.found:
                ca, acc_a, _ = smooth_result(spec, ra, is_grid=True)
            else:
                ca, acc_a = None, None
            ta = time.perf_counter() - t0

            rows.append({
                "map": name, "takeoff_wh": tk, "landing_wh": round(tk * 0.6, 3),
                "planner": "astar", "seed": None, "found": ra.found,
                "cost": ca, "runtime_s": round(ta, 6), **_acc_dict(acc_a),
            })

            for s in range(n_seeds):
                r = rrt_star(spec, seed=s, max_samples=10 ** 7, timeout=2.0)
                if r.found:
                    c, acc_r, _ = smooth_result(spec, r, is_grid=False)
                else:
                    c, acc_r = None, None
                rows.append({
                    "map": name, "takeoff_wh": tk,
                    "landing_wh": round(tk * 0.6, 3), "planner": "rrt_star",
                    "seed": s, "found": r.found, "cost": c,
                    "runtime_s": round(r.runtime_s, 6), **_acc_dict(acc_r),
                })
        print(f"    takeoff={tk} 완료")

    _save("param_sweep", rows, {
        "experiment": "이착륙 에너지에 따른 비행 선택 빈도와 플래너 우열",
        "takeoff_wh_values": takeoffs, "landing_ratio": 0.6,
        "n_seeds": n_seeds, "rrt_budget_s": 2.0, "quick": quick,
        **_env_meta(),
    })


# ---------------------------------------------------------------------------
# 실험 4: narrow passage
# ---------------------------------------------------------------------------
def exp_narrow(model, quick: bool) -> None:
    print("[4/4] narrow passage")
    from exp_narrow_passage import make_narrow_map

    gaps = [2.0, 1.0, 0.6] if quick else [3.0, 2.0, 1.5, 1.0, 0.8, 0.6, 0.4]
    n_seeds = 5 if quick else 20

    rows = []
    for gap in gaps:
        hm = make_narrow_map(gap)
        spec = ProblemSpec(hm=hm, model=model, dims=3)

        t0 = time.perf_counter()
        ra = astar(spec, timeout=60.0)
        if ra.found:
            ca, acc_a, _ = smooth_result(spec, ra, is_grid=True)
        else:
            ca, acc_a = None, None
        ta = time.perf_counter() - t0

        rows.append({
            "gap_m": gap, "planner": "astar", "seed": None,
            "found": ra.found, "cost": ca, "runtime_s": round(ta, 6),
            **_acc_dict(acc_a),
        })

        for s in range(n_seeds):
            r = rrt_star(spec, seed=s, max_samples=10 ** 7, timeout=2.0)
            if r.found:
                c, acc_r, _ = smooth_result(spec, r, is_grid=False)
            else:
                c, acc_r = None, None
            rows.append({
                "gap_m": gap, "planner": "rrt_star", "seed": s,
                "found": r.found, "cost": c,
                "runtime_s": round(r.runtime_s, 6), **_acc_dict(acc_r),
            })
        print(f"    통로 {gap}m 완료")

    _save("narrow_passage", rows, {
        "experiment": "통로 폭에 따른 성공률 — 샘플링 기반의 narrow passage 약점",
        "gaps_m": gaps, "n_seeds": n_seeds, "rrt_budget_s": 2.0,
        "quick": quick, **_env_meta(),
    })


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true",
                    help="축소 실행 (동작 확인용)")
    ap.add_argument("--only", type=str, default=None,
                    help="특정 실험만: time/z/param/narrow")
    args = ap.parse_args()

    model = EnergyModel.from_yaml()
    maps = build_all(res=0.1)

    print("=" * 70)
    print(f"벤치마크 실행 {'(quick)' if args.quick else '(full)'}")
    print("=" * 70)
    t0 = time.perf_counter()

    only = args.only
    if only in (None, "time"):
        exp_time_budget(model, maps, args.quick)
    if only in (None, "z"):
        exp_4d(model, maps, args.quick)
    if only in (None, "param"):
        exp_param_sweep(model, maps, args.quick)
    if only in (None, "narrow"):
        exp_narrow(model, args.quick)

    print()
    print(f"전체 완료: {time.perf_counter() - t0:.1f}초")
    print(f"결과: {RESULTS}")


if __name__ == "__main__":
    main()
