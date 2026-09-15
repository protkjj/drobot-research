"""가중치 sweep — 3 modal 비교로 에너지 효율 경로를 분석한다.

연구 목적 (2026-08-24 회의):
    카메라 등 비주얼 센서를 쓰지 않고, 맵을 전부 안다는 가정 하에
    3개 modal 각각에 대해 1개 traj 를 생성하고,
    동일한 에너지 모델로 각 traj 를 평가해 에너지 효율 경로를 살핀다.

    이는 Contribution 2.2 (Energy Cost Evaluator)에 해당한다 —
    특정 플래너에 종속되지 않고, 생성된 경로를 후처리로 평가하는 모듈.

Modal — 장애물을 만났을 때 '어떻게 넘어가느냐'로 나뉜다
    rover_detour  우회      장애물을 피해 돌아간다
    rover_climb   밟고넘기   0.7m 이하 장애물은 타고 넘는다
    hybrid        하이브리드  드론으로 전환해 날아 넘는다

sweep 대상
    wE + wS + wT = 1 을 만족하는 격자 (0.1 단위)
    각 조합에서 3 modal 의 경로를 생성하고 비용을 비교한다.

주의: 세 modal 의 '경로 자체'는 가중치에 따라 달라진다(플래너가 그 비용을
      최소화하므로). 따라서 각 조합마다 3번 계획한다.

실행: python3 benchmark/exp_weight_sweep.py
"""
from __future__ import annotations

import copy
import itertools
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
from planners.smoothing import smooth_result, path_cost  # noqa: E402
from modal_compare import (  # noqa: E402
    enforce_superset, pick_winner, MODALS, KR, SUPERSET_OF)

RESULTS = Path(__file__).resolve().parent / "results"
RESULTS.mkdir(parents=True, exist_ok=True)

SEP = "=" * 92
MAPS = ["easy_open", "easy_corridor", "medium_open", "medium_corridor", "hard_open"]
STEP = 0.1


def weight_grid(step: float = STEP):
    """wE + wS + wT = 1 을 만족하는 격자.

    각 가중치가 0 인 경우도 포함한다 (해당 항을 완전히 무시하는 극단).
    """
    n = int(round(1.0 / step))
    out = []
    for i in range(n + 1):
        for j in range(n + 1 - i):
            k = n - i - j
            out.append((round(i * step, 3), round(j * step, 3), round(k * step, 3)))
    return out


def plan_one(hm, model, modal: str) -> dict | None:
    """한 modal 로 계획 + 스무딩. 경로도 함께 돌려준다."""
    spec = ProblemSpec(hm=hm, model=model, dims=3, modal=modal)
    t0 = time.perf_counter()
    r = astar(spec, timeout=60.0)
    dt = time.perf_counter() - t0
    if not r.found:
        return None
    cost, acc, path = smooth_result(spec, r, is_grid=True)
    return {
        "cost": cost,
        "energy_wh": acc.e_total,
        "time_s": acc.time_s,
        "switches": acc.n_switches,
        "dist_ground": acc.dist_ground,
        "dist_air": acc.dist_air,
        "plan_time_s": dt,
        "path": path,
    }


def evaluate_path(hm, model, modal: str, path) -> dict | None:
    """이미 만들어진 경로를 다른 가중치로 다시 평가한다 (계획 없이).

    왜 필요한가:
        가중치를 바꾸면 A* 가 다른 경로를 찾고, 그리디 스무더를 거치면
        결과가 뒤집힐 수 있다. 등반계수 sweep 에서 실제로
        '상위집합 modal 이 하위집합보다 비싸다'는 불가능한 결과가 나왔고,
        원인을 분리해보니 A* 격자해는 정상이고 스무딩에서만 역전됐다
        (우회 경로는 5.0~5.8% 개선, 밟고넘기 경로는 0.4~2.4% 개선).

        모든 가중치 조합에서 얻은 경로를 한데 모아 각 조합마다 전부
        재평가하고 최소를 취하면, 후보 집합이 조합과 무관하게 같아져서
        이런 아티팩트가 사라진다.

        결과 조작이 아니다 — 후보 경로는 전부 실제 실행 가능하며,
        최적 플래너라면 그중 가장 싼 것을 골랐을 경로들이다.

    해당 modal 로 실행 불가능한 경로면 path_cost 가 None 을 내므로
    자동으로 걸러진다 (예: 비행 구간이 있는 경로를 rover_climb 으로 평가).
    """
    spec = ProblemSpec(hm=hm, model=model, dims=3, modal=modal)
    acc = path_cost(spec, path)
    if acc is None:
        return None
    return {
        "cost": model.cost(acc),
        "energy_wh": acc.e_total,
        "time_s": acc.time_s,
        "switches": acc.n_switches,
        "dist_ground": acc.dist_ground,
        "dist_air": acc.dist_air,
        "plan_time_s": 0.0,
        "path": path,
    }


def path_key(path) -> tuple:
    """경로의 중복 판정용 키. 좌표를 소수 4자리로 반올림해 비교한다.

    가중치가 달라도 같은 경로가 나오는 경우가 많아, 중복을 지우지 않으면
    재평가 횟수가 13만 회까지 늘어난다.
    """
    return tuple((round(x, 4), round(y, 4), round(z, 4), m) for x, y, z, m in path)


def dedup_paths(paths: list) -> list:
    seen = set()
    out = []
    for pth in paths:
        k = path_key(pth)
        if k in seen:
            continue
        seen.add(k)
        out.append(pth)
    return out


def candidate_paths(pool: dict[str, list], modal: str) -> list:
    """이 modal 이 실행할 수 있는 후보 경로들.

    자기 modal 에서 나온 경로 + 하위집합 modal 에서 나온 경로.
    (밟고 넘을 수 있는 로봇은 밟지 않는 경로도 그대로 갈 수 있다)
    """
    out = list(pool.get(modal, []))
    for sub in SUPERSET_OF.get(modal, []):
        out.extend(pool.get(sub, []))
    return out


def main() -> None:
    base = EnergyModel.from_yaml()
    maps = build_all(res=0.1)
    grid = weight_grid()

    print(SEP)
    print("가중치 sweep — 3 modal 에너지 효율 비교")
    print(SEP)
    print(f"  가중치 조합 {len(grid)}개 (wE+wS+wT=1, {STEP} 단위)")
    print(f"  맵 {len(MAPS)}개 x modal {len(MODALS)}개")
    print(f"  총 계획 횟수 {len(grid) * len(MAPS) * len(MODALS):,}")
    print()

    t_start = time.perf_counter()

    # --- 1단계: 조합마다 계획해서 후보 경로를 모은다 ---
    # pool[맵][modal] = 그 modal 에서 나온 경로 목록
    pool: dict[str, dict[str, list]] = {
        name: {md: [] for md in MODALS} for name in MAPS
    }
    for gi, (wE, wS, wT) in enumerate(grid):
        m = copy.deepcopy(base)
        m.w_energy, m.w_switch, m.w_time = wE, wS, wT
        for name in MAPS:
            for modal in MODALS:
                r = plan_one(maps[name], m, modal)
                if r is not None:
                    pool[name][modal].append(r["path"])
        if (gi + 1) % 10 == 0 or gi == len(grid) - 1:
            el = time.perf_counter() - t_start
            print(f"  계획 {gi+1}/{len(grid)} 조합  ({el:.0f}초 경과)")

    print()
    print("  후보 경로 중복 제거 (같은 경로가 여러 가중치에서 반복해 나온다)")
    for name in MAPS:
        before = {md: len(pool[name][md]) for md in MODALS}
        for md in MODALS:
            pool[name][md] = dedup_paths(pool[name][md])
        print(f"  {name:<17} "
              + ", ".join(f"{KR[md]} {before[md]}->{len(pool[name][md])}"
                          for md in MODALS))
    print()

    # --- 2단계: 각 조합에서 모든 후보 경로를 재평가하고 최소를 취한다 ---
    rows = []
    for gi, (wE, wS, wT) in enumerate(grid):
        m = copy.deepcopy(base)
        m.w_energy, m.w_switch, m.w_time = wE, wS, wT

        for name in MAPS:
            per_modal = {}
            for modal in MODALS:
                evals = [evaluate_path(maps[name], m, modal, pth)
                         for pth in candidate_paths(pool[name], modal)]
                evals = [e for e in evals if e is not None]
                per_modal[modal] = min(evals, key=lambda e: e["cost"]) if evals else None

            # 안전망 — 경로 풀링으로 대부분 해소되지만 한 번 더 확인한다
            per_modal = enforce_superset(per_modal)
            winner, tied = pick_winner(per_modal)
            n_corr = sum(1 for v in per_modal.values() if v and "corrected_from" in v)

            row = {"wE": wE, "wS": wS, "wT": wT, "map": name,
                   "winner": winner, "tied": tied, "n_corrected": n_corr}
            for modal, res in per_modal.items():
                if res is None:
                    row[f"{modal}_cost"] = None
                    continue
                row[f"{modal}_cost"] = res["cost"]
                row[f"{modal}_energy_wh"] = res["energy_wh"]
                row[f"{modal}_time_s"] = res["time_s"]
                row[f"{modal}_switches"] = res["switches"]
            rows.append(row)

        if (gi + 1) % 10 == 0 or gi == len(grid) - 1:
            el = time.perf_counter() - t_start
            print(f"  재평가 {gi+1}/{len(grid)} 조합  ({el:.0f}초 경과)")

    # ---------------------------------------------------------------- 요약
    print()
    print(SEP)
    print("결과 1 — 가중치 조합별 승자 modal 분포")
    print(SEP)
    from collections import Counter
    # 동률은 따로 센다 — min() 이 dict 순서로 고르는 걸 승리로 집계하면 안 된다
    cnt = Counter(r["winner"] for r in rows if r["winner"] and not r["tied"])
    n_tied = sum(1 for r in rows if r["tied"])
    tot = len(rows)
    for modal in MODALS:
        c = cnt.get(modal, 0)
        print(f"  {KR[modal]:<12} {c:>4} / {tot}  ({100*c/tot:5.1f}%)")
    print(f"  {'동률':<12} {n_tied:>4} / {tot}  ({100*n_tied/tot:5.1f}%)")
    n_corr = sum(r["n_corrected"] for r in rows)
    print()
    print(f"  상위집합 보정이 적용된 건수: {n_corr} "
          f"(스무딩이 그리디라 상위집합이 더 비싸게 나온 경우)")

    print()
    print(SEP)
    print("결과 1b — 에너지 가중치 wE 에 따른 승자 분포")
    print(SEP)
    print(f"  {'wE':>4} {'하이브리드':>11} {'밟고넘기':>10} {'우회':>8} {'동률':>7}")
    print("  " + "-" * 46)
    for we in sorted({r["wE"] for r in rows}):
        sub = [r for r in rows if r["wE"] == we]
        c = Counter(r["winner"] for r in sub if r["winner"] and not r["tied"])
        t = sum(1 for r in sub if r["tied"])
        print(f"  {we:>4.1f} {c.get('hybrid',0):>11} {c.get('rover_climb',0):>10} "
              f"{c.get('rover_detour',0):>8} {t:>7}")

    print()
    print(SEP)
    print("결과 2 — wS(모드 전환 억제)에 따른 hybrid 선택률")
    print(SEP)
    print(f"  {'wS':>5} {'하이브리드':>11} {'밟고넘기':>10} {'우회':>9} {'하이브리드 전환수':>18}")
    print("  " + "-" * 58)
    for ws in sorted({r["wS"] for r in rows}):
        sub = [r for r in rows if r["wS"] == ws and r["winner"]]
        if not sub:
            continue
        c = Counter(r["winner"] for r in sub)
        sw = [r.get("hybrid_switches") for r in sub if r.get("hybrid_switches") is not None]
        print(f"  {ws:>5.1f} {c.get('hybrid',0):>11} {c.get('rover_climb',0):>10} "
              f"{c.get('rover_detour',0):>9} "
              f"{np.mean(sw) if sw else 0:>17.2f}")

    print()
    print(SEP)
    print("결과 3 — 현재 설정(0.5/0.2/0.3)에서 맵별 3 modal 비교")
    print(SEP)
    cur = [r for r in rows if (r["wE"], r["wS"], r["wT"]) == (0.5, 0.2, 0.3)]
    if cur:
        print(f"  {'맵':<17} {'우회':>9} {'밟고넘기':>10} {'하이브리드':>11} {'승자':>12} "
              f"{'승자 에너지':>13}")
        print("  " + "-" * 78)
        for r in cur:
            rv = r.get("rover_detour_cost")
            cl = r.get("rover_climb_cost")
            hy = r.get("hybrid_cost")
            f = lambda v: f"{v:9.3f}" if v is not None else f"{'해없음':>9}"
            we = r.get(f"{r['winner']}_energy_wh") if r["winner"] else None
            print(f"  {r['map']:<17} {f(rv)} {f(cl):>10} {f(hy):>11} "
                  f"{KR.get(r['winner'],'-'):>12} {we or 0:>10.1f} Wh")

    out = RESULTS / "benchmark_weight_sweep.json"
    with open(out, "w") as f:
        json.dump({"grid_step": STEP, "modals": MODALS, "maps": MAPS,
                   "rows": rows}, f, indent=1, ensure_ascii=False)
    print()
    print(f"저장: {out.name}  ({len(rows)}행, {time.perf_counter()-t_start:.0f}초)")


if __name__ == "__main__":
    main()
