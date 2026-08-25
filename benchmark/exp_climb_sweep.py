"""등반 에너지 계수 sweep — "밟고 넘기"가 언제 유리한가.

배경
    3 modal 비교에서 rover_climb(밟고 넘기)의 에너지는 아직 실측 전이다.
    INA226 측정이 지연되고 있어, 값을 하나로 고정하면 결론이 그 추정치에
    통째로 의존한다.

    그래서 계수를 미지수로 두고 넓게 훑는다.
    실측값이 나오면 이 곡선 위에 점 하나만 찍으면 결론이 정해진다.

    위치에너지 모델을 쓰지 않은 이유:
        로봇 2.723kg (URDF 링크 14개 합계), h=0.7m, 효율 0.4 로 계산하면
        0.013 Wh — 평지 주행 2.6cm 에 불과해 사실상 공짜다.
        그러면 rover_climb 이 항상 이겨서 비교 자체가 무의미해진다.
        실제 등반은 모터 토크 급증·슬립·저속 주행 때문에 훨씬 크다.

sweep 대상
    climb_mode.energy_per_height_m  (Wh/m) — 장애물 높이 1m 당 추가 에너지

    비교 대상 modal 은 이 계수와 무관하므로 (등반을 아예 안 하므로)
    맵당 한 번만 계획하고 재사용한다. 그래서 sweep 이 싸다.

실행: python3 benchmark/exp_climb_sweep.py
"""
from __future__ import annotations

import copy
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cost.energy import EnergyModel  # noqa: E402
from envs.heightmap import build_all  # noqa: E402
from planners.state_space import ProblemSpec  # noqa: E402
from planners.grid_search import astar  # noqa: E402
from planners.smoothing import smooth_result, path_cost  # noqa: E402
from modal_compare import enforce_superset, pick_winner, KR  # noqa: E402

RESULTS = Path(__file__).resolve().parent / "results"
RESULTS.mkdir(parents=True, exist_ok=True)

SEP = "=" * 90
MAPS = ["easy_open", "easy_corridor", "medium_open", "medium_corridor", "hard_open"]

# 0 부터 25 Wh/m 까지. 상한 근거:
#   0.55m 벽 하나를 넘는 추가 에너지가 25*0.55 = 13.75 Wh 로,
#   같은 벽을 비행으로 넘는 비용(이착륙 13.4 Wh)을 이미 넘어선다.
#   즉 이보다 크면 등반은 어떤 경우에도 선택되지 않는다.
CLIMB_VALUES = [0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0,
                10.0, 12.0, 14.0, 16.0, 18.0, 20.0, 22.0, 25.0]


def plan(hm, model, modal: str):
    """한 modal 로 계획하고 스무딩까지 마친 결과. 경로(waypoint)도 함께 돌려준다.

    경로를 돌려주는 이유는 아래 evaluate_path() 와 짝을 이루기 위해서다.
    """
    spec = ProblemSpec(hm=hm, model=model, dims=3, modal=modal)
    r = astar(spec, timeout=180.0)
    if not r.found:
        return None
    cost, acc, path = smooth_result(spec, r, is_grid=True)
    return {"cost": cost, "energy_wh": acc.e_total, "time_s": acc.time_s,
            "switches": acc.n_switches, "dist_ground": acc.dist_ground,
            "dist_air": acc.dist_air, "path": path}


def evaluate_path(hm, model, modal: str, path):
    """이미 만들어진 경로를 다른 파라미터로 다시 평가한다 (계획 없이).

    왜 필요한가:
        등반계수를 바꾸면 A* 가 다른 경로를 찾고, 그리디 스무더를 거치면
        결과가 계수에 대해 단조롭지 않을 수 있다.
        실제로 계수 0.5 -> 1.0 에서 rover_climb 비용이 오히려 줄어드는
        구간이 있었다. 등반이 비싸졌는데 더 싸진다는 건 있을 수 없다.

        원인은 플래너/스무더가 그 계수에서 더 나쁜 경로를 골랐다는 것이다.
        모든 계수에서 얻은 경로를 한데 모아 각 계수마다 전부 재평가하고
        최소를 취하면 이 문제가 사라진다:
          - 후보 경로 집합이 계수와 무관하게 동일하고
          - 각 경로의 비용은 계수에 대해 단조증가하므로
          - 그 최소도 단조증가한다.

        이는 결과 조작이 아니다. 후보 경로는 전부 실제 실행 가능하며,
        최적 플래너라면 당연히 그중 가장 싼 것을 골랐을 경로들이다.
    """
    spec = ProblemSpec(hm=hm, model=model, dims=3, modal=modal)
    acc = path_cost(spec, path)
    if acc is None:
        return None
    return {"cost": model.cost(acc), "energy_wh": acc.e_total,
            "time_s": acc.time_s, "switches": acc.n_switches,
            "dist_ground": acc.dist_ground, "dist_air": acc.dist_air,
            "path": path}


def main() -> None:
    base = EnergyModel.from_yaml()
    maps = build_all(res=0.1)
    t0 = time.perf_counter()

    print(SEP)
    print("등반 에너지 계수 sweep — 밟고 넘기가 언제 유리한가")
    print(SEP)
    print(f"  고정 가중치  wE={base.w_energy} wS={base.w_switch} wT={base.w_time}")
    print(f"  등반 한계높이 {base.rover_climb_max_h} m,  등반 속도배수 {base.rover_climb_speed_factor}")
    print(f"  sweep       {len(CLIMB_VALUES)}개 값 x {len(MAPS)}개 맵")
    print()

    # --- 등반계수와 무관한 두 modal 은 맵당 1회만 계획 ---
    fixed: dict[str, dict] = {}
    for name in MAPS:
        fixed[name] = {md: plan(maps[name], base, md)
                       for md in ("rover_detour", "hybrid")}
    print(f"  기준 modal 계획 완료 ({time.perf_counter()-t0:.0f}초)")
    print()

    # --- 1단계: 계수마다 계획해서 후보 경로를 모은다 ---
    climb_paths: dict[str, list] = {name: [] for name in MAPS}
    for cv in CLIMB_VALUES:
        m = copy.deepcopy(base)
        m.rover_climb_wh_per_m = cv
        for name in MAPS:
            r = plan(maps[name], m, "rover_climb")
            if r is not None:
                climb_paths[name].append(r["path"])
        print(f"  계획 등반계수 {cv:>5.1f} Wh/m ({time.perf_counter()-t0:.0f}초)")

    # 우회 경로도 후보에 넣는다 — 밟고넘기 로봇은 밟지 않는 경로도 갈 수 있다
    for name in MAPS:
        rd = fixed[name]["rover_detour"]
        if rd is not None:
            climb_paths[name].append(rd["path"])
    print()
    print(f"  후보 경로 수집 완료: "
          + ", ".join(f"{n}={len(climb_paths[n])}" for n in MAPS))
    print()

    # --- 2단계: 각 계수에서 모든 후보 경로를 재평가하고 최소를 취한다 ---
    rows = []
    for cv in CLIMB_VALUES:
        m = copy.deepcopy(base)
        m.rover_climb_wh_per_m = cv
        for name in MAPS:
            evals = [evaluate_path(maps[name], m, "rover_climb", pth)
                     for pth in climb_paths[name]]
            evals = [e for e in evals if e is not None]
            climb = min(evals, key=lambda e: e["cost"]) if evals else None

            cand = {"rover_detour": fixed[name]["rover_detour"],
                    "hybrid": fixed[name]["hybrid"],
                    "rover_climb": climb}
            # 상위집합 관계 강제 (modal_compare.py 주석 참고)
            cand = enforce_superset(cand)
            winner, tied = pick_winner(cand)
            n_corr = sum(1 for v in cand.values() if v and "corrected_from" in v)
            rows.append({"climb_wh_per_m": cv, "map": name, "winner": winner,
                         "tied": tied, "n_corrected": n_corr,
                         **{f"{k}_cost": (v["cost"] if v else None)
                            for k, v in cand.items()},
                         **{f"{k}_energy_wh": (v["energy_wh"] if v else None)
                            for k, v in cand.items()}})
        print(f"  재평가 등반계수 {cv:>5.1f} Wh/m ({time.perf_counter()-t0:.0f}초)")

    # ------------------------------------------------------------ 결과 1
    print()
    print(SEP)
    print("결과 1 — 등반계수별 맵당 승자 modal")
    print(SEP)
    print(f"  {'등반계수':>9} | " + " ".join(f"{n[:13]:>14}" for n in MAPS))
    print("  " + "-" * (11 + 15 * len(MAPS)))
    for cv in CLIMB_VALUES:
        cells = []
        for name in MAPS:
            r = next(x for x in rows if x["climb_wh_per_m"] == cv and x["map"] == name)
            # 동률이면 승자 이름 대신 '동률'로 표시한다.
            # min() 이 dict 순서로 고른 걸 승리로 읽으면 오해가 생긴다.
            label = "동률" if r["tied"] else KR.get(r["winner"], "-")
            cells.append(f"{label:>14}")
        print(f"  {cv:>9.1f} | " + " ".join(cells))

    # ------------------------------------------------------------ 결과 2
    print()
    print(SEP)
    print("결과 2 — 밟고넘기가 우위를 잃는 손익분기 등반계수")
    print(SEP)
    print("  (이 값보다 실측치가 작으면 밟고넘기가 유리하다)")
    print()
    print(f"  {'맵':<17}{'손익분기':>10}   {'그 이후 승자':<12}")
    print("  " + "-" * 45)
    for name in MAPS:
        sub = [x for x in rows if x["map"] == name]
        be, after = None, None
        for x in sub:
            # 단독 우위를 잃는 지점 — 동률이 되는 것도 우위 상실이다
            if x["tied"] or x["winner"] != "rover_climb":
                be = x["climb_wh_per_m"]
                after = "동률(우회와 같아짐)" if x["tied"] else KR.get(x["winner"], "-")
                break
        if be is None:
            print(f"  {name:<17}{'>25':>10}   {'(끝까지 밟고넘기 단독 우위)':<12}")
        elif be == CLIMB_VALUES[0]:
            print(f"  {name:<17}{'0 이하':>10}   {after:<12}")
        else:
            print(f"  {name:<17}{be:>10.1f}   {after:<12}")

    # ------------------------------------------------------------ 결과 3
    print()
    print(SEP)
    print("결과 3 — 등반계수 2.0 (현재 추정치)에서 3 modal 상세")
    print(SEP)
    print(f"  {'맵':<17}{'우회':>9}{'밟고넘기':>11}{'하이브리드':>12}{'승자':>12}"
          f"{'하이브리드 에너지':>17}")
    print("  " + "-" * 78)
    for name in MAPS:
        r = next(x for x in rows if x["climb_wh_per_m"] == 2.0 and x["map"] == name)
        f = lambda v: f"{v:9.3f}" if v is not None else f"{'해없음':>9}"
        print(f"  {name:<17}{f(r['rover_detour_cost'])}{f(r['rover_climb_cost']):>11}"
              f"{f(r['hybrid_cost']):>12}"
              f"{('동률' if r['tied'] else KR.get(r['winner'],'-')):>12}"
              f"{r['hybrid_energy_wh'] or 0:>14.1f} Wh")

    out = RESULTS / "benchmark_climb_sweep.json"
    with open(out, "w") as f:
        json.dump({"climb_values": CLIMB_VALUES, "maps": MAPS,
                   "weights": {"wE": base.w_energy, "wS": base.w_switch,
                               "wT": base.w_time},
                   "climb_max_height": base.rover_climb_max_h,
                   "rows": rows}, f, indent=1, ensure_ascii=False)
    print()
    print(f"저장: {out.name}  ({len(rows)}행, {time.perf_counter()-t0:.0f}초)")


if __name__ == "__main__":
    main()
