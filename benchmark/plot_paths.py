"""A* vs RRT* 경로 비교 그림.

연구일지 1(RESEARCH_LOG.md)의 결론을 눈으로 보여주는 그림이다.
숫자표만으로는 "왜 A*인가"가 잘 전달되지 않는다. 두 가지를 동시에 보여준다.

    1. 같은 맵에서 두 플래너가 실제로 어떤 경로를 내는가
    2. A*는 매번 같은 경로, RRT*는 시드마다 다른 경로       <- 결정론 대 확률

RRT* 는 시드 5개를 전부 그린다. 흐린 선이 개별 시드, 진한 선이 최선값이다.
이 퍼짐 자체가 RESEARCH_LOG.md 가 보고한 표준편차(0.25~0.73)의 그림판이다.

색 선택 (눈으로 고르지 않고 계산함)
    지형 배경이 이미 4색(#e8e8e6 #a8cfe0 #e8933a #2b2b2b)을 쓰므로
    경로색은 그것과도, 서로와도 떨어져야 한다.
    보라 #4a3aa7 / 빨강 #e34948 을 골랐다 (OKLab ΔE x100):
        정상시야 서로       33.6   (하한 15)
        색각이상 최악       23.4   (목표 8, protan)
        지형색과 최소       16.3
    지상/공중 구분은 색이 아니라 선 모양으로 준다 (실선/점선).

실행: python3 benchmark/plot_paths.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import plotstyle  # noqa: E402,F401  (import 만으로 한글 폰트가 설정된다)

import numpy as np  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import BoundaryNorm, ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

from cost.energy import EnergyModel  # noqa: E402
from envs.heightmap import build_all, ROVER_MAX, H_3D_LIMIT, H_4D_LIMIT  # noqa: E402
from planners.state_space import ProblemSpec, AIR  # noqa: E402
from planners.grid_search import astar  # noqa: E402
from planners.rrt_star import rrt_star  # noqa: E402
from planners.smoothing import smooth_result  # noqa: E402

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
RESULTS.mkdir(parents=True, exist_ok=True)

MAPS = ["easy_open", "easy_corridor", "medium_open", "medium_corridor", "hard_open"]
N_SEEDS = 5
RRT_BUDGET = 2.0          # 초 — nav2 글로벌 플래너 응답시간 제약과 같은 값

C_ASTAR, C_RRT = "#4a3aa7", "#e34948"
INK, INK2, INK3 = "#0b0b0b", "#52514e", "#8a8a85"
SURFACE = "#fcfcfb"

# 지형 4단계 — plot_maps.py 와 같은 분류를 쓴다
TERRAIN_CMAP = ListedColormap(["#e8e8e6", "#a8cfe0", "#e8933a", "#2b2b2b"])
TERRAIN_NORM = BoundaryNorm([-1e-9, ROVER_MAX, H_3D_LIMIT, H_4D_LIMIT, 99.0],
                            TERRAIN_CMAP.N)


def draw_terrain(ax, hm):
    ax.imshow(hm.grid, origin="lower", cmap=TERRAIN_CMAP, norm=TERRAIN_NORM,
              extent=[0, hm.width_m, 0, hm.height_m], interpolation="nearest")
    ax.set_aspect("equal")          # 기하가 왜곡되면 경로 비교가 거짓말이 된다
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#d8d8d4")


def draw_path(ax, path, color, lw=2.0, alpha=1.0, zorder=5, marks=False):
    """경로를 지상/공중 구간으로 끊어 그린다.

    색이 아니라 선 모양으로 모드를 구분한다 — 색은 이미 플래너 식별에 쓰였다.
        실선 = 지상 주행
        점선 = 비행
    """
    if not path:
        return
    segs, cur = [], [path[0]]
    for p in path[1:]:
        if p[3] != cur[-1][3]:
            segs.append(cur)
            cur = [p]
        else:
            cur.append(p)
    segs.append(cur)

    for seg in segs:
        if len(seg) < 2:
            continue
        xs = [p[0] for p in seg]
        ys = [p[1] for p in seg]
        flying = seg[0][3] == AIR
        ax.plot(xs, ys, color=color, linewidth=lw, alpha=alpha, zorder=zorder,
                linestyle=(0, (2.4, 1.8)) if flying else "-",
                solid_capstyle="round")

    if marks:
        # 이/착륙 지점 — 모드가 바뀌는 곳
        for a, b in zip(path, path[1:]):
            if a[3] != b[3]:
                ax.plot(a[0], a[1], marker="o", markersize=5.5, color=color,
                        markeredgecolor="white", markeredgewidth=1.3,
                        zorder=zorder + 1)


def run_one(hm, model):
    """한 맵에서 A* 1회 + RRT* N_SEEDS 회를 돌린다."""
    spec = ProblemSpec(hm=hm, model=model, dims=3)

    t0 = time.perf_counter()
    ra = astar(spec, timeout=120.0)
    a_time = time.perf_counter() - t0
    a_cost, a_acc, a_path = smooth_result(spec, ra, is_grid=True)

    runs = []
    for s in range(N_SEEDS):
        r = rrt_star(spec, seed=s, max_samples=10 ** 7, timeout=RRT_BUDGET)
        if not r.found:
            runs.append(None)
            continue
        c, acc, pth = smooth_result(spec, r, is_grid=False)
        runs.append({"cost": c, "acc": acc, "path": pth})

    ok = [r for r in runs if r]
    best = min(ok, key=lambda r: r["cost"]) if ok else None
    costs = [r["cost"] for r in ok]
    return {
        "spec": spec,
        "astar": {"cost": a_cost, "acc": a_acc, "path": a_path, "time": a_time},
        "rrt_runs": runs,
        "rrt_best": best,
        "rrt_mean": float(np.mean(costs)) if costs else float("nan"),
        "rrt_std": float(np.std(costs)) if costs else float("nan"),
        "n_found": len(ok),
    }


def main():
    model = EnergyModel.from_yaml()
    maps = build_all(res=0.1)

    print(f"A* 1회 + RRT* {N_SEEDS}시드 x {len(MAPS)}맵 "
          f"(RRT* 예산 {RRT_BUDGET}초) — 약 {len(MAPS)*N_SEEDS*RRT_BUDGET:.0f}초")
    data = {}
    for name in MAPS:
        data[name] = run_one(maps[name], model)
        d = data[name]
        rb = d['rrt_best']
        win = "A*" if d['astar']['cost'] <= (rb['cost'] if rb else 9e9) else "RRT*"
        print(f"  {name:<17} A* {d['astar']['cost']:7.3f} (전환 {d['astar']['acc'].n_switches})"
              f" | RRT* {rb['cost'] if rb else float('nan'):7.3f} "
              f"(전환 {rb['acc'].n_switches if rb else '-'})"
              f"  sigma {d['rrt_std']:.3f}   승자 {win}")

    # ------------------------------------------------------------ 그림
    fig = plt.figure(figsize=(15.2, 8.2))
    gs = fig.add_gridspec(2, 3, hspace=0.26, wspace=0.12)

    for i, name in enumerate(MAPS):
        ax = fig.add_subplot(gs[i // 3, i % 3])
        d = data[name]
        hm = maps[name]
        draw_terrain(ax, hm)

        # RRT* 시드 전부 — 흐리게. 이 퍼짐이 곧 비결정성이다
        for r in d["rrt_runs"]:
            if r:
                draw_path(ax, r["path"], C_RRT, lw=1.1, alpha=0.32, zorder=4)
        if d["rrt_best"]:
            draw_path(ax, d["rrt_best"]["path"], C_RRT, lw=2.0, zorder=6, marks=True)
        draw_path(ax, d["astar"]["path"], C_ASTAR, lw=2.4, zorder=7, marks=True)

        ax.plot(*hm.start, marker="s", markersize=7, color="white",
                markeredgecolor=INK, markeredgewidth=1.4, zorder=9)
        ax.plot(*hm.goal, marker="*", markersize=13, color="white",
                markeredgecolor=INK, markeredgewidth=1.2, zorder=9)

        ac = d["astar"]["cost"]
        rb = d["rrt_best"]["cost"] if d["rrt_best"] else float("nan")
        rm = d["rrt_mean"]
        # 연구일지 1·2 는 RRT* '평균' 과 비교했다. 기준에 따라 승패가 갈리므로 둘 다 쓴다.
        g_best = 100 * (rb - ac) / rb
        g_mean = 100 * (rm - ac) / rm
        ax.set_title(name, loc="left", fontsize=11.5, color=INK,
                     fontweight="bold", pad=7)
        ax.text(0, -0.058,
                f"A* {ac:.2f}   ·   RRT* 최선 {rb:.2f} ({g_best:+.1f}%)"
                f"   평균 {rm:.2f} ± {d['rrt_std']:.2f} ({g_mean:+.1f}%)",
                transform=ax.transAxes, fontsize=9.2, color=INK2)

    # ---- 여섯 번째 칸: 범례와 읽는 법 ----
    ax = fig.add_subplot(gs[1, 2])
    ax.set_facecolor(SURFACE)
    ax.axis("off")
    n_sw = sum(d["astar"]["acc"].n_switches for d in data.values())
    handles = [
        Line2D([], [], color=C_ASTAR, lw=2.4, label="A*  (결정론적 · 1회 실행)"),
        Line2D([], [], color=C_RRT, lw=2.0, label=f"RRT*  최선값 ({N_SEEDS}시드 중)"),
        Line2D([], [], color=C_RRT, lw=1.1, alpha=0.32, label="RRT*  개별 시드"),
        Line2D([], [], color=INK, lw=0, marker="s", markersize=7,
               markerfacecolor="white", label="출발"),
        Line2D([], [], color=INK, lw=0, marker="*", markersize=12,
               markerfacecolor="white", label="목표"),
    ]
    leg1 = ax.legend(handles=handles, loc="upper left", frameon=False,
                     fontsize=10.5, labelcolor=INK2, handlelength=2.2,
                     labelspacing=0.9, bbox_to_anchor=(0.0, 1.0))
    ax.add_artist(leg1)

    # 지형 등급 — 수동 사각형 대신 두 번째 범례로 (칩과 글자가 어긋나지 않게)
    terrain_handles = [
        Patch(facecolor=c, edgecolor="#d8d8d4", label=t)
        for c, t in zip(["#e8e8e6", "#a8cfe0", "#e8933a", "#2b2b2b"],
                        ["평지 (로버 통과)", "비행 필요 (3D 가능)",
                         "비행 필요 (4D 전용)", "통과 불가"])
    ]
    ax.legend(handles=terrain_handles, loc="upper left", frameon=False,
              fontsize=9.8, labelcolor=INK3, handlelength=1.4,
              labelspacing=0.6, bbox_to_anchor=(0.0, 0.52), title="지형",
              title_fontsize=9.8, alignment="left")

    n_sw_total = sum(d["astar"]["acc"].n_switches for d in data.values())
    fig.text(0.008, 0.035,
             "흐린 빨간 선이 퍼져 있을수록 RRT* 가 실행마다 다른 답을 냈다는 뜻이다. "
             "A* 는 결정론적이라 선이 하나다.",
             fontsize=10.5, color=INK2)
    fig.text(0.008, -0.005,
             f"주의 — 현재 비용함수에서는 5개 맵 전부 비행이 선택되지 않았다 "
             f"(A* 모드 전환 합계 {n_sw_total}회). 즉 이 비교는 사실상 지상 경로 비교이며, "
             f"연구일지 2 당시에는 2개 맵에서 비행이 선택되었다.",
             fontsize=10.5, color="#9c3a29")

    fig.suptitle("A* vs RRT* — 같은 문제, 같은 비용함수에서 나온 경로",
                 x=0.008, ha="left", fontsize=15, color=INK, fontweight="bold")
    fig.text(0.008, 0.945,
             f"3D 상태공간 · 격자 0.1 m · RRT* 예산 {RRT_BUDGET}초 · 양쪽 같은 스무더  "
             f"|  괄호 안 %는 A* 우위 (+면 A* 가 쌈).  연구일지 1·2 는 '평균' 기준으로 비교했다",
             fontsize=9.5, color=INK3)

    out = RESULTS / "fig_paths_astar_vs_rrt.png"
    fig.patch.set_facecolor(SURFACE)
    fig.savefig(out, dpi=190, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)
    print(f"\n저장: {out.relative_to(HERE.parent)}")


if __name__ == "__main__":
    main()
