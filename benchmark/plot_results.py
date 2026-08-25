"""벤치마크 결과 시각화 — results/benchmark_*.json 을 읽어 그래프로.

run_benchmark.py 가 저장한 JSON만 읽는다. 실험을 다시 돌리지 않는다.

생성물 (results/)
    fig_time_budget.png   시간 예산별 해 품질 + 성공률
    fig_z_resolution.png  4D z해상도별 A* 한계
    fig_param_sweep.png   이착륙 에너지별 비행 선택과 우열
    fig_narrow.png        통로 폭별 성공률
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plotstyle  # noqa: E402,F401  (한글 폰트 설정)

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

RESULTS = Path(__file__).resolve().parent / "results"

# 공통 색: A*는 진한 파랑, RRT*는 주황
C_A = "#2b6cb0"
C_R = "#dd6b20"
C_GRID = "#d8d8d4"


def _load(name: str):
    p = RESULTS / f"benchmark_{name}.json"
    if not p.exists():
        return None, None
    with open(p) as f:
        d = json.load(f)
    return pd.DataFrame(d["rows"]), d["meta"]


def _style(ax):
    ax.grid(True, color=C_GRID, linewidth=0.7, zorder=0)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


# ---------------------------------------------------------------------------
def fig_time_budget():
    df, meta = _load("time_budget")
    if df is None:
        print("  건너뜀: time_budget 결과 없음")
        return

    maps = list(dict.fromkeys(df["map"]))
    n = len(maps)
    ncol = 3
    nrow = int(np.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(4.6 * ncol, 3.6 * nrow),
                             squeeze=False)

    for ax, name in zip(axes.flat, maps):
        sub = df[df["map"] == name]
        a = sub[sub["planner"] == "astar"].iloc[0]
        r = sub[(sub["planner"] == "rrt_star") & (sub["found"])]

        if a["found"]:
            ax.axhline(a["cost"], color=C_A, linewidth=2.2, zorder=3,
                       label=f"A* ({a['runtime_s']:.2f}s)")

        if len(r):
            g = r.groupby("budget_s")["cost"]
            bs = sorted(g.groups)
            mean = [g.get_group(b).mean() for b in bs]
            lo = [g.get_group(b).min() for b in bs]
            hi = [g.get_group(b).max() for b in bs]
            ax.plot(bs, mean, "o-", color=C_R, linewidth=1.8, markersize=5,
                    zorder=4, label="RRT* 평균")
            ax.fill_between(bs, lo, hi, color=C_R, alpha=0.18, zorder=2,
                            label="RRT* 최선~최악")

        # 성공률이 100% 미만인 예산에 표시
        allr = sub[sub["planner"] == "rrt_star"]
        if len(allr):
            for b, grp in allr.groupby("budget_s"):
                rate = grp["found"].mean()
                if rate < 1.0:
                    ax.axvspan(b * 0.88, b * 1.12, color="#c53030", alpha=0.10,
                               zorder=1)
                    ymin, ymax = ax.get_ylim()
                    ax.text(b, ymax, f"{rate*100:.0f}%", ha="center",
                            va="top", fontsize=7, color="#c53030")

        ax.set_xscale("log")
        ax.set_title(name, fontsize=10)
        ax.set_xlabel("시간 예산 [s]", fontsize=8)
        ax.set_ylabel("비용 C (낮을수록 좋음)", fontsize=8)
        ax.tick_params(labelsize=7.5)
        ax.legend(fontsize=7, loc="upper right", framealpha=0.9)
        _style(ax)

    for ax in axes.flat[n:]:
        ax.axis("off")

    fig.suptitle("동일 시간 예산에서의 해 품질 (3D)  —  붉은 구간은 RRT* 성공률 100% 미만",
                 fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = RESULTS / "fig_time_budget.png"
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"  저장 {out.name}")


# ---------------------------------------------------------------------------
def fig_z_resolution():
    df, meta = _load("z_resolution")
    if df is None:
        print("  건너뜀: z_resolution 결과 없음")
        return

    maps = list(dict.fromkeys(df["map"]))
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))

    # (1) A* 실행시간 vs 상태공간 크기
    ax = axes[0]
    for name in maps:
        sub = df[(df["map"] == name) & (df["planner"] == "astar")]
        sub = sub.sort_values("n_states")
        ok = sub[sub["found"]]
        to = sub[~sub["found"]]
        ax.plot(ok["n_states"], ok["runtime_s"], "o-", markersize=5,
                linewidth=1.6, label=name, zorder=3)
        if len(to):
            ax.plot(to["n_states"], to["runtime_s"], "x", markersize=10,
                    color="#c53030", zorder=4)
    to_s = meta.get("astar_timeout_s")
    if to_s:
        ax.axhline(to_s, color="#c53030", linestyle="--", linewidth=1.3,
                   label=f"timeout {to_s}s")
    ax.axhline(2.0, color="#2f855a", linestyle=":", linewidth=1.6,
               label="운용 제약 2.0s")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel("상태공간 크기 (노드 수)", fontsize=9)
    ax.set_ylabel("A* 실행시간 [s]", fontsize=9)
    ax.set_title("A*의 확장성 한계  (X = timeout)", fontsize=10.5)
    ax.legend(fontsize=7.5)
    _style(ax)

    # (2) z해상도별 승자
    ax = axes[1]
    zs = sorted(df["z_res"].unique(), reverse=True)
    ypos = np.arange(len(maps))
    for j, zr in enumerate(zs):
        for i, name in enumerate(maps):
            sub = df[(df["map"] == name) & (df["z_res"] == zr)]
            a = sub[sub["planner"] == "astar"]
            r = sub[(sub["planner"] == "rrt_star") & (sub["found"])]
            ca = a["cost"].iloc[0] if len(a) and a["found"].iloc[0] else None
            cr = r["cost"].min() if len(r) else None

            if ca is not None and cr is not None:
                color = C_A if ca <= cr else C_R
                txt = "A*" if ca <= cr else "RRT*"
            elif ca is not None:
                color, txt = C_A, "A*"
            elif cr is not None:
                color, txt = C_R, "RRT*"
            else:
                color, txt = "#999", "-"
            ax.add_patch(plt.Rectangle((j - 0.45, i - 0.42), 0.9, 0.84,
                                       facecolor=color, alpha=0.75, zorder=2))
            ax.text(j, i, txt, ha="center", va="center", fontsize=9,
                    color="white", fontweight="bold", zorder=3)

    ax.set_xticks(range(len(zs)))
    ax.set_xticklabels([f"{z}m" for z in zs], fontsize=9)
    ax.set_yticks(ypos)
    ax.set_yticklabels(maps, fontsize=9)
    ax.set_xlim(-0.6, len(zs) - 0.4)
    ax.set_ylim(-0.6, len(maps) - 0.4)
    ax.set_xlabel("z 해상도", fontsize=9)
    ax.set_title("z해상도별 승자 (2초 예산)", fontsize=10.5)
    ax.grid(False)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)

    fig.suptitle("4D 상태공간 — A*가 무너지는 경계", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out = RESULTS / "fig_z_resolution.png"
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"  저장 {out.name}")


# ---------------------------------------------------------------------------
def fig_param_sweep():
    df, meta = _load("param_sweep")
    if df is None:
        print("  건너뜀: param_sweep 결과 없음")
        return

    maps = list(dict.fromkeys(df["map"]))
    fig, axes = plt.subplots(2, len(maps), figsize=(4.4 * len(maps), 7),
                             squeeze=False)

    for k, name in enumerate(maps):
        sub = df[df["map"] == name]

        # 위: 비행 전환 횟수
        ax = axes[0][k]
        a = sub[sub["planner"] == "astar"].sort_values("takeoff_wh")
        ax.plot(a["takeoff_wh"], a["n_switches"], "o-", color=C_A,
                linewidth=1.8, markersize=5, label="A*")
        r = sub[(sub["planner"] == "rrt_star") & (sub["found"])]
        if len(r):
            g = r.groupby("takeoff_wh")["n_switches"].mean()
            ax.plot(g.index, g.values, "s--", color=C_R, linewidth=1.6,
                    markersize=4, label="RRT* 평균")
        ax.set_xscale("log")
        ax.set_title(name, fontsize=10)
        ax.set_ylabel("모드 전환 횟수", fontsize=8)
        ax.set_xlabel("takeoff 에너지 [Wh]", fontsize=8)
        ax.legend(fontsize=7)
        ax.tick_params(labelsize=7.5)
        _style(ax)

        # 아래: A* 대비 RRT* 상대 비용
        ax = axes[1][k]
        rel_x, rel_y, rel_lo, rel_hi = [], [], [], []
        for tk, grp in sub[sub["planner"] == "rrt_star"].groupby("takeoff_wh"):
            arow = a[a["takeoff_wh"] == tk]
            if not len(arow) or not arow["found"].iloc[0]:
                continue
            ca = arow["cost"].iloc[0]
            ok = grp[grp["found"]]
            if not len(ok):
                continue
            vals = 100 * (ok["cost"] - ca) / ca
            rel_x.append(tk)
            rel_y.append(vals.mean())
            rel_lo.append(vals.min())
            rel_hi.append(vals.max())
        if rel_x:
            ax.plot(rel_x, rel_y, "o-", color=C_R, linewidth=1.8, markersize=5)
            ax.fill_between(rel_x, rel_lo, rel_hi, color=C_R, alpha=0.18)
        ax.axhline(0, color=C_A, linewidth=2.0)
        ax.set_xscale("log")
        ax.set_xlabel("takeoff 에너지 [Wh]", fontsize=8)
        ax.set_ylabel("RRT* 비용 - A* 비용 [%]", fontsize=8)
        ax.text(0.02, 0.95, "0 위 = A*가 우수", transform=ax.transAxes,
                fontsize=7.5, va="top", color=C_A)
        ax.tick_params(labelsize=7.5)
        _style(ax)

    fig.suptitle("이착륙 에너지 파라미터 sweep  —  비행이 많이 선택되는 영역에서도 A*가 우위인가",
                 fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = RESULTS / "fig_param_sweep.png"
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"  저장 {out.name}")


# ---------------------------------------------------------------------------
def fig_narrow():
    df, meta = _load("narrow_passage")
    if df is None:
        print("  건너뜀: narrow_passage 결과 없음")
        return

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))

    gaps = sorted(df["gap_m"].unique(), reverse=True)

    # (1) 성공률
    ax = axes[0]
    a_rate = [100.0 * df[(df["gap_m"] == g) & (df["planner"] == "astar")]["found"].mean()
              for g in gaps]
    r_rate = [100.0 * df[(df["gap_m"] == g) & (df["planner"] == "rrt_star")]["found"].mean()
              for g in gaps]
    x = np.arange(len(gaps))
    ax.bar(x - 0.2, a_rate, 0.4, color=C_A, label="A*", zorder=3)
    ax.bar(x + 0.2, r_rate, 0.4, color=C_R, label="RRT* (2초)", zorder=3)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{g}m" for g in gaps], fontsize=8.5)
    ax.set_xlabel("통로 폭", fontsize=9)
    ax.set_ylabel("성공률 [%]", fontsize=9)
    ax.set_ylim(0, 108)
    ax.set_title("통로 폭별 성공률", fontsize=10.5)
    ax.legend(fontsize=8)
    _style(ax)

    # (2) 실행시간
    ax = axes[1]
    a_t = [df[(df["gap_m"] == g) & (df["planner"] == "astar")]["runtime_s"].mean()
           for g in gaps]
    rr = df[(df["planner"] == "rrt_star") & (df["found"])]
    r_t = [rr[rr["gap_m"] == g]["runtime_s"].mean() if len(rr[rr["gap_m"] == g])
           else np.nan for g in gaps]
    ax.plot(x, a_t, "o-", color=C_A, linewidth=1.9, markersize=5, label="A*")
    ax.plot(x, r_t, "s--", color=C_R, linewidth=1.7, markersize=4,
            label="RRT* (성공한 시드 평균)")
    ax.set_xticks(x)
    ax.set_xticklabels([f"{g}m" for g in gaps], fontsize=8.5)
    ax.set_xlabel("통로 폭", fontsize=9)
    ax.set_ylabel("해 도달 시간 [s]", fontsize=9)
    ax.set_yscale("log")
    ax.set_title("통로 폭별 해 도달 시간", fontsize=10.5)
    ax.legend(fontsize=8)
    _style(ax)

    fig.suptitle("narrow passage — 샘플링 기반 플래너의 통로 폭 의존성", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out = RESULTS / "fig_narrow.png"
    fig.savefig(out, dpi=140, bbox_inches="tight")
    plt.close(fig)
    print(f"  저장 {out.name}")


if __name__ == "__main__":
    print("결과 시각화")
    fig_time_budget()
    fig_z_resolution()
    fig_param_sweep()
    fig_narrow()
    print(f"완료 -> {RESULTS}")
