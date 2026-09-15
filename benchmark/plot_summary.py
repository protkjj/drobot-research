"""연구일지용 종합 그래프.

지금까지 측정한 핵심 수치를 한 장으로 정리한다.
숫자는 각 실험 스크립트의 실측값을 그대로 옮긴 것이며,
출처를 주석으로 남겨 재현 가능하게 한다.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plotstyle  # noqa: E402,F401

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

RESULTS = Path(__file__).resolve().parent / "results"
C_A, C_R = "#2b6cb0", "#dd6b20"
C_GRID = "#dcdcd8"


def _style(ax):
    ax.grid(True, color=C_GRID, linewidth=0.7, zorder=0)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


fig = plt.figure(figsize=(14.5, 10))
gs = fig.add_gridspec(2, 2, hspace=0.32, wspace=0.24)

# ---------------------------------------------------------------- (1)
# 출처: 3D 재측정 (RRT* 착륙버그 수정 후, 2초 예산, 시드 20개)
ax = fig.add_subplot(gs[0, 0])
maps = ["easy_open", "easy_corridor", "medium_open", "medium_corridor", "hard_open"]
a_cost = [44.276, 47.744, 72.088, 79.256, 79.407]
r_mean = [44.278, 48.473, 74.158, 79.502, 81.671]
r_std = [0.250, 0.305, 0.552, 0.627, 0.726]

x = np.arange(len(maps))
ax.bar(x - 0.2, a_cost, 0.4, color=C_A, label="A* (결정론적)", zorder=3)
ax.bar(x + 0.2, r_mean, 0.4, yerr=r_std, color=C_R, capsize=4,
       label="RRT* (평균±σ, 시드 20)", zorder=3,
       error_kw={"linewidth": 1.2, "ecolor": "#7a3d10"})
ax.set_xticks(x)
ax.set_xticklabels([m.replace("_", "\n") for m in maps], fontsize=8)
ax.set_ylabel("비용 C (낮을수록 좋음)", fontsize=9)
ax.set_title("① 현재 config (takeoff 5.0Wh) — 비행 0회\nA* 5/5 우위",
             fontsize=10.5)
ax.legend(fontsize=8, loc="upper left")
ax.set_ylim(0, 95)
_style(ax)
for i, (a, r) in enumerate(zip(a_cost, r_mean)):
    ax.text(i, max(a, r) + 2.5, f"{100*(r-a)/a:+.1f}%", ha="center",
            fontsize=7.5, color=C_A if r > a else C_R)

# ---------------------------------------------------------------- (2)
# 출처: benchmark_param_sweep.json
ax = fig.add_subplot(gs[0, 1])
tk = [0.1, 0.3, 0.7, 1.5, 3.0, 5.0]
# A* 대비 RRT* 상대비용 (음수 = RRT*가 더 좋음)
rel_medium = [-3.06, -3.08, -3.16, -2.95, -3.18, +2.74]
rel_hard = [+0.08, -0.13, -1.08, -2.54, +3.10, +3.15]
sw_medium = [2, 2, 2, 2, 2, 0]     # A*의 모드 전환 횟수

ax.axhline(0, color=C_A, linewidth=2.0, zorder=3)
ax.plot(tk, rel_medium, "o-", color=C_R, linewidth=1.9, markersize=6,
        label="medium_open", zorder=4)
ax.plot(tk, rel_hard, "s--", color="#9c4221", linewidth=1.7, markersize=5,
        label="hard_open", zorder=4)
ax.set_xscale("log")
ax.set_xticks(tk)
ax.set_xticklabels([str(v) for v in tk], fontsize=8)
ax.minorticks_off()
ax.set_xlabel("takeoff 에너지 [Wh]  (낮을수록 비행이 활발)", fontsize=9)
ax.set_ylabel("RRT* 비용 - A* 비용 [%]", fontsize=9)
ax.set_title("② 파라미터 sweep — 비행이 활발해지면 역전", fontsize=10.5)
ax.set_ylim(-4.6, 5.4)
ax.legend(fontsize=8, loc="lower left")
ax.axvspan(0.08, 2.2, color=C_R, alpha=0.07, zorder=1)
ax.axvspan(2.2, 6.5, color=C_A, alpha=0.07, zorder=1)
ax.text(0.42, 4.5, "비행 활발 → RRT* 우위", fontsize=8, color="#9c4221",
        ha="center")
ax.text(3.6, 4.5, "비행 없음\n→ A* 우위", fontsize=8, color=C_A, ha="center")
ax.text(0.5, -4.15, "0 아래 = RRT*가 우수", fontsize=7.5, color="#666")
ax.annotate("현재 config", xy=(5.0, 3.15), xytext=(2.3, 1.6), fontsize=7.5,
            arrowprops={"arrowstyle": "->", "color": "#555", "linewidth": 0.9})
_style(ax)

# ---------------------------------------------------------------- (3)
# 출처: 격자 해상도 가설 검증 (takeoff 0.3Wh)
ax = fig.add_subplot(gs[1, 0])
res_list = [0.20, 0.10, 0.05]
rel_m = [-4.56, -3.09, -0.50]
rel_h = [-2.19, -0.31, +1.44]
t_m = [0.20, 0.92, 3.40]
t_h = [0.23, 0.97, 4.52]

ax.axhline(0, color=C_A, linewidth=2.0, zorder=3)
ax.plot(res_list, rel_m, "o-", color=C_R, linewidth=1.9, markersize=6,
        label="medium_open", zorder=4)
ax.plot(res_list, rel_h, "s--", color="#9c4221", linewidth=1.7, markersize=5,
        label="hard_open", zorder=4)
ax.invert_xaxis()
ax.set_xlabel("격자 해상도 [m]  (왼쪽이 성김)", fontsize=9)
ax.set_ylabel("RRT* 비용 - A* 비용 [%]", fontsize=9)
ax.set_title("③ A*의 열세는 격자 해상도 탓\n촘촘하게 하면 따라잡지만…", fontsize=10.5)
ax.set_ylim(-5.6, 2.6)
ax.legend(fontsize=8, loc="lower left")
for r, tm in zip(res_list, t_m):
    ax.text(r, 2.05, f"A* {tm:.1f}s", fontsize=7.5, ha="center", color="#666")
ax.text(0.125, -5.15, "촘촘할수록 A*가 유리해지지만 시간이 급증한다",
        fontsize=7.5, color="#666", ha="center")
_style(ax)

# ---------------------------------------------------------------- (4)
ax = fig.add_subplot(gs[1, 1])
labels = ["0.20m", "0.10m", "0.05m"]
xx = np.arange(3)
ax.bar(xx - 0.2, t_m, 0.4, color=C_A, label="medium_open", zorder=3)
ax.bar(xx + 0.2, t_h, 0.4, color="#63b3ed", label="hard_open", zorder=3)
ax.axhline(2.0, color="#c53030", linestyle="--", linewidth=1.8, zorder=4,
           label="운용 제약 2.0s")
ax.set_xticks(xx)
ax.set_xticklabels(labels, fontsize=9)
ax.set_xlabel("격자 해상도", fontsize=9)
ax.set_ylabel("A* 실행시간 [s]", fontsize=9)
ax.set_title("④ 그 대가는 시간\nA*가 이기는 해상도는 제약을 위반한다", fontsize=10.5)
ax.legend(fontsize=8, loc="upper left")
for i, (a, b) in enumerate(zip(t_m, t_h)):
    ax.text(i - 0.2, a + 0.12, f"{a:.1f}", ha="center", fontsize=7.5)
    ax.text(i + 0.2, b + 0.12, f"{b:.1f}", ha="center", fontsize=7.5)
_style(ax)

fig.suptitle("A* vs RRT* — drobot 하이브리드 경로계획 벤치마크 요약",
             fontsize=13.5, y=0.975)
out = RESULTS / "fig_summary.png"
fig.savefig(out, dpi=145, bbox_inches="tight")
print(f"저장: {out}")
