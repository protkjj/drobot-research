"""비용함수 전후 비교 그래프 — 연구일지용.

benchmark/results/benchmark_costfn_ablation.json 을 읽어 그린다.
"""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plotstyle  # noqa: E402,F401

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

RESULTS = Path(__file__).resolve().parent / "results"
C_A, C_R = "#2b6cb0", "#dd6b20"
C_BEFORE, C_AFTER = "#a0aec0", "#2f855a"
C_GRID = "#dcdcd8"

d = json.loads((RESULTS / "benchmark_costfn_ablation.json").read_text())
before, after = d["before"], d["after"]
maps = [r["map"] for r in before["rows"]]


def _style(ax):
    ax.grid(True, color=C_GRID, linewidth=0.7, zorder=0)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)


fig = plt.figure(figsize=(14, 9.5))
gs = fig.add_gridspec(2, 2, hspace=0.36, wspace=0.24)

# ---------------------------------------------------------------- (1) 가중치 배분
ax = fig.add_subplot(gs[0, 0])
labels = ["에너지\nwE", "모드전환\nwS", "시간\nwT"]
bw, aw = before["weights"], after["weights"]
b_vals = [bw["share_energy"], bw["share_switch"], bw["share_time"]]
a_vals = [aw["share_energy"], aw["share_switch"], aw["share_time"]]
x = np.arange(3)
ax.bar(x - 0.2, b_vals, 0.4, color=C_BEFORE, label="전 (α/β/γ)", zorder=3)
ax.bar(x + 0.2, a_vals, 0.4, color=C_AFTER, label="후 (정규화)", zorder=3)
for i, (b, a) in enumerate(zip(b_vals, a_vals)):
    ax.text(i - 0.2, b + 1.5, f"{b:.1f}%", ha="center", fontsize=9)
    ax.text(i + 0.2, a + 1.5, f"{a:.1f}%", ha="center", fontsize=9)
ax.set_xticks(x)
ax.set_xticklabels(labels, fontsize=9)
ax.set_ylabel("비용에서 차지하는 비중 [%]", fontsize=9)
ax.set_ylim(0, 92)
ax.set_title("① 같은 계수를 정규화로 표현하니\n전환 페널티가 79%였음이 드러남",
             fontsize=10.5)
ax.legend(fontsize=8.5)
_style(ax)

# ---------------------------------------------------------------- (2) 모드 전환
ax = fig.add_subplot(gs[0, 1])
b_sw = [r["astar_switches"] or 0 for r in before["rows"]]
a_sw = [r["astar_switches"] or 0 for r in after["rows"]]
x = np.arange(len(maps))
ax.bar(x - 0.2, b_sw, 0.4, color=C_BEFORE, label="전", zorder=3)
ax.bar(x + 0.2, a_sw, 0.4, color=C_AFTER, label="후", zorder=3)
ax.set_xticks(x)
ax.set_xticklabels([m.replace("_", "\n") for m in maps], fontsize=8)
ax.set_ylabel("최적해의 모드 전환 횟수", fontsize=9)
ax.set_yticks([0, 1, 2])
ax.set_title("② 전에는 비행이 한 번도 선택되지 않음\n후에는 실제로 이착륙이 발생",
             fontsize=10.5)
ax.legend(fontsize=8.5)
ax.text(0.5, 1.55, "하이브리드 플래너인데\n비행이 죽은 옵션이었다",
        fontsize=8, color="#666", ha="center")
_style(ax)

# ---------------------------------------------------------------- (3) A* 우위
ax = fig.add_subplot(gs[1, 0])
b_adv = [r["astar_advantage_pct"] for r in before["rows"]]
a_adv = [r["astar_advantage_pct"] for r in after["rows"]]
ax.bar(x - 0.2, b_adv, 0.4, color=C_BEFORE, label="전", zorder=3)
ax.bar(x + 0.2, a_adv, 0.4, color=C_AFTER, label="후", zorder=3)
ax.axhline(0, color=C_A, linewidth=1.8, zorder=4)
ax.set_xticks(x)
ax.set_xticklabels([m.replace("_", "\n") for m in maps], fontsize=8)
ax.set_ylabel("A* 우위 [%]  (RRT* 평균 대비)", fontsize=9)
ax.set_title("③ A*는 두 조건 모두 5/5 우위\n다만 마진은 3.09% → 2.08% 로 축소",
             fontsize=10.5)
ax.legend(fontsize=8.5)
_style(ax)

# ---------------------------------------------------------------- (4) 비용 스케일
ax = fig.add_subplot(gs[1, 1])
b_cost = [r["astar_cost"] for r in before["rows"]]
a_cost = [r["astar_cost"] for r in after["rows"]]
ax.plot(x, b_cost, "o-", color=C_BEFORE, linewidth=2, markersize=7,
        label="전 (Wh + s 혼합)", zorder=3)
ax.plot(x, a_cost, "s-", color=C_AFTER, linewidth=2, markersize=6,
        label="후 (무차원)", zorder=3)
ax.set_xticks(x)
ax.set_xticklabels([m.replace("_", "\n") for m in maps], fontsize=8)
ax.set_ylabel("A* 최적해 비용 C", fontsize=9)
ax.set_title("④ 비용 스케일이 달라짐\n두 형태는 동등하지만 절대값 비교는 불가",
             fontsize=10.5)
ax.legend(fontsize=8.5)
_style(ax)

fig.suptitle("에너지 비용함수 정규화 — 적용 전후 비교", fontsize=13.5, y=0.975)
out = RESULTS / "fig_ablation.png"
fig.savefig(out, dpi=145, bbox_inches="tight")
print(f"저장: {out}")
