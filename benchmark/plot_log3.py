"""연구일지 3 그림 — 3 modal 비교.

RESEARCH_LOG3.md 를 줄글로 쓰면서 표·수식을 본문에서 뺐다.
꼭 필요한 것만 여기서 PNG 로 만든다.

색 선택 근거
    기존 보고서(modal_report.html)는 우회=회색 / 밟고넘기=앰버 / 하이브리드=파랑을
    썼는데, 팔레트 검증에서 실패했다.
        회색 #767d78 ↔ 앰버 #a4661f : 정상시야 ΔE 11.6 (구분 하한 15 미달)
        회색·파랑은 채도 하한 미달로 무채색처럼 읽힘
    검증을 통과하는 팔레트로 교체했다 (파랑/주황/청록, all-pairs 통과).
    대비 WARN 이 붙는 색은 직접 라벨을 달아 해소한다.

실행: python3 benchmark/plot_log3.py
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Rectangle  # noqa: E402

import plotstyle  # noqa: E402,F401  (import 만으로 한글 폰트가 설정된다)

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"

# 검증 통과 팔레트 (dataviz 레퍼런스 슬롯 1~5)
BLUE, ORANGE, AQUA, YELLOW, MAGENTA = "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4"

# modal 식별색 — 세 그림에서 같은 대상은 같은 색
C_DETOUR, C_CLIMB, C_HYBRID = BLUE, ORANGE, AQUA
KR = {"rover_detour": "우회", "rover_climb": "밟고넘기", "hybrid": "하이브리드"}
CMAP = {"rover_detour": C_DETOUR, "rover_climb": C_CLIMB, "hybrid": C_HYBRID}

INK, INK2, INK3 = "#0b0b0b", "#52514e", "#8a8a85"
GRID = "#e6e6e2"
SURFACE = "#fcfcfb"

MAPS = ["easy_open", "easy_corridor", "medium_open", "medium_corridor", "hard_open"]
MAP_COLOR = dict(zip(MAPS, [BLUE, ORANGE, AQUA, YELLOW, MAGENTA]))


def _base(ax):
    """공통 축 스타일 — 격자·축은 뒤로 물린다."""
    ax.set_facecolor(SURFACE)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(GRID)
    ax.tick_params(colors=INK2, labelsize=9, length=0)
    ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)


def _save(fig, name):
    out = RESULTS / name
    fig.patch.set_facecolor(SURFACE)
    fig.savefig(out, dpi=200, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)
    print(f"  저장 {out.relative_to(HERE.parent)}")


# ---------------------------------------------------------------- 그림 1
def fig_modal_matrix():
    """3 modal 이 무엇을 할 수 있는가 — 능력 매트릭스.

    표를 그림으로 옮긴 것. 셀에 O/X 대신 채움/빈칸을 써서
    색이 아니라 형태로도 읽히게 한다.
    """
    rows = [
        ("우회\nrover_detour", C_DETOUR, [1, 0, 0], "0.15 m"),
        ("밟고넘기\nrover_climb", C_CLIMB, [1, 1, 0], "0.70 m"),
        ("하이브리드\nhybrid", C_HYBRID, [1, 0, 1], "0.15 m"),
    ]
    cols = ["돌아가기", "밟고 넘기", "날아 넘기"]

    fig, ax = plt.subplots(figsize=(7.4, 3.1))
    ax.set_facecolor(SURFACE)
    ax.set_xlim(-0.05, 4.55)
    ax.set_ylim(-0.4, 3.5)
    ax.axis("off")

    for j, c in enumerate(cols):
        ax.text(1.0 + j, 3.15, c, ha="center", va="bottom",
                fontsize=10, color=INK2)
    ax.text(4.3, 3.15, "로버 통과높이", ha="center", va="bottom",
            fontsize=10, color=INK2)

    for i, (name, color, caps, hlim) in enumerate(rows):
        y = 2 - i
        ax.text(-0.05, y + 0.4, name, ha="left", va="center",
                fontsize=10.5, color=INK, linespacing=1.5)
        for j, on in enumerate(caps):
            x = 0.62 + j
            if on:
                ax.add_patch(Rectangle((x, y + 0.12), 0.76, 0.56,
                                       facecolor=color, edgecolor="none"))
                ax.text(x + 0.38, y + 0.4, "가능", ha="center", va="center",
                        fontsize=9.5, color="white", fontweight="bold")
            else:
                ax.add_patch(Rectangle((x, y + 0.12), 0.76, 0.56,
                                       facecolor="none", edgecolor=GRID,
                                       linewidth=1.4))
        weight = "bold" if hlim.startswith("0.70") else "normal"
        ax.text(4.3, y + 0.4, hlim, ha="center", va="center",
                fontsize=10.5, color=INK, fontweight=weight)

    ax.text(-0.05, -0.25,
            "밟고넘기·하이브리드는 각각 우회의 상위집합 — 비용이 우회보다 클 수 없다",
            ha="left", va="center", fontsize=9, color=INK3)
    ax.set_title("세 modal 의 능력", loc="left", fontsize=12.5,
                 color=INK, fontweight="bold", pad=16)
    _save(fig, "fig3_modal.png")


# ---------------------------------------------------------------- 그림 2
def fig_costfn():
    """비용함수 결함 — 같은 1 Wh 의 비용 기여가 40배 차이.

    왼쪽 두 막대는 modal 이 아니라 '에너지를 어디서 쓰는가'이므로
    modal 식별색을 쓰지 않는다. 문제가 되는 쪽만 강조색을 준다.
    """
    NEUTRAL, FLAG = "#b9b9b2", "#e34948"     # 회색 / 경고 빨강(status)

    fig, (ax0, ax1) = plt.subplots(
        1, 2, figsize=(10.2, 3.3), gridspec_kw={"width_ratios": [1.3, 1]})
    fig.subplots_adjust(wspace=0.42)

    # --- 왼쪽: 1 Wh 의 비용 기여 ---
    _base(ax0)
    ax0.grid(axis="y", visible=False)
    vals = [1.000, 0.025]
    bars = ax0.barh([1, 0], vals, height=0.42,
                    color=[NEUTRAL, FLAG], zorder=3)
    for b, v in zip(bars, vals):
        ax0.text(v + 0.035, b.get_y() + b.get_height() / 2, f"{v:.3f}",
                 va="center", fontsize=12, color=INK, fontweight="bold")
    ax0.set_yticks([1, 0])
    ax0.set_yticklabels(["주행으로 1 Wh", "모드 전환으로 1 Wh"],
                        fontsize=10.5, color=INK)
    ax0.set_xlim(0, 1.3)
    ax0.set_xlabel("비용 기여", fontsize=9.5, color=INK2)
    ax0.text(0.62, 0.5, "40배 차이", fontsize=13, color=FLAG,
             fontweight="bold", ha="center", va="center")
    ax0.set_title("수정 전 — 같은 1 Wh 인데 소모 지점에 따라 다르게 계상됨",
                  loc="left", fontsize=11, color=INK, fontweight="bold", pad=10)

    # --- 오른쪽: 그 결과 (총 에너지) ---
    _base(ax1)
    ax1.grid(axis="x", visible=False)
    energy = [14.1, 52.9]
    bars = ax1.bar([0, 1], energy, width=0.46,
                   color=[C_CLIMB, C_HYBRID], zorder=3)
    for b, e in zip(bars, energy):
        ax1.text(b.get_x() + b.get_width() / 2, e + 2.0, f"{e:.1f} Wh",
                 ha="center", fontsize=11, color=INK, fontweight="bold")
    ax1.text(1, 28, "이쪽이\n'최적'으로\n선정됨", ha="center", va="center",
             fontsize=10, color="white", fontweight="bold", linespacing=1.4)
    ax1.set_xticks([0, 1])
    ax1.set_xticklabels(["밟고넘기", "하이브리드"], fontsize=10.5, color=INK)
    ax1.set_ylim(0, 66)
    ax1.set_ylabel("총 에너지 (Wh)", fontsize=9.5, color=INK2, labelpad=8)
    ax1.set_title("medium_open — 3.7배 쓰는 경로가 이김",
                  loc="left", fontsize=11, color=INK, fontweight="bold", pad=10)

    fig.text(0.005, -0.08,
             "수정: C = wE·(E_motion + E_switch)/E_ref + wS·n_switch + wT·T/T_ref"
             "    — 에너지는 같은 참조값으로, 전환은 횟수 페널티로 분리",
             fontsize=9.5, color=INK2)
    _save(fig, "fig3_costfn.png")


# ---------------------------------------------------------------- 그림 3
def fig_climb_sweep():
    """등반계수에 따른 밟고넘기의 우회 대비 절감률.

    선 끝 직접 라벨을 쓰지 않는 이유: 다섯 선이 오른쪽에서 전부 0 으로
    수렴해 라벨이 겹친다. 빈 우상단에 범례를 둔다 (색 단독 식별이 아니게).
    """
    d = json.load(open(RESULTS / "benchmark_climb_sweep.json"))
    rows, cvs = d["rows"], d["climb_values"]

    fig, ax = plt.subplots(figsize=(8.4, 4.6))
    _base(ax)

    for name in MAPS:
        ys = []
        for cv in cvs:
            r = next(x for x in rows
                     if x["climb_wh_per_m"] == cv and x["map"] == name)
            ys.append(100 * (r["rover_detour_cost"] - r["rover_climb_cost"])
                      / r["rover_detour_cost"])
        ax.plot(cvs, ys, color=MAP_COLOR[name], linewidth=2.2,
                marker="o", markersize=4.5, zorder=3, label=name,
                markeredgecolor=SURFACE, markeredgewidth=1)

    ax.axvline(2.0, color=INK3, linewidth=1.2, linestyle="--", zorder=2)
    ax.annotate("현재 추정치 2.0\n(미검증)", xy=(2.0, 19.5), xytext=(4.6, 20.6),
                fontsize=9.5, color=INK2, linespacing=1.4,
                arrowprops=dict(arrowstyle="-", color=INK3, lw=1))

    ax.set_xlim(-1.2, 26.5)
    ax.set_ylim(-1.2, 24)
    ax.set_xticks([0, 5, 10, 15, 20, 25])
    ax.set_xlabel("등반계수   rover_climb_wh_per_m   (Wh/m)",
                  fontsize=10, color=INK2, labelpad=8)
    ax.set_ylabel("우회 대비 비용 절감률 (%)", fontsize=10, color=INK2, labelpad=8)
    ax.legend(frameon=False, fontsize=9.5, loc="upper right",
              labelcolor=INK2, handlelength=1.6, borderaxespad=0.4)
    ax.set_title("밟고넘기의 이득은 등반계수에 단조 감소한다",
                 loc="left", fontsize=12.5, color=INK, fontweight="bold", pad=14)

    fig.text(0.008, -0.035,
             "실측값이 나오면 해당 x 좌표를 읽으면 된다.   "
             "medium_corridor 는 0.7 m 이하 장애물이 1개뿐이라 처음부터 이득이 없다",
             fontsize=9, color=INK3)
    _save(fig, "fig3_climb.png")


# ---------------------------------------------------------------- 그림 4
def fig_weight_sweep():
    """에너지 가중치에 따른 승자 분포 + 하이브리드가 이길 때의 에너지 대가."""
    rows = json.load(open(RESULTS / "benchmark_weight_sweep.json"))["rows"]
    TOL = 1e-9

    wes = sorted({r["wE"] for r in rows})
    stack = {m: [] for m in ("hybrid", "rover_climb", "rover_detour")}
    ties = []
    for we in wes:
        sub = [r for r in rows if r["wE"] == we]
        for m in stack:
            stack[m].append(sum(1 for r in sub
                                if r["winner"] == m and not r["tied"]))
        ties.append(sum(1 for r in sub if r["tied"]))

    fig, (ax0, ax1) = plt.subplots(
        1, 2, figsize=(10.6, 4.2), gridspec_kw={"width_ratios": [1.25, 1]})

    # --- 왼쪽: wE 별 승자 스택 ---
    _base(ax0)
    ax0.grid(axis="x", visible=False)
    x = np.arange(len(wes))
    bottom = np.zeros(len(wes))
    for m in ("hybrid", "rover_climb", "rover_detour"):
        v = np.array(stack[m], dtype=float)
        ax0.bar(x, v, bottom=bottom, width=0.62, color=CMAP[m],
                label=KR[m], zorder=3, edgecolor=SURFACE, linewidth=2)
        bottom += v
    ax0.bar(x, ties, bottom=bottom, width=0.62, color=GRID,
            label="동률(축퇴)", zorder=3, edgecolor=SURFACE, linewidth=2)

    ax0.axvspan(-0.5, 2.5, color=C_HYBRID, alpha=0.07, zorder=1)
    # 막대 위 여백에 배치 — wE=0.0 막대가 55 까지 올라오므로 그보다 위에 둔다
    ax0.text(1.0, 62.5, "하이브리드가 이기는 구간", ha="center", fontsize=9.5,
             color=C_HYBRID, fontweight="bold")
    ax0.set_xticks(x)
    ax0.set_xticklabels([f"{w:.1f}" for w in wes], fontsize=9)
    ax0.set_xlim(-0.7, len(wes) - 0.3)
    ax0.set_ylim(0, 68)
    ax0.set_xlabel("에너지 가중치  wE", fontsize=10, color=INK2)
    ax0.set_ylabel("승리 조건 수", fontsize=10, color=INK2)
    ax0.legend(frameon=False, fontsize=9.5, loc="upper right",
               labelcolor=INK2, handlelength=1.2, borderaxespad=0.3)
    ax0.set_title("wE ≥ 0.3 에서 비행은 한 번도 이기지 않는다",
                  loc="left", fontsize=12, color=INK, fontweight="bold", pad=12)

    # --- 오른쪽: 이길 때의 에너지 대가 ---
    _base(ax1)
    ax1.grid(axis="x", visible=False)
    sel = [r for r in rows
           if (r["wE"], r["wS"], r["wT"]) == (0.0, 0.0, 1.0)]
    sel = [next(r for r in sel if r["map"] == n) for n in MAPS]
    xs = np.arange(len(MAPS))
    ax1.bar(xs - 0.19, [r["rover_climb_energy_wh"] for r in sel], width=0.36,
            color=C_CLIMB, label="밟고넘기", zorder=3)
    ax1.bar(xs + 0.19, [r["hybrid_energy_wh"] for r in sel], width=0.36,
            color=C_HYBRID, label="하이브리드 (승자)", zorder=3)
    for i, r in enumerate(sel):
        ratio = r["hybrid_energy_wh"] / r["rover_climb_energy_wh"]
        ax1.text(i + 0.19, r["hybrid_energy_wh"] + 1.6, f"{ratio:.1f}×",
                 ha="center", fontsize=9.5, color=INK, fontweight="bold")
    ax1.set_xticks(xs)
    ax1.set_xticklabels([m.replace("_", "\n") for m in MAPS], fontsize=8.5)
    ax1.set_ylim(0, 72)
    ax1.set_ylabel("총 에너지 (Wh)", fontsize=10, color=INK2)
    ax1.legend(frameon=False, fontsize=9.5, loc="upper left",
               labelcolor=INK2, handlelength=1.2)
    ax1.set_title("이길 때도 에너지는 3~4.6배 쓴다  (wE=0, wT=1)",
                  loc="left", fontsize=12, color=INK, fontweight="bold", pad=12)

    _save(fig, "fig3_weight.png")


if __name__ == "__main__":
    print("연구일지 3 그림 생성")
    fig_modal_matrix()
    fig_costfn()
    fig_climb_sweep()
    fig_weight_sweep()
