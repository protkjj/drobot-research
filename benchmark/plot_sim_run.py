"""시뮬레이션 주행 궤적 그림 — 포스터용.

record_run.py 가 Gazebo 실행에서 떠온 JSON 을 그린다.
Python 벤치마크 그래프와 다른 점: 이건 '실제로 주행한' 궤적이다.
    계획 경로 (/plan)        플래너가 낸 것
    실제 궤적 (/odom)        컨트롤러가 따라간 것        <- 둘의 차이가 보인다
    모드 전환 (/mode_switch_points)  이·착륙 지점

색은 plot_paths.py 와 같은 근거로 고른 것을 쓴다 (계산 결과는 그 파일 주석 참고).

사용법
    python3 benchmark/plot_sim_run.py benchmark/results/sim_easy_open.json
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import plotstyle  # noqa: E402,F401

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import BoundaryNorm, ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

from envs.heightmap import build_all, ROVER_MAX, H_3D_LIMIT, H_4D_LIMIT  # noqa: E402

C_PLAN, C_ODOM = "#4a3aa7", "#e34948"
INK, INK2, INK3 = "#0b0b0b", "#52514e", "#8a8a85"
SURFACE = "#fcfcfb"

TERRAIN_CMAP = ListedColormap(["#e8e8e6", "#a8cfe0", "#e8933a", "#2b2b2b"])
TERRAIN_NORM = BoundaryNorm([-1e-9, ROVER_MAX, H_3D_LIMIT, H_4D_LIMIT, 99.0],
                            TERRAIN_CMAP.N)


def main(path: str):
    d = json.load(open(path))
    world = d["world"]
    hm = build_all(res=0.1).get(world)

    fig, ax = plt.subplots(figsize=(11.5, 7.2))
    ax.set_facecolor(SURFACE)

    if hm is not None:
        ax.imshow(hm.grid, origin="lower", cmap=TERRAIN_CMAP, norm=TERRAIN_NORM,
                  extent=[0, hm.width_m, 0, hm.height_m], interpolation="nearest")
    ax.set_aspect("equal")

    if d["plan"]:
        ax.plot([p[0] for p in d["plan"]], [p[1] for p in d["plan"]],
                color=C_PLAN, lw=2.6, zorder=5, label="계획 경로 (/plan)")
    if d["odom"]:
        ax.plot([o[1] for o in d["odom"]], [o[2] for o in d["odom"]],
                color=C_ODOM, lw=2.2, zorder=6, alpha=0.9,
                label="실제 주행 (/odom)")

    for s in d["mode_switches"]:
        take_off = s["type"] == 0
        ax.plot(s["x"], s["y"], marker="^" if take_off else "v", markersize=11,
                color=C_PLAN, markeredgecolor="white", markeredgewidth=1.5,
                zorder=8)

    sx, sy = (d["odom"][0][1], d["odom"][0][2]) if d["odom"] else (0, 0)
    ax.plot(sx, sy, marker="s", markersize=10, color="white",
            markeredgecolor=INK, markeredgewidth=1.6, zorder=9)
    ax.plot(*d["goal"], marker="*", markersize=19, color="white",
            markeredgecolor=INK, markeredgewidth=1.4, zorder=9)

    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_color("#d8d8d4")

    handles = [
        Line2D([], [], color=C_PLAN, lw=2.6, label="계획 경로 (/plan)"),
        Line2D([], [], color=C_ODOM, lw=2.2, label="실제 주행 (/odom)"),
        Line2D([], [], color=INK, lw=0, marker="s", markersize=9,
               markerfacecolor="white", label="출발"),
        Line2D([], [], color=INK, lw=0, marker="*", markersize=15,
               markerfacecolor="white", label="목표"),
    ]
    if d["mode_switches"]:
        handles.insert(2, Line2D([], [], color=C_PLAN, lw=0, marker="^",
                                 markersize=10, label="이륙"))
        handles.insert(3, Line2D([], [], color=C_PLAN, lw=0, marker="v",
                                 markersize=10, label="착륙"))
    ax.legend(handles=handles, loc="lower left", frameon=False, fontsize=11,
              labelcolor=INK2, ncol=len(handles), handlelength=1.8,
              bbox_to_anchor=(0.0, -0.10))

    # 제목과 메모는 축 좌표계에 붙인다 — fig.text 를 쓰면 bbox_inches="tight"
    # 크롭 때문에 제목과 겹친다 (가짜 데이터로 확인함)
    n_sw = len(d["mode_switches"])
    note = (f"{d['duration_s']}초 · 재계획 {d['n_replans']}회 · "
            f"모드 전환 {n_sw}회 · 결과 {d['result']}")
    if n_sw == 0:
        note += "      모드 전환 0회 = 이 주행에서는 비행을 쓰지 않았다"
    ax.set_title(f"Gazebo 주행 궤적 — {world}\n{note}", loc="left",
                 fontsize=15, color=INK, fontweight="bold",
                 pad=14, linespacing=1.9)

    out = Path(path).with_suffix(".png")
    fig.patch.set_facecolor(SURFACE)
    fig.savefig(out, dpi=200, bbox_inches="tight", facecolor=SURFACE)
    print(f"저장 {out}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit("사용법: python3 benchmark/plot_sim_run.py <sim_*.json>")
    main(sys.argv[1])
