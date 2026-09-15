"""A* vs RRT* 경로 — 3D 지형 위에 그린다.

plot_paths.py 의 평면도는 높이를 못 보여준다. 벽이 0.55 m 인지 1.40 m 인지
구분이 안 되니 "왜 여기서는 돌아가고 저기서는 안 도는가"가 읽히지 않는다.
같은 데이터를 지형 높이와 함께 그린다.

읽는 법
    바닥 회색      평지 (로버 통과)
    솟은 파랑/주황  장애물 — 높이가 곧 막대 높이다
    검정           통과 불가 벽
    경로는 지상 주행이면 지형 위에, 비행이면 실제 비행 고도에 그려진다.

실행: python3 benchmark/plot_paths_3d.py [맵이름 ...]
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import plotstyle  # noqa: E402,F401

import numpy as np  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap, BoundaryNorm  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

from cost.energy import EnergyModel  # noqa: E402
from envs.heightmap import build_all, ROVER_MAX, H_3D_LIMIT, H_4D_LIMIT  # noqa: E402
from planners.state_space import ProblemSpec, AIR  # noqa: E402
from planners.grid_search import astar  # noqa: E402
from planners.rrt_star import rrt_star  # noqa: E402
from planners.smoothing import smooth_result  # noqa: E402

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"

DEFAULT_MAPS = ["easy_open", "medium_open", "hard_open"]
N_SEEDS = 5
RRT_BUDGET = 2.0

C_ASTAR, C_RRT = "#4a3aa7", "#e34948"
INK, INK2, INK3 = "#0b0b0b", "#52514e", "#8a8a85"
SURFACE = "#fcfcfb"

TERRAIN_CMAP = ListedColormap(["#e8e8e6", "#a8cfe0", "#e8933a", "#2b2b2b"])
TERRAIN_NORM = BoundaryNorm([-1e-9, ROVER_MAX, H_3D_LIMIT, H_4D_LIMIT, 99.0],
                            TERRAIN_CMAP.N)

# 벽(3.0 m)을 그대로 그리면 장애물(0.35~1.4 m)이 바닥에 붙어 보인다.
# 벽만 잘라서 높이 비교가 읽히게 한다 — 축 라벨에 명시한다.
WALL_CLIP = 1.8


def draw_terrain_3d(ax, hm, stride=2):
    """높이맵을 3D 표면으로. 등급별 색은 평면도와 같은 분류를 쓴다."""
    g = np.clip(hm.grid, 0, WALL_CLIP)
    ny, nx = g.shape
    xs = np.arange(0, nx, stride) * hm.resolution
    ys = np.arange(0, ny, stride) * hm.resolution
    X, Y = np.meshgrid(xs, ys)
    Z = g[::stride, ::stride]

    # 색은 '자른 높이'가 아니라 원래 높이로 판정해야 벽이 검정으로 남는다
    raw = hm.grid[::stride, ::stride]
    colors = TERRAIN_CMAP(TERRAIN_NORM(raw))

    ax.plot_surface(X, Y, Z, facecolors=colors, rstride=1, cstride=1,
                    linewidth=0, antialiased=False, shade=False, zorder=1)


def draw_path_3d(ax, hm, path, color, lw=2.2, alpha=1.0, zorder=5):
    """경로를 3D 로. 지상 구간은 지형 바로 위, 비행 구간은 실제 고도."""
    if not path:
        return
    LIFT = 0.06                      # 지형에 파묻히지 않게 살짝 띄운다

    def terrain_at(x, y):
        ix, iy = hm.to_idx(x, y)
        ix = min(max(ix, 0), hm.nx - 1)
        iy = min(max(iy, 0), hm.ny - 1)
        return float(hm.grid[iy, ix])

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
        flying = seg[0][3] == AIR
        xs = [p[0] for p in seg]
        ys = [p[1] for p in seg]
        zs = [p[2] + LIFT if flying else terrain_at(p[0], p[1]) + LIFT
              for p in seg]
        ax.plot(xs, ys, zs, color=color, linewidth=lw, alpha=alpha,
                zorder=zorder, linestyle=(0, (2.2, 1.6)) if flying else "-")


def run_one(hm, model):
    spec = ProblemSpec(hm=hm, model=model, dims=3)
    ra = astar(spec, timeout=120.0)
    a_cost, a_acc, a_path = smooth_result(spec, ra, is_grid=True)

    best = None
    for s in range(N_SEEDS):
        r = rrt_star(spec, seed=s, max_samples=10 ** 7, timeout=RRT_BUDGET)
        if not r.found:
            continue
        c, acc, pth = smooth_result(spec, r, is_grid=False)
        if best is None or c < best["cost"]:
            best = {"cost": c, "acc": acc, "path": pth}
    return {"astar": {"cost": a_cost, "acc": a_acc, "path": a_path},
            "rrt": best}


def main(names):
    model = EnergyModel.from_yaml()
    maps = build_all(res=0.1)

    print(f"3D 경로 그림 — 맵 {len(names)}개 (RRT* {N_SEEDS}시드 x {RRT_BUDGET}초)")
    data = {}
    for n in names:
        t0 = time.perf_counter()
        data[n] = run_one(maps[n], model)
        d = data[n]
        print(f"  {n:<17} A* {d['astar']['cost']:7.3f} (전환 {d['astar']['acc'].n_switches})"
              f" | RRT* {d['rrt']['cost']:7.3f} (전환 {d['rrt']['acc'].n_switches})"
              f"   {time.perf_counter()-t0:.0f}초")

    ncol = len(names)
    fig = plt.figure(figsize=(6.0 * ncol, 6.2))
    fig.subplots_adjust(top=0.86, bottom=0.10)
    for i, n in enumerate(names):
        ax = fig.add_subplot(1, ncol, i + 1, projection="3d",
                             computed_zorder=False)
        hm = maps[n]
        draw_terrain_3d(ax, hm)
        d = data[n]
        if d["rrt"]:
            draw_path_3d(ax, hm, d["rrt"]["path"], C_RRT, lw=2.0, zorder=6)
        draw_path_3d(ax, hm, d["astar"]["path"], C_ASTAR, lw=2.6, zorder=7)

        sx, sy = hm.start
        gx, gy = hm.goal
        ax.scatter([sx], [sy], [0.12], s=55, c="white", edgecolors=INK,
                   linewidths=1.4, marker="s", zorder=10, depthshade=False)
        ax.scatter([gx], [gy], [0.12], s=150, c="white", edgecolors=INK,
                   linewidths=1.2, marker="*", zorder=10, depthshade=False)

        # z 를 실제 비율보다 과장한다 — 1.8 m 를 20 m 축척으로 그리면 안 보인다.
        # 과장했다는 사실은 부제에 명시한다.
        ax.set_box_aspect((hm.width_m, hm.height_m, 6.5))
        ax.view_init(elev=46, azim=-62)
        ax.set_zlim(0, WALL_CLIP)
        ax.set_xticks(np.arange(0, hm.width_m + 1, 5))
        ax.set_yticks(np.arange(0, hm.height_m + 1, 5))
        ax.set_zticks([0, 0.5, 1.0, 1.5])          # 뭉개지지 않게 4개만
        ax.set_xlabel("x (m)", fontsize=9, color=INK2, labelpad=2)
        ax.set_ylabel("y (m)", fontsize=9, color=INK2, labelpad=2)
        ax.set_zlabel("높이 (m)", fontsize=9, color=INK2, labelpad=1)
        ax.tick_params(labelsize=8, colors=INK3, pad=1)
        ax.set_facecolor(SURFACE)
        for pane in (ax.xaxis, ax.yaxis, ax.zaxis):
            pane.pane.set_facecolor(SURFACE)
            pane.pane.set_edgecolor("#e6e6e2")
        ax.grid(False)

        a, r = d["astar"]["cost"], d["rrt"]["cost"] if d["rrt"] else float("nan")
        ax.set_title(f"{n}\nA* {a:.2f}   RRT* {r:.2f}", fontsize=11.5,
                     color=INK, fontweight="bold", pad=14, linespacing=1.7)

    handles = [
        Line2D([], [], color=C_ASTAR, lw=2.6, label="A*"),
        Line2D([], [], color=C_RRT, lw=2.0, label=f"RRT* (최선 / {N_SEEDS}시드)"),
        Line2D([], [], color=INK2, lw=2.0, linestyle="-", label="지상 주행"),
        Line2D([], [], color=INK2, lw=2.0, linestyle=(0, (2.2, 1.6)), label="비행"),
    ]
    fig.legend(handles=handles, loc="lower center", ncol=4, frameon=False,
               fontsize=11, labelcolor=INK2, handlelength=2.4,
               bbox_to_anchor=(0.5, 0.005))

    # suptitle 대신 fig.text 로 — 3D 축은 subplots_adjust 를 잘 안 따라서
    # suptitle 과 부제가 겹친다. 좌표를 직접 준다.
    fig.text(0.008, 1.035, "A* vs RRT* — 지형 높이를 포함한 3D 경로",
             fontsize=15, color=INK, fontweight="bold")
    fig.text(0.008, 0.995,
             f"벽(3.0 m)은 {WALL_CLIP} m 에서 잘라 그렸고 높이축은 과장했다 — "
             f"실제 축척이면 0.35~1.4 m 장애물이 바닥에 붙어 보인다.",
             fontsize=9.5, color=INK3)
    fig.text(0.008, 0.963,
             "경로가 전부 지형에 붙어 있다 = 비행이 한 번도 선택되지 않았다는 뜻이다.",
             fontsize=9.5, color="#9c3a29")

    out = RESULTS / "fig_paths_3d.png"
    fig.patch.set_facecolor(SURFACE)
    fig.savefig(out, dpi=175, bbox_inches="tight", facecolor=SURFACE)
    plt.close(fig)
    print(f"\n저장: {out.relative_to(HERE.parent)}")


if __name__ == "__main__":
    main(sys.argv[1:] or DEFAULT_MAPS)
