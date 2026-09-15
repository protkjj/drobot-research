"""맵 6종 시각화 — 높이맵을 눈으로 확인한다.

색 구분은 elevation_params.yaml 의 4단계 분류 + 1단계 검증 결과를 따른다.
    회백    : 로버 통과 가능 (h <= 0.15)
    연파랑  : 비행 필요, 3D로도 가능 (0.15 < h <= 0.95)
    주황    : 비행 필요, 4D만 가능 (0.95 < h <= 1.55)   <- 3D 형식의 사각지대
    검정    : 통과 불가 (벽)
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plotstyle  # noqa: E402,F401  (import 시점에 한글 폰트 설정)

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import BoundaryNorm, ListedColormap  # noqa: E402
import numpy as np  # noqa: E402

from envs.heightmap import build_all, ROVER_MAX, H_3D_LIMIT, H_4D_LIMIT  # noqa: E402

OUT = Path(__file__).resolve().parent / "results" / "maps.png"
OUT.parent.mkdir(parents=True, exist_ok=True)

maps = build_all(res=0.1)

# 4단계 분류용 컬러맵
cmap = ListedColormap(["#e8e8e6", "#a8cfe0", "#e8933a", "#2b2b2b"])
bounds = [-1e-9, ROVER_MAX, H_3D_LIMIT, H_4D_LIMIT, 99.0]
norm = BoundaryNorm(bounds, cmap.N)

fig, axes = plt.subplots(3, 2, figsize=(13, 14))
order = ["easy_open", "easy_corridor",
         "medium_open", "medium_corridor",
         "hard_open", "hard_corridor"]

for ax, name in zip(axes.flat, order):
    hm = maps[name]
    ax.imshow(hm.grid, origin="lower", cmap=cmap, norm=norm,
              extent=[0, hm.width_m, 0, hm.height_m], interpolation="nearest")

    # 장애물 높이를 숫자로 표기 (넘어갈 수 있는 것만)
    g = hm.grid
    obst = (g > ROVER_MAX) & (g <= H_4D_LIMIT)
    seen = np.zeros_like(obst, dtype=bool)
    from collections import deque
    ys, xs = np.nonzero(obst)
    for y0, x0 in zip(ys, xs):
        if seen[y0, x0]:
            continue
        h0 = round(float(g[y0, x0]), 3)
        comp, q = [], deque([(x0, y0)])
        seen[y0, x0] = True
        while q:
            x, y = q.popleft()
            comp.append((x, y))
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx_, ny_ = x + dx, y + dy
                if (hm.in_bounds(nx_, ny_) and not seen[ny_, nx_] and obst[ny_, nx_]
                        and abs(float(g[ny_, nx_]) - h0) < 1e-6):
                    seen[ny_, nx_] = True
                    q.append((nx_, ny_))
        if len(comp) < 20:      # 너무 작은 조각은 라벨 생략
            continue
        cx = np.mean([c[0] for c in comp]) * hm.resolution
        cy = np.mean([c[1] for c in comp]) * hm.resolution
        ax.text(cx, cy, f"{h0:.2f}", ha="center", va="center",
                fontsize=7.5, color="white" if h0 > H_3D_LIMIT else "#123",
                fontweight="bold")

    ax.plot(*hm.start, "o", color="#1a7f37", markersize=11,
            markeredgecolor="white", markeredgewidth=1.5, zorder=5)
    ax.plot(*hm.goal, "*", color="#c33", markersize=18,
            markeredgecolor="white", markeredgewidth=1.0, zorder=5)

    s = hm.stats()
    ax.set_title(f"{name}   ({s['size_m']} m, {s['cells']:,} cells)\n"
                 f"{hm.description}", fontsize=9.5)
    ax.set_xlabel("x [m]", fontsize=8)
    ax.set_ylabel("y [m]", fontsize=8)
    ax.tick_params(labelsize=7)

handles = [
    plt.Rectangle((0, 0), 1, 1, fc="#e8e8e6", ec="#999"),
    plt.Rectangle((0, 0), 1, 1, fc="#a8cfe0", ec="#999"),
    plt.Rectangle((0, 0), 1, 1, fc="#e8933a", ec="#999"),
    plt.Rectangle((0, 0), 1, 1, fc="#2b2b2b", ec="#999"),
    plt.Line2D([], [], marker="o", color="#1a7f37", ls="", markersize=9),
    plt.Line2D([], [], marker="*", color="#c33", ls="", markersize=13),
]
labels = [
    f"로버 통과 (h ≤ {ROVER_MAX})",
    f"비행 필요, 3D 가능 ({ROVER_MAX} < h ≤ {H_3D_LIMIT})",
    f"4D만 가능 ({H_3D_LIMIT} < h ≤ {H_4D_LIMIT})",
    f"통과 불가 (h > {H_4D_LIMIT})",
    "start", "goal",
]
fig.legend(handles, labels, loc="lower center", ncol=3, fontsize=9,
           frameon=False, bbox_to_anchor=(0.5, -0.005))
fig.suptitle("A* vs RRT* 벤치마크 실험 맵 6종", fontsize=13, y=0.997)
fig.tight_layout(rect=[0, 0.045, 1, 0.985])
fig.savefig(OUT, dpi=135, bbox_inches="tight")
print(f"저장: {OUT}")
