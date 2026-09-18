"""높이맵 환경.

실험 맵을 '각 셀의 지형 높이(m)'를 담은 2D 배열로 표현한다.
Gazebo 월드를 쓰지 않고 이렇게 하는 이유:
  - 순수 알고리즘 비교가 목적이므로 물리 시뮬레이션이 불필요하다
  - 결정론적이라 재현이 보장된다 (같은 코드 -> 같은 맵)
  - macOS에서 바로 돌아간다

높이 값의 의미 (elevation_params.yaml 의 4단계 분류와 대응)
    h <= ROVER_MAX (0.15)   : 로버가 그냥 지나감
    h <= H_3D_LIMIT (0.95)  : 비행으로 넘을 수 있음 (3D 형식으로도 가능)
    h <= H_4D_LIMIT (1.55)  : 4D 형식으로만 넘을 수 있음
    그 이상                  : 통과 불가 (벽)

---------------------------------------------------------------------------
설계 근거 (analyze_breakeven.py 결과)

1차 설계는 실패했다. Dijkstra 최적해가 6개 맵 전부에서 비행을 0회 선택했다.
원인: 비행 1회의 고정비(이착륙)가 지상 주행 6.4m에 해당하는데,
      장애물이 폭 1m 남짓이라 우회거리가 2~3m밖에 안 됐다.

손익분기 우회거리 (비행이 이기려면 이만큼은 돌아야 함)
    폭 0.6m 높이 0.60m -> 8.2m
    폭 0.6m 높이 1.10m -> 9.1m
    폭 1.0m 높이 0.80m -> 9.1m

그래서 2차 설계는 '좁고 긴 벽' 구조로 간다.
    - 폭 0.6m  : 좁아야 비행 비용이 작다 (수평 비행 거리 = 벽 폭)
    - 길이 가변 : 길수록 우회 비용이 크다

그리고 벽마다 길이를 다르게 해서
    어떤 벽은 넘는 게 이득 / 어떤 벽은 도는 게 이득
이 되도록 만든다. 전부 넘어야만 하거나 전부 돌아야만 하면
'선택'이 없어서 에너지 인식 플래너의 가치를 보여줄 수 없다.
---------------------------------------------------------------------------
"""
from __future__ import annotations

from dataclasses import dataclass

from pathlib import Path

import numpy as np

# 높이 상수 — verify_step1b.py 결과에 근거
WALL_HEIGHT = 3.0        # 천장까지 = 절대 통과 불가
ROVER_MAX = 0.15         # elevation_params.yaml: rover_traversable_max
H_3D_LIMIT = 0.95        # 3D 형식(고도=h+0.8 고정)의 통과 가능 상한
H_4D_LIMIT = 1.55        # 4D 형식(고도 자유)의 통과 가능 상한

WALL_W = 0.6             # 넘어가는 벽의 표준 폭 (좁을수록 비행에 유리)


@dataclass
class HeightMap:
    """지형 높이 격자.

    grid[iy, ix] = 해당 셀의 지형 높이 (m)
    실좌표 (x, y) -> 인덱스: ix = int(x / res), iy = int(y / res)
    """

    grid: np.ndarray            # (ny, nx) float, 단위 m
    resolution: float           # m/cell
    name: str
    start: tuple[float, float]  # (x, y) 실좌표
    goal: tuple[float, float]
    ceiling: float = 2.5
    description: str = ""

    @property
    def ny(self) -> int:
        return self.grid.shape[0]

    @property
    def nx(self) -> int:
        return self.grid.shape[1]

    @property
    def width_m(self) -> float:
        return self.nx * self.resolution

    @property
    def height_m(self) -> float:
        return self.ny * self.resolution

    def to_idx(self, x: float, y: float) -> tuple[int, int]:
        return int(x / self.resolution), int(y / self.resolution)

    def to_xy(self, ix: int, iy: int) -> tuple[float, float]:
        """셀 중심의 실좌표."""
        return (ix + 0.5) * self.resolution, (iy + 0.5) * self.resolution

    def height_at(self, ix: int, iy: int) -> float:
        return float(self.grid[iy, ix])

    def in_bounds(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    def stats(self) -> dict:
        g = self.grid
        total = g.size
        return {
            "name": self.name,
            "shape": f"{self.nx}x{self.ny}",
            "size_m": f"{self.width_m:.0f}x{self.height_m:.0f}",
            "cells": total,
            "rover_ok_pct": 100.0 * np.sum(g <= ROVER_MAX) / total,
            "flyable_3d_pct": 100.0 * np.sum((g > ROVER_MAX) & (g <= H_3D_LIMIT)) / total,
            "only_4d_pct": 100.0 * np.sum((g > H_3D_LIMIT) & (g <= H_4D_LIMIT)) / total,
            "wall_pct": 100.0 * np.sum(g > H_4D_LIMIT) / total,
            "h_max": float(g.max()),
        }


# ---------------------------------------------------------------------------
# 맵 생성 헬퍼
# ---------------------------------------------------------------------------
def _blank(nx: int, ny: int) -> np.ndarray:
    return np.zeros((ny, nx), dtype=float)


def _rect(g: np.ndarray, x0: float, y0: float, x1: float, y1: float,
          h: float, res: float) -> None:
    """실좌표 직사각형 영역을 높이 h로 채운다 (in-place)."""
    ix0, iy0 = int(round(x0 / res)), int(round(y0 / res))
    ix1, iy1 = int(round(x1 / res)), int(round(y1 / res))
    ix0, iy0 = max(0, ix0), max(0, iy0)
    ix1, iy1 = min(g.shape[1], ix1), min(g.shape[0], iy1)
    g[iy0:iy1, ix0:ix1] = h


def _border_walls(g: np.ndarray, res: float, thickness: float = 0.3) -> None:
    """맵 외곽에 벽을 두른다."""
    t = max(1, int(thickness / res))
    g[:t, :] = WALL_HEIGHT
    g[-t:, :] = WALL_HEIGHT
    g[:, :t] = WALL_HEIGHT
    g[:, -t:] = WALL_HEIGHT


def _vwall_from_bottom(g, x, length, h, res, H, width=WALL_W):
    """맵 바닥(y=0)에서 위로 length 만큼 올라오는 세로 벽.

    위쪽에 (H - length) 만큼의 틈이 남는다 -> 우회하려면 그리로 돌아야 한다.
    """
    _rect(g, x, 0.0, x + width, length, h, res)


def _vwall_from_top(g, x, length, h, res, H, width=WALL_W):
    """맵 천장(y=H)에서 아래로 length 만큼 내려오는 세로 벽."""
    _rect(g, x, H - length, x + width, H, h, res)


# ---------------------------------------------------------------------------
# 맵 6종
# ---------------------------------------------------------------------------
# 공통 구조: start는 왼쪽 중앙, goal은 오른쪽 중앙. 세로 벽들을 가로질러 간다.
# 벽마다 '남은 틈'의 위치와 크기를 다르게 해서 우회거리를 조절한다.
#
# 우회거리 어림셈: 직선 경로(y=중앙)에서 틈까지 갔다 돌아오는 거리
#   ≈ 2 * |중앙 y - 틈 중심 y|
# 이 값이 손익분기(8~10m)보다 크면 비행이 이득, 작으면 우회가 이득.

def make_easy_open(res: float = 0.1) -> HeightMap:
    """넓은 공간. 벽 2개, 하나는 넘는 게 이득 / 하나는 도는 게 이득."""
    W, H = 20.0, 14.0
    g = _blank(int(W / res), int(H / res))
    _border_walls(g, res)
    mid = H / 2

    # 벽A: 아래에서 12m -> 틈은 y=12~14 (중심 13). 우회 ≈ 2*(13-7) = 12m > 8.2
    #      -> 넘는 게 이득
    _vwall_from_bottom(g, 6.0, 12.0, 0.55, res, H)
    # 벽B: 아래에서 8.5m -> 틈은 y=8.5~14 (중심 11.2). 우회 ≈ 2*(11.2-7) = 8.4m
    #      손익분기 언저리 -> 진짜 고민되는 케이스
    _vwall_from_bottom(g, 13.0, 8.5, 0.35, res, H)

    return HeightMap(
        grid=g, resolution=res, name="easy_open",
        start=(1.5, mid), goal=(18.5, mid),
        description="벽 2개(h=0.55/0.35), 우회 12m/8.4m — 넘기 vs 돌기 선택",
    )


def make_easy_corridor(res: float = 0.1) -> HeightMap:
    """복도 구조. 우회 통로가 좁고 멀어 비행 유인이 크다."""
    W, H = 20.0, 14.0
    g = _blank(int(W / res), int(H / res))
    _border_walls(g, res)
    mid = H / 2

    # 통과 불가한 격벽으로 복도를 만든다 (여기는 넘을 수 없음)
    _rect(g, 4.0, 0.0, 4.5, 9.5, WALL_HEIGHT, res)     # 틈: y=9.5~14
    _rect(g, 16.0, 4.5, 16.5, 14.0, WALL_HEIGHT, res)  # 틈: y=0~4.5

    # 넘을 수 있는 낮은 벽 — 복도 안쪽에 배치
    # 벽A: 아래에서 11m, 틈 중심 y=12.5 -> 우회 ≈ 11m > 손익분기
    _vwall_from_bottom(g, 9.0, 11.0, 0.50, res, H)
    # 벽B: 위에서 6m, 틈 중심 y=4 -> 우회 ≈ 6m < 손익분기 -> 도는 게 이득
    _vwall_from_top(g, 12.0, 6.0, 0.70, res, H)

    return HeightMap(
        grid=g, resolution=res, name="easy_corridor",
        start=(1.5, mid), goal=(18.5, 2.0),
        description="격벽 복도 + 낮은 벽 2개, 우회 11m/6m",
    )


def make_medium_open(res: float = 0.1) -> HeightMap:
    """벽 3개, 3D 한계(0.95m)를 넘는 높이 포함."""
    W, H = 22.0, 14.0
    g = _blank(int(W / res), int(H / res))
    _border_walls(g, res)
    mid = H / 2

    # 벽A: 우회 12m, 높이 0.55 (3D 가능) -> 넘는 게 이득
    _vwall_from_bottom(g, 5.0, 12.0, 0.55, res, H)
    # 벽B: 우회 11m, 높이 1.10 (4D 전용) -> 3D는 못 넘어서 돌아야 함
    _vwall_from_top(g, 11.0, 11.5, 1.10, res, H)
    # 벽C: 우회 9m, 높이 0.80 (3D 가능) -> 손익분기 근처
    _vwall_from_bottom(g, 17.0, 10.5, 0.80, res, H)

    return HeightMap(
        grid=g, resolution=res, name="medium_open",
        start=(1.5, mid), goal=(20.5, mid),
        description="벽 3개(0.55/1.10/0.80), 1.10은 4D만 통과 가능",
    )


def make_medium_corridor(res: float = 0.1) -> HeightMap:
    """복도 + 4D 전용 높이. 3D는 우회를 강요당한다."""
    W, H = 22.0, 14.0
    g = _blank(int(W / res), int(H / res))
    _border_walls(g, res)
    mid = H / 2

    # 격벽
    _rect(g, 7.5, 0.0, 8.0, 10.0, WALL_HEIGHT, res)
    _rect(g, 15.0, 4.0, 15.5, 14.0, WALL_HEIGHT, res)

    # 넘을 수 있는 벽들 — 한 비행구간(5m) 안에 높이가 다른 것이 오도록
    _vwall_from_bottom(g, 4.0, 11.5, 0.60, res, H)     # 우회 ≈ 11m
    _vwall_from_bottom(g, 4.8, 11.5, 1.05, res, H)     # 바로 옆, 4D 전용
    _vwall_from_top(g, 11.5, 11.0, 0.85, res, H)       # 우회 ≈ 10m
    _vwall_from_bottom(g, 19.0, 9.0, 1.25, res, H)     # 4D 전용, 우회 ≈ 5m

    return HeightMap(
        grid=g, resolution=res, name="medium_corridor",
        start=(1.5, mid), goal=(20.5, 2.0),
        description="격벽 + 벽 4개, 인접 벽 높이차(0.60/1.05)로 고도 계획 필요",
    )


def make_hard_open(res: float = 0.1) -> HeightMap:
    """벽 5개, 높이 다양. 여러 번의 넘기/돌기 판단이 연쇄된다."""
    W, H = 24.0, 16.0
    g = _blank(int(W / res), int(H / res))
    _border_walls(g, res)
    mid = H / 2

    # 벽마다 우회거리와 높이를 다르게 -> 매번 다른 판단
    _vwall_from_bottom(g, 4.0, 13.5, 0.45, res, H)   # 우회 ~13m, 낮음 -> 넘기
    _vwall_from_bottom(g, 4.8, 13.5, 1.15, res, H)   # 인접, 4D 전용 (높이차 0.70)
    _vwall_from_top(g, 9.5, 13.0, 0.70, res, H)      # 우회 ~12m -> 넘기
    _vwall_from_bottom(g, 14.0, 6.0, 1.40, res, H)   # 우회 ~4m, 높음 -> 돌기
    _vwall_from_top(g, 18.5, 12.5, 0.60, res, H)     # 우회 ~11m -> 넘기
    _vwall_from_top(g, 19.3, 12.5, 1.30, res, H)     # 인접, 4D 전용 (높이차 0.70)

    return HeightMap(
        grid=g, resolution=res, name="hard_open",
        start=(1.5, mid), goal=(22.5, mid),
        description="벽 6개, 인접 벽 높이차 0.70m, 넘기/돌기 판단 연쇄",
    )


def make_hard_corridor(res: float = 0.1) -> HeightMap:
    """우회로 없음 — 지상 전용은 반드시 실패한다 (baseline 실패 케이스)."""
    W, H = 24.0, 16.0
    g = _blank(int(W / res), int(H / res))
    _border_walls(g, res)
    mid = H / 2

    # 맵을 완전히 가로지르는 격벽 3개. 통로는 '넘어야만 지날 수 있는' 낮은 구간.
    for x, h_gap, gap_y0, gap_y1 in [
        (5.0, 1.10, 6.5, 8.5),    # 4D 전용 높이 -> 3D는 여기서 막힘
        (12.0, 0.75, 9.0, 11.0),  # 3D도 가능
        (19.0, 1.35, 4.0, 6.0),   # 4D 전용
    ]:
        _rect(g, x, 0.0, x + WALL_W, H, WALL_HEIGHT, res)      # 전면 격벽
        _rect(g, x, gap_y0, x + WALL_W, gap_y1, h_gap, res)    # 통로만 낮게

    # 통로 앞뒤에 높이가 다른 장애물 -> 한 비행 안에서 고도 변화 필요
    _rect(g, 4.2, 6.5, 5.0, 8.5, 0.40, res)
    _rect(g, 5.6, 6.5, 6.4, 8.5, 0.65, res)
    _rect(g, 18.2, 4.0, 19.0, 6.0, 0.55, res)
    _rect(g, 19.6, 4.0, 20.4, 6.0, 0.90, res)

    return HeightMap(
        grid=g, resolution=res, name="hard_corridor",
        start=(1.5, mid), goal=(22.5, mid),
        description="격벽 3개, 통로가 전부 장애물 -> 비행 필수 (지상 전용 실패)",
    )


ALL_MAPS = {
    "easy_open": make_easy_open,
    "easy_corridor": make_easy_corridor,
    "medium_open": make_medium_open,
    "medium_corridor": make_medium_corridor,
    "hard_open": make_hard_open,
    "hard_corridor": make_hard_corridor,
}


def build_all(res: float = 0.1) -> dict[str, HeightMap]:
    return {k: f(res) for k, f in ALL_MAPS.items()}


# ---------------------------------------------------------------------------
# 독립연구 맵 (base_map) — 제안서 4.1 의 통제 실험 환경
#
# 위 make_* 함수들이 만드는 벤치마크 맵과 목적이 다르다.
#   벤치마크 맵   긴 벽 여러 개, 우회거리를 손익분기 근처로 맞춰 '선택'을 만든다
#   base_map     6x11m 방에 단일 장애물, 높이만 독립변수로 바꾼다
#                (0.05 / 0.3 / 0.5 / 1.0 / 1.8 / 2.5 m — 4단계 분류 경계를 관통)
#
# 이 맵들은 Gazebo SDF 와 .npz 로 이중 표현돼 있어 시뮬레이션과 알고리즘 검증이
# 같은 형상을 쓴다. 여기서는 .npz 를 그대로 읽는다 — 형상을 코드로 다시 만들면
# SDF 와 어긋날 수 있기 때문이다.
# ---------------------------------------------------------------------------
_BASE_MAP_DIR = Path(__file__).resolve().parent / "base_maps"


def load_base_map(name: str) -> HeightMap:
    """base_map_h*.npz 를 HeightMap 으로 읽는다.

    name 은 'base_map_h0.5' 또는 'h0.5' 또는 '0.5' 모두 받는다.
    """
    stem = name if name.startswith("base_map_") else f"base_map_h{name.lstrip('h')}"
    f = _BASE_MAP_DIR / f"{stem}.npz"
    if not f.exists():
        avail = sorted(p.stem for p in _BASE_MAP_DIR.glob("*.npz"))
        raise FileNotFoundError(f"{f} 없음. 사용 가능: {avail}")

    d = np.load(f)
    # npz 는 (ny, nx) 순서로 저장돼 있다 — HeightMap.grid 와 같은 규약
    grid = np.asarray(d["height_map"], dtype=float)
    res = float(d["resolution"])
    sx, sy = d["start_m"], d["goal_m"]
    h = float(stem.split("_h")[1])
    return HeightMap(
        grid=grid,
        resolution=res,
        name=stem,
        start=(float(sx[0]), float(sx[1])),
        goal=(float(sy[0]), float(sy[1])),
        description=f"독립연구 통제 맵 6x11m, 단일 장애물 높이 {h} m",
    )


def base_map_names() -> list[str]:
    """사용 가능한 base_map 이름을 높이 오름차순으로."""
    names = [p.stem for p in _BASE_MAP_DIR.glob("base_map_h*.npz")]
    return sorted(names, key=lambda s: float(s.split("_h")[1]))


def build_base_maps() -> dict[str, HeightMap]:
    """base_map 전부를 높이 오름차순으로."""
    return {n: load_base_map(n) for n in base_map_names()}
