#!/usr/bin/env python3
"""Gazebo 월드(SDF)에서 prior map(.heightmap)을 만든다.

왜 필요한가
    prior map 은 ElevationLayer 가 시작할 때 심는 '정답 높이맵' 이다
    (use_prior_map:=true). 카메라가 지도의 4.8 % 만 보는 상황에서 플래너가 장애물
    높이를 알게 하려는 것이다. 이 연구는 '지도가 주어진 조건' 이 전제다.

    예전 파일은 2026-10-05 에 만든 도구가 커밋되지 않아 다시 만들 수 없었다. 그 도구는
    상자가 겹칠 때 나중에 그린 상자로 덮어써서, nogap·narrow 에서 0.5 m 상자가 양옆
    벽(3.0 m)을 덮은 칸 240·120 개가 0.5 m 로 적혀 있었다 — prior map 상 벽에 구멍.
    prior 칸은 '카메라가 잰 칸' 으로 치므로 ElevationLayer 가 그 벽 칸의 LiDAR 254 를
    200 으로 내렸다. 여기서는 겹치면 최댓값을 쓴다.

    원천은 SDF 다 — Gazebo 가 실제로 시뮬레이션하는 형상이기 때문이다. base_map 들의
    예전 파일은 test_maps.py 의 .npz 배열에서 왔는데, 같은 도구가 낸 SDF 와 위쪽 벽이
    한 줄(5 cm) 어긋나 있었다.

규칙
    - <collision> 의 <box> 만 쓴다. <visual> 만 있는 것(출발·목표 표시 원판)은 뺀다.
      바닥 <plane> 도 뺀다.
    - 칸 중심이 상자 안(경계 포함)이면 그 칸 높이 = 상자 윗면. 겹치면 최댓값.
    - 회전은 yaw 만 지원한다. roll·pitch 가 있으면 멈춘다 (지금 월드에는 없다).
    - 격자 범위는 상자 전체를 덮는 최소 직사각형을 해상도에 맞춰 넓힌 것.

형식 (ElevationLayer::loadPriorMap 이 읽는다)
    # 로 시작하는 줄은 주석
    가로칸 세로칸 해상도 원점x 원점y
    높이(m) ... (세로칸 줄, y 가 작은 줄부터)

사용
    python3 tools/sdf_to_heightmap.py                 # 모든 월드 → src/drobot_description/maps/
    python3 tools/sdf_to_heightmap.py --check         # 저장된 파일과 같은지만 본다 (다르면 1)
    python3 tools/sdf_to_heightmap.py worlds/x.sdf    # 하나만
"""
from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DESC = REPO / "src/drobot_description"
WORLD_DIRS = [DESC / "worlds", DESC / "worlds/benchmark"]
MAP_DIR = DESC / "maps"
RES = 0.05


@dataclass
class Box:
    x: float
    y: float
    yaw: float
    sx: float
    sy: float
    top: float

    def contains(self, px: float, py: float) -> bool:
        dx, dy = px - self.x, py - self.y
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        lx, ly = c * dx - s * dy, s * dx + c * dy
        eps = 1e-9
        return abs(lx) <= self.sx / 2 + eps and abs(ly) <= self.sy / 2 + eps

    def corners(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        for ex, ey in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            lx, ly = ex * self.sx / 2, ey * self.sy / 2
            yield self.x + c * lx - s * ly, self.y + s * lx + c * ly


def _pose(el) -> list[float]:
    p = el.findtext("pose") if el is not None else None
    v = [float(t) for t in (p or "0 0 0 0 0 0").split()]
    return v + [0.0] * (6 - len(v))


def _compose(a: list[float], b: list[float]) -> list[float]:
    """a 좌표계 안의 b 를 바깥으로 (yaw 만)."""
    c, s = math.cos(a[5]), math.sin(a[5])
    return [a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2],
            0.0, 0.0, a[5] + b[5]]


def read_boxes(sdf: Path) -> list[Box]:
    boxes = []
    for model in ET.parse(sdf).getroot().iter("model"):
        mp = _pose(model)
        for link in model.iter("link"):
            lp = _compose(mp, _pose(link))
            for col in link.iter("collision"):
                size = col.findtext("geometry/box/size")
                if size is None:
                    continue
                cp = _compose(lp, _pose(col))
                for el, p in ((model, mp), (link, _pose(link)), (col, _pose(col))):
                    if abs(p[3]) > 1e-9 or abs(p[4]) > 1e-9:
                        raise SystemExit(f"{sdf.name}: roll/pitch 가 있는 pose 는 지원하지 않는다 "
                                         f"({el.tag} {el.get('name')})")
                sx, sy, sz = (float(t) for t in size.split())
                boxes.append(Box(cp[0], cp[1], cp[5], sx, sy, cp[2] + sz / 2))
    return boxes


def rasterize(boxes: list[Box], res: float = RES):
    xs = [x for b in boxes for x, _ in b.corners()]
    ys = [y for b in boxes for _, y in b.corners()]
    ox = math.floor(min(xs) / res + 1e-9) * res
    oy = math.floor(min(ys) / res + 1e-9) * res
    w = math.ceil((max(xs) - ox) / res - 1e-9)
    h = math.ceil((max(ys) - oy) / res - 1e-9)
    grid = [[0.0] * w for _ in range(h)]
    for b in boxes:
        bx = [x for x, _ in b.corners()]
        by = [y for _, y in b.corners()]
        i0 = max(0, math.floor((min(bx) - ox) / res) - 1)
        i1 = min(w - 1, math.ceil((max(bx) - ox) / res) + 1)
        j0 = max(0, math.floor((min(by) - oy) / res) - 1)
        j1 = min(h - 1, math.ceil((max(by) - oy) / res) + 1)
        for j in range(j0, j1 + 1):
            cy = oy + (j + 0.5) * res
            for i in range(i0, i1 + 1):
                if b.contains(ox + (i + 0.5) * res, cy):
                    grid[j][i] = max(grid[j][i], b.top)
    return w, h, res, ox, oy, grid


def render(sdf: Path, raster) -> str:
    w, h, res, ox, oy, grid = raster
    rel = sdf.relative_to(DESC) if sdf.is_relative_to(DESC) else sdf.name
    lines = [
        "# drobot heightmap v1",
        f"# source: {rel} — tools/sdf_to_heightmap.py (충돌 상자 윗면, 겹치면 최댓값)",
        f"{w} {h} {res:.6f} {ox:.6f} {oy:.6f}",
    ]
    lines += [" ".join(f"{v:.4f}" for v in row) for row in grid]
    return "\n".join(lines) + "\n"


def read_heightmap(path: Path):
    rows = [ln for ln in path.read_text().splitlines() if ln.strip() and not ln.startswith("#")]
    w, h = (int(t) for t in rows[0].split()[:2])
    return [[float(v) for v in ln.split()] for ln in rows[1:1 + h]], w, h


def all_worlds() -> list[Path]:
    return sorted(p for d in WORLD_DIRS for p in d.glob("*.sdf"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("worlds", nargs="*", type=Path, help="SDF 파일 (기본: 모든 월드)")
    ap.add_argument("--check", action="store_true", help="저장된 .heightmap 과 비교만")
    args = ap.parse_args()

    bad = 0
    for sdf in args.worlds or all_worlds():
        sdf = sdf.resolve()
        out = MAP_DIR / f"{sdf.stem}.heightmap"
        boxes = read_boxes(sdf)
        raster = rasterize(boxes)
        if args.check:
            if not out.exists():
                print(f"없음   {out.name}")
                bad += 1
                continue
            old, w, h = read_heightmap(out)
            new = raster[5]
            if (w, h) != (raster[0], raster[1]):
                print(f"크기 다름 {out.name}: {w}x{h} vs {raster[0]}x{raster[1]}")
                bad += 1
                continue
            n = sum(abs(a - b) > 1e-3 for ro, rn in zip(old, new) for a, b in zip(ro, rn))
            print(f"{'같음' if n == 0 else '다름'}   {out.name}" + (f" ({n} 칸)" if n else ""))
            bad += n > 0
        else:
            out.write_text(render(sdf, raster))
            print(f"썼다   {out.name}  ({raster[0]}x{raster[1]}, 상자 {len(boxes)} 개)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
