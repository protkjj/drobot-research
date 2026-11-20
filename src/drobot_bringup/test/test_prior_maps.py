# Copyright 2026 leo11dk
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
prior map(.heightmap)이 월드(SDF)와 같은지 검사한다.

prior map 은 ElevationLayer 가 시작할 때 '카메라가 잰 높이' 로 심는다. 월드와
어긋나면 그대로 지도가 된다 — 2026-10-05 에 만든 nogap·narrow 맵은 상자가 벽을
덮어써 벽에 0.5 m 구멍이 있었고, prior 칸이라 LiDAR 의 254 까지 200 으로 내려갔다.
월드를 바꾸면 tools/sdf_to_heightmap.py 로 다시 만들 것.
"""

import importlib.util
from pathlib import Path
import sys

import pytest


def _repo_root() -> Path:
    for start in (Path(__file__).resolve(), Path(__file__).absolute()):
        for p in start.parents:
            if (p / 'tools' / 'sdf_to_heightmap.py').is_file():
                return p
    raise RuntimeError('저장소 루트를 찾지 못했다')


ROOT = _repo_root()
_spec = importlib.util.spec_from_file_location(
    'sdf_to_heightmap', ROOT / 'tools/sdf_to_heightmap.py')
gen = importlib.util.module_from_spec(_spec)
sys.modules['sdf_to_heightmap'] = gen   # dataclass 가 모듈을 찾는다
_spec.loader.exec_module(gen)

WORLDS = gen.all_worlds()


def test_every_world_has_a_prior_map():
    missing = [w.stem for w in WORLDS if not (gen.MAP_DIR / f'{w.stem}.heightmap').exists()]
    assert not missing, f'prior map 이 없는 월드: {missing} — tools/sdf_to_heightmap.py 로 만들 것'


@pytest.mark.parametrize('world', WORLDS, ids=[w.stem for w in WORLDS])
def test_prior_map_matches_world(world):
    w, h, res, ox, oy, grid = gen.rasterize(gen.read_boxes(world))
    old, ow, oh = gen.read_heightmap(gen.MAP_DIR / f'{world.stem}.heightmap')
    assert (ow, oh) == (w, h), f'격자 크기 {ow}x{oh} != 월드 {w}x{h}'
    bad = [(i, j, old[j][i], grid[j][i])
           for j in range(h) for i in range(w) if abs(old[j][i] - grid[j][i]) > 1e-3]
    assert not bad, (f'{len(bad)} 칸이 월드와 다르다 (예: 칸, 파일값, 월드값 {bad[:3]}) '
                     '— tools/sdf_to_heightmap.py 로 다시 만들 것')


def test_overlapping_boxes_keep_the_taller_one():
    """벽(3.0 m)과 겹친 상자(0.5 m)가 벽을 덮어쓰면 안 된다 — 예전 nogap 맵의 구멍."""
    wall = gen.Box(x=0.05, y=5.5, yaw=0.0, sx=0.1, sy=11.0, top=3.0)
    box = gen.Box(x=3.0, y=5.5, yaw=0.0, sx=6.0, sy=3.0, top=0.5)
    for order in ([wall, box], [box, wall]):
        _, _, res, ox, oy, grid = gen.rasterize(order)
        i = int((0.05 - ox) / res)
        j = int((5.5 - oy) / res)
        assert grid[j][i] == 3.0
