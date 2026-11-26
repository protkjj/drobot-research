# Copyright 2026 leo11dk
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
월드 SDF 의 <world name> 이 파일 이름과 같아야 한다.

mode_manager 는 world 파라미터(= 파일 이름, RUN.md 의 -p world:=...)로
/world/<이름>/set_pose 를 부른다. 이름이 다르면 그 서비스가 없어 자세 전송이
'응답 없음' 으로 실패하고, 비행 구간이 매번 실패해 변신만 반복한다.
2026-11-20 확인 — nogap·narrow 는 base_map_h0.5 를 복사하며 이름이 그대로 남아 있었다.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


def _repo_root() -> Path:
    for start in (Path(__file__).resolve(), Path(__file__).absolute()):
        for p in start.parents:
            if (p / 'src' / 'drobot_description' / 'worlds').is_dir():
                return p
    raise RuntimeError('저장소 루트를 찾지 못했다')


WORLD_DIR = _repo_root() / 'src/drobot_description/worlds'
WORLDS = sorted(list(WORLD_DIR.glob('*.sdf')) + list((WORLD_DIR / 'benchmark').glob('*.sdf')))


@pytest.mark.parametrize('sdf', WORLDS, ids=[w.stem for w in WORLDS])
def test_world_name_matches_file_name(sdf):
    world = ET.parse(sdf).getroot().find('world')
    assert world is not None, f'{sdf.name}: <world> 가 없다'
    assert world.get('name') == sdf.stem, (
        f'{sdf.name}: <world name="{world.get("name")}"> — 파일 이름 {sdf.stem!r} 과 달라 '
        f'mode_manager 의 /world/{sdf.stem}/set_pose 가 없다')
