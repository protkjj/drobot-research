# Copyright 2026 leo11dk
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
같은 값을 여러 파일에 복사해 둔 곳이 서로 어긋나지 않았는지 검사한다.

왜 필요한가
    런타임(Gazebo)은 nav2_params 의 인라인 값만 읽는다 — launch 는 패키지
    config 를 하나도 읽지 않는다. 오프라인 벤치마크는 패키지 config
    (energy_params*.yaml) 를 읽는다. 이 연구는 두 결과를 비교하므로 두 쪽이
    같은 숫자를 읽어야 하는데, 지금은 손으로 맞춰 둔 것뿐이다.

    실제로 이미 두 번 어긋났다 (2026-11-20 발견).
      - fly_over_max: elevation_params.yaml 2.0, nav2_params 1.2
      - 지역 fly_over: default 파일 200, derived 파일 254 — 에너지 세트를
        바꾸면 지역 지도까지 같이 바뀌어 default↔derived 비교가 오염됐다

    원천을 하나로 합치기 전까지(documents/REQUIREMENTS_TRACE.md D-5) 어긋나면
    여기서 실패하게 한다.

의도된 차이
    ALLOWED_ELEVATION_OVERRIDES 에 이유와 함께 적는다. 거기 없는 차이는 실수로 본다.
"""

from pathlib import Path

import pytest
import yaml


def _repo_root() -> Path:
    # symlink-install 이면 이 파일은 소스 트리를 가리키고, 아니면 build/ 아래에
    # 복사본이 있다. 어느 쪽이든 위로 올라가며 src/drobot_bringup 이 있는 곳을 찾는다.
    for start in (Path(__file__).resolve(), Path(__file__).absolute()):
        for p in start.parents:
            if (p / 'src' / 'drobot_bringup').is_dir():
                return p
    raise RuntimeError('저장소 루트를 찾지 못했다')


ROOT = _repo_root()
NAV = ROOT / 'src/drobot_bringup/config/navigation'
NAV2_FILES = {
    'default': NAV / 'nav2_params_hybrid.yaml',
    'derived': NAV / 'nav2_params_hybrid_derived.yaml',
}
ENERGY_FILES = {
    'default': ROOT / 'src/drobot_hybrid_planner/config/energy_params.yaml',
    'derived': ROOT / 'src/drobot_hybrid_planner/config/energy_params_derived.yaml',
}
ELEVATION_FILE = ROOT / 'src/drobot_costmap_2_5d/config/elevation_params.yaml'
PHYSICAL_FILE = ROOT / 'src/drobot_description/config/robot_physical.yaml'
MODE_SWITCH_FILE = ROOT / 'src/drobot_mode_manager/config/mode_switch_params.yaml'

COSTMAPS = ('local_costmap', 'global_costmap')

# (costmap, 키): (nav2_params 에 있어야 하는 값, 이유)
ALLOWED_ELEVATION_OVERRIDES = {
    ('local_costmap', 'cost_values.fly_over'): (
        254, '지역 지도는 지상 컨트롤러만 읽는다. 200 이면 DWB 가 밟는다 (D-2)'),
    ('global_costmap', 'clear_on_reset'): (
        False, '복구 동작이 지형 기억과 prior map 을 지우지 않게 (310de31)'),
    ('global_costmap', 'publish_elevation_grid'): (
        True, 'RViz 확인용'),
}

# 결론을 좌우하는 값 — 이 키들이 비교 대상에서 빠지면 검사가 무의미해진다
ENERGY_KEYS_REQUIRED = (
    'cost_weights.w_energy', 'cost_weights.w_switch', 'cost_weights.w_time',
    'ground_mode.energy_per_meter', 'ground_mode.speed',
    'air_mode.energy_per_meter', 'air_mode.speed',
    'mode_switch.takeoff_energy', 'mode_switch.landing_energy',
    'mode_switch.takeoff_time', 'mode_switch.landing_time',
    'mode_switch.energy_per_altitude_meter',
)


def _load(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def _leaves(d, prefix=''):
    """중첩 dict 를 'a.b.c': 값 으로 편다."""
    out = {}
    if isinstance(d, dict):
        for k, v in d.items():
            out.update(_leaves(v, f'{prefix}.{k}' if prefix else str(k)))
    else:
        out[prefix] = d
    return out


def _elevation_block(nav2: dict, costmap: str) -> dict:
    return nav2[costmap][costmap]['ros__parameters']['elevation_layer']


def _diff(expected: dict, actual: dict, keys) -> list:
    return [f'{k}: {expected[k]!r} != {actual[k]!r}'
            for k in sorted(keys) if expected[k] != actual[k]]


def test_two_nav2_files_differ_only_in_energy():
    """에너지 계수 말고는 default 와 derived 가 같아야 한다."""
    a = _load(NAV2_FILES['default'])
    b = _load(NAV2_FILES['derived'])
    for d in (a, b):
        d['planner_server']['ros__parameters'].pop('energy_model')
    la, lb = _leaves(a), _leaves(b)
    only = sorted(set(la) ^ set(lb))
    assert not only, f'한쪽 파일에만 있는 키: {only}'
    bad = _diff(la, lb, la.keys())
    assert not bad, ('에너지 블록 밖에서 두 파일이 다르다 — 세트를 바꾸면 이것도 같이 '
                     '바뀐다:\n' + '\n'.join(bad))


@pytest.mark.parametrize('name', sorted(NAV2_FILES))
def test_energy_block_matches_package_config(name):
    """nav2_params 의 energy_model 과 벤치마크가 읽는 energy_params 가 같아야 한다."""
    pkg = _leaves(_load(ENERGY_FILES[name])['energy_model']['ros__parameters'])
    nav2 = _leaves(_load(NAV2_FILES[name])['planner_server']['ros__parameters']
                   ['energy_model'])
    missing = [k for k in ENERGY_KEYS_REQUIRED if k not in pkg or k not in nav2]
    assert not missing, f'비교해야 할 키가 한쪽에 없다: {missing}'
    bad = _diff(pkg, nav2, set(pkg) & set(nav2))
    assert not bad, (f'{ENERGY_FILES[name].name} 와 {NAV2_FILES[name].name} 가 다르다 '
                     '— 오프라인과 Gazebo 가 다른 숫자로 계획한다:\n' + '\n'.join(bad))


@pytest.mark.parametrize('name', sorted(NAV2_FILES))
@pytest.mark.parametrize('costmap', COSTMAPS)
def test_elevation_block_matches_package_config(name, costmap):
    """elevation_layer 값은 elevation_params.yaml 과 같아야 한다 (의도된 차이 제외)."""
    pkg = _leaves(_load(ELEVATION_FILE)['elevation_layer']['ros__parameters'])
    nav2 = _leaves(_elevation_block(_load(NAV2_FILES[name]), costmap))
    common = set(pkg) & set(nav2)
    assert len(common) >= 20, f'비교 대상 키가 {len(common)} 개뿐이다 — 구조가 바뀌었나'

    overrides = {k: v for (cm, k), v in ALLOWED_ELEVATION_OVERRIDES.items() if cm == costmap}
    for key, (want, why) in overrides.items():
        assert nav2.get(key) == want, (
            f'{costmap}.{key} 는 {want!r} 여야 한다 ({why}). 지금 {nav2.get(key)!r}')
    bad = _diff(pkg, nav2, common - set(overrides))
    assert not bad, (f'{NAV2_FILES[name].name} 의 {costmap} elevation_layer 가 '
                     'elevation_params.yaml 과 다르다. 의도된 차이면 '
                     'ALLOWED_ELEVATION_OVERRIDES 에 이유와 함께 넣을 것:\n' + '\n'.join(bad))


def test_ssot_constants():
    """robot_physical.yaml 의 공유 상수를 복사해 쓴 곳이 전부 같은 값이어야 한다."""
    phys = _load(PHYSICAL_FILE)['robot_physical']
    mm = _load(MODE_SWITCH_FILE)['mode_manager']['ros__parameters']
    elev_pkg = _load(ELEVATION_FILE)['elevation_layer']['ros__parameters']

    found = []   # (어디, 값, 기대값)
    found.append(('mode_switch_params max_flight_distance', mm['max_flight_distance'],
                  phys['max_flight_distance']))
    found.append(('mode_switch_params min_ceiling_clearance', mm['min_ceiling_clearance'],
                  phys['min_ceiling_clearance']))
    blocks = [('elevation_params.yaml', elev_pkg)]
    for name, path in NAV2_FILES.items():
        nav2 = _load(path)
        grid = nav2['planner_server']['ros__parameters']['GridBased']
        found.append((f'{name} planner flight_clearance', grid['flight_clearance'],
                      phys['flight_clearance']))
        found.append((f'{name} planner max_flight_segment', grid['max_flight_segment'],
                      phys['max_flight_distance']))
        found.append((f'{name} planner ceiling_height', grid['ceiling_height'],
                      phys['ceiling_height']))
        for cm in COSTMAPS:
            blocks.append((f'{name} {cm}', _elevation_block(nav2, cm)))
    for where, b in blocks:
        found.append((f'{where} robot_flight_height', b['robot_flight_height'],
                      phys['flight_clearance']))
        found.append((f'{where} min_ceiling_clearance', b['min_ceiling_clearance'],
                      phys['min_ceiling_clearance']))
        found.append((f'{where} ceiling_height', b['ceiling_height'], phys['ceiling_height']))
        # fly_over_max 는 천장 제약에서 유도한 값이다: 천장 - 비행고도 - 천장 여유
        derived = phys['ceiling_height'] - phys['flight_clearance'] - phys['min_ceiling_clearance']
        found.append((f'{where} fly_over_max', b['height_thresholds']['fly_over_max'],
                      pytest.approx(derived)))

    bad = [f'{w}: {v!r} (기대 {e!r})' for w, v, e in found if v != e]
    assert not bad, 'robot_physical.yaml 과 다른 복사본:\n' + '\n'.join(bad)
