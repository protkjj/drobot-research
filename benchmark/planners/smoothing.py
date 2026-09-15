"""경로 후처리 — shortcut smoothing.

왜 필요한가
    A*는 8방향 격자에 묶여 있어 이동 각도가 45도 배수로 제한된다.
    RRT*는 연속 공간이라 임의 각도로 간다.
    이 상태로 비교하면 RRT*가 A*보다 5% 정도 싼 해를 내는데,
    이건 알고리즘의 우열이 아니라 '격자 이산화 페널티'다.

    실제 시스템에서도 격자 플래너의 출력은 그대로 쓰지 않고 스무딩한다.
    kj의 config 양쪽 모두 이를 전제한다:
        hybrid_rrt_params.yaml:  smooth_path: true
        nav2_params.yaml:        SmacPlanner2D + smoother

    그래서 A*와 RRT* 양쪽에 '같은' 스무더를 적용해서 비교한다.
    한쪽에만 적용하면 그게 또 불공정이 된다.

방식
    shortcut smoothing: 경로 위의 두 점 i, j를 직선으로 이을 수 있고
    그게 더 싸면 중간 점들을 제거한다. 수렴할 때까지 반복.

    모드 전환 지점(이/착륙)은 건너뛰지 않는다. 이착륙은 '같은 위치에서만'
    허용되는 전이라 shortcut으로 잘라내면 물리적으로 불가능한 경로가 된다.
"""
from __future__ import annotations

import math

from cost.energy import CostAccumulator
from planners.state_space import ProblemSpec, GROUND, AIR

# 통일 경로 표현: [(x, y, z, mode), ...]
Waypoint = tuple[float, float, float, int]


def grid_path_to_xyz(spec: ProblemSpec, path: list[tuple]) -> list[Waypoint]:
    """A*/Dijkstra의 격자 경로를 (x, y, z, mode) 실좌표 경로로 변환."""
    out: list[Waypoint] = []
    for s in path:
        if spec.dims == 3:
            # 3D 상태는 (ix, iy, mode, level) — level 이 비행 고도 인덱스.
            # 고도가 지형을 따라가지 않고 구간 내내 고정이므로
            # 여기서도 level 로 조회한다.
            ix, iy, mode, level = s
            x, y = spec.hm.to_xy(ix, iy)
            z = spec.air_levels[level] if mode == AIR else 0.0
        else:
            ix, iy, iz, mode = s
            x, y = spec.hm.to_xy(ix, iy)
            z = spec.z_of(iz) if mode == AIR else 0.0
        out.append((x, y, z, mode))
    return out


def _terrain(spec: ProblemSpec, x: float, y: float) -> float:
    ix, iy = spec.hm.to_idx(x, y)
    if not spec.hm.in_bounds(ix, iy):
        return float("inf")
    return float(spec.hm.grid[iy, ix])


def _segment_ok(spec: ProblemSpec, a: Waypoint, b: Waypoint) -> bool:
    """a에서 b로 직선 이동이 가능한가 (같은 모드 가정)."""
    ax, ay, az, am = a
    bx, by, bz, bm = b
    if am != bm:
        return False
    n = max(2, int(math.hypot(bx - ax, by - ay) / (spec.hm.resolution * 0.4)) + 1)
    for i in range(n + 1):
        t = i / n
        x = ax + (bx - ax) * t
        y = ay + (by - ay) * t
        ix, iy = spec.hm.to_idx(x, y)
        if not spec.hm.in_bounds(ix, iy):
            return False
        if am == GROUND:
            if not spec.ground_ok(ix, iy):
                return False
        else:
            z = az + (bz - az) * t
            h = _terrain(spec, x, y)
            if z < h + spec.model.min_flight_clearance - 1e-9:
                return False
            if z > spec.z_max + 1e-9:
                return False
    return True


def segment_cost(spec: ProblemSpec, a: Waypoint, b: Waypoint) -> CostAccumulator | None:
    """a -> b 한 구간의 비용. 불가능하면 None.

    A*/RRT* 와 같은 cost/energy.py 를 쓴다.
    """
    ax, ay, az, am = a
    bx, by, bz, bm = b
    d = math.hypot(bx - ax, by - ay)
    m = spec.model

    # 모드 전환 — 같은 위치에서만
    if am != bm:
        if d > 1e-6:
            return None
        if bm == AIR:
            return m.takeoff(bz)
        return m.landing(az)

    if not _segment_ok(spec, a, b):
        return None

    if am == GROUND:
        if not spec.allow_climb:
            return m.ground_move(d)
        return _ground_cost_with_climb(spec, a, b, d)

    # 최대 상승각 — 스무딩이 물리적으로 불가능한 급경사 구간을 만들지 않도록.
    # A*/RRT* 와 같은 제약이어야 스무딩 후 비교도 공정하다.
    if abs(bz - az) > m.max_dz_for(d) + 1e-9:
        return None

    clr = min(az - _terrain(spec, ax, ay), bz - _terrain(spec, bx, by))
    acc = m.air_move_horizontal(d, clearance=clr)
    if abs(bz - az) > 1e-9:
        acc = acc + m.air_move_vertical(bz - az)
    return acc


def _ground_cost_with_climb(spec: ProblemSpec, a: Waypoint, b: Waypoint,
                            d: float) -> CostAccumulator:
    """rover_climb 구간의 지상 비용 — 지나가는 지형의 총 상승량을 반영한다.

    왜 필요한가:
        스무더는 두 점을 직선으로 잇는데, 그 직선이 장애물 위를 지날 수 있다.
        이때 평지 주행 비용만 매기면 '밟고 넘기'가 공짜가 되어
        rover_climb 이 부당하게 싸진다.
        state_space._ground_edge 와 같은 규칙(양의 높이차에만 부과)을
        직선 구간에도 적용해야 격자 경로와 스무딩 경로의 비용이 일관된다.

    격자 해상도 간격으로 샘플링해 셀 단위 이동을 흉내낸다.
    지형이 셀 단위 계단 함수라 이 간격이면 상승 지점을 놓치지 않는다.
    """
    ax, ay, _, _ = a
    bx, by, _, _ = b
    n = max(1, int(round(d / spec.hm.resolution)))
    step = d / n
    total = CostAccumulator()
    prev_h = _terrain(spec, ax, ay)
    for i in range(1, n + 1):
        t = i / n
        h = _terrain(spec, ax + (bx - ax) * t, ay + (by - ay) * t)
        total = total + spec.model.climb_move(step, h - prev_h)
        prev_h = h
    return total


def path_cost(spec: ProblemSpec, path: list[Waypoint]) -> CostAccumulator | None:
    """경로 전체 비용."""
    total = CostAccumulator()
    for a, b in zip(path, path[1:]):
        c = segment_cost(spec, a, b)
        if c is None:
            return None
        total = total + c
    return total


def _smooth_segment(spec: ProblemSpec, seg: list[Waypoint],
                    max_passes: int) -> list[Waypoint]:
    """같은 모드로만 이루어진 한 구간을 스무딩한다."""
    if len(seg) < 3:
        return seg

    cur = list(seg)
    for _ in range(max_passes):
        changed = False
        i = 0
        while i < len(cur) - 2:
            best_j = None
            best_gain = 1e-9
            # 가장 멀리 있는 j 부터 — 많이 자를수록 이득이 크다
            for j in range(len(cur) - 1, i + 1, -1):
                direct = segment_cost(spec, cur[i], cur[j])
                if direct is None:
                    continue
                orig = CostAccumulator()
                ok = True
                for a, b in zip(cur[i:j], cur[i + 1:j + 1]):
                    c = segment_cost(spec, a, b)
                    if c is None:
                        ok = False
                        break
                    orig = orig + c
                if not ok:
                    continue
                gain = spec.model.cost(orig) - spec.model.cost(direct)
                if gain > best_gain:
                    best_gain, best_j = gain, j
                    break
            if best_j is not None:
                cur = cur[:i + 1] + cur[best_j:]
                changed = True
            i += 1
        if not changed:
            break
    return cur


def shortcut_smooth(spec: ProblemSpec, path: list[Waypoint],
                    max_passes: int = 30) -> list[Waypoint]:
    """중간 점을 건너뛰어 경로를 짧게 만든다.

    모드 전환 지점을 경계로 구간을 나눠 각 구간 안에서만 스무딩한다.

    왜 나누는가:
        예전에는 경로 전체를 한 덩어리로 보고 shortcut 을 시도했다.
        그런데 이착륙은 '같은 위치에서만' 허용되는 전이라
        전환 지점을 건너뛰는 shortcut 은 segment_cost 가 None 을 낸다.
        그 결과 hybrid 경로는 어느 구간도 스무딩되지 못했고,
        rover_detour 경로만 스무딩되어 비교가 왜곡됐다.
        (hard_open: 원본은 hybrid 30.01 < rover 31.03 인데
         스무딩 후 hybrid 30.01 > rover 29.48 로 역전)

        hybrid 는 rover_detour 의 상위집합이므로 hybrid 가 더 비쌀 수 없다.
        구간별로 나눠 스무딩하면 이 성질이 유지된다.

    전환 지점 자체는 보존된다 — 잘라내면 물리적으로 불가능한 경로가 된다.
    """
    if len(path) < 3:
        return path

    # 모드가 바뀌는 지점에서 구간을 끊는다
    segments: list[list[Waypoint]] = []
    cur_seg = [path[0]]
    for k in range(1, len(path)):
        if path[k][3] != path[k - 1][3]:   # [3] = mode
            segments.append(cur_seg)
            cur_seg = [path[k]]
        else:
            cur_seg.append(path[k])
    segments.append(cur_seg)

    out: list[Waypoint] = []
    for si, seg in enumerate(segments):
        sm = _smooth_segment(spec, seg, max_passes)
        if si == 0:
            out.extend(sm)
        else:
            # 구간 경계(모드 전환)는 그대로 이어 붙인다
            out.extend(sm)
    return out


def smooth_result(spec: ProblemSpec, result, is_grid: bool):
    """SearchResult를 스무딩해서 새 (cost, acc, path) 를 반환.

    is_grid=True 면 격자 경로를 실좌표로 먼저 변환한다.
    스무딩이 실패하거나 개선이 없으면 원본을 그대로 돌려준다.
    """
    if not result.found or not result.path:
        return result.cost, result.acc, result.path

    raw = grid_path_to_xyz(spec, result.path) if is_grid else list(result.path)

    base = path_cost(spec, raw)
    if base is None:
        # 변환된 경로가 유효하지 않다 — 원본 유지 (드문 경우)
        return result.cost, result.acc, result.path

    sm = shortcut_smooth(spec, raw)
    acc = path_cost(spec, sm)
    if acc is None:
        return spec.model.cost(base), base, raw

    c_base, c_sm = spec.model.cost(base), spec.model.cost(acc)
    if c_sm <= c_base:
        return c_sm, acc, sm
    return c_base, base, raw
