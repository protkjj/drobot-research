"""상태공간 정의 — 3D (x,y,m) 와 4D (x,y,z,m).

이 모듈이 존재하는 이유:
    Dijkstra / A* / RRT* 가 '같은 문제'를 풀고 있다는 걸 코드로 보장하기 위해서다.
    플래너마다 이웃 생성 규칙이나 비용 계산이 미묘하게 다르면
    성능 차이가 알고리즘 때문인지 문제 정의 때문인지 구분할 수 없게 된다.
    그래서 상태 정의, 전이 규칙, 비용은 전부 여기 한 곳에 둔다.

상태 표현
    3D: (ix, iy, mode, level)   mode: 0=ground, 1=air
        level: 비행 고도 레벨 인덱스 (ground 면 0)

        비행 고도를 '지형높이 + clearance' 로 매 셀 재계산하면(지형 추종),
        장애물 경계에서 한 셀 만에 고도가 장애물 높이만큼 점프한다.
        격자 0.1m 에서 장애물 0.55m 면 상승각 79.7도가 되어
        물리적으로 불가능하다.

        그래서 '비행 구간 고도 고정' 방식을 쓴다:
          - 이륙할 때 도달할 고도를 정하고
          - 그 비행 구간 내내 같은 고도를 유지하며
          - 착륙할 때 내려온다
        실제 드론 운용(장애물 앞에서 미리 상승 후 수평 통과)과 일치한다.

        고도 후보는 맵에 존재하는 장애물 높이에서만 만든다
        (h + flight_clearance). 맵당 2~7개뿐이라 상태공간이 크게 늘지 않는다.

    4D: (ix, iy, iz, mode)      iz: 고도 인덱스 (mode=ground면 항상 0)
        air 모드의 고도가 탐색 변수가 된다.

전이 (엣지)
    1. 지상 이동   ground -> ground   (인접 셀, 8방향)
    2. 공중 이동   air -> air         (인접 셀, 8방향, 4D는 고도 변경 동반 가능)
    3. 이륙        ground -> air      (같은 셀)
    4. 착륙        air -> ground      (같은 셀)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from cost.energy import CostAccumulator, EnergyModel
from envs.heightmap import HeightMap, ROVER_MAX

GROUND, AIR = 0, 1

# 제자리 수직 이동에서 한 번에 이동 가능한 최대 높이 (m).
# 격자 칸수가 아니라 실제 길이로 정의해야 z해상도에 무관해진다.
# 값 자체는 이 정도면 어떤 고도로도 몇 스텝 안에 도달한다는 실용적 선택.
_VERTICAL_JUMP_M = 0.4

# 8방향 이웃. (dx, dy, 거리배수)
_NEIGHBORS_8 = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


@dataclass
class ProblemSpec:
    """한 번의 경로계획 문제를 정의하는 모든 것.

    dims: 3 이면 (x,y,m), 4 이면 (x,y,z,m)
    """

    hm: HeightMap
    model: EnergyModel
    dims: int                      # 3 or 4
    flight_clearance: float = 0.8  # 3D에서만 사용
    z_res: float = 0.1             # 4D에서만 사용 — 고도 격자 해상도

    # 이동 방식 제한 — 3개 modal 을 같은 상태공간 위에서 표현한다.
    # 장애물을 만났을 때 '어떻게 넘어가느냐'가 modal 을 가른다.
    #
    #   modal          우회   밟고넘기   비행
    #   ------------------------------------
    #   rover_detour    O       X        X     장애물을 피해 돌아간다
    #   rover_climb     O       O        X     낮은 장애물은 타고 넘는다
    #   hybrid          O       X        O     드론으로 전환해 날아 넘는다
    #
    # 밟고넘기는 climb_mode.max_height (0.7m) 이하 장애물만 가능하다.
    # 세 modal 이 같은 비용 함수를 쓰므로 결과를 직접 비교할 수 있다.
    #
    # 참고용으로 남겨둔 modal:
    #   "drone_only"  시작에서 이륙해 목표에서만 착륙 (전 구간 비행)
    modal: str = "hybrid"

    def __post_init__(self):
        if self.dims not in (3, 4):
            raise ValueError(f"dims는 3 또는 4여야 한다: {self.dims}")

        self.z_max = self.model.max_flight_altitude(self.hm.ceiling)
        # 4D 고도 격자: 0 부터 z_max 까지
        self.nz = int(self.z_max / self.z_res) + 1 if self.dims == 4 else 1

        # 비행 고도 후보 (3D 전용).
        # 맵에 실제로 존재하는 장애물 높이 + clearance 만 후보로 삼는다.
        # 그보다 높이 날 이유가 없고, 낮게 날면 장애물에 걸린다.
        if self.dims == 3:
            hs = np.unique(np.round(self.hm.grid, 3))
            cands = []
            for h in hs:
                if h > self.z_max:      # 어차피 못 넘는 높이
                    continue
                z = float(h) + self.flight_clearance
                if z <= self.z_max + 1e-9:
                    cands.append(round(z, 4))
            # 최소 하나는 있어야 이륙이 가능하다 (평지 위 비행)
            if not cands:
                cands = [min(self.flight_clearance, self.z_max)]
            self.air_levels = sorted(set(cands))
        else:
            self.air_levels = []

        # 통과 가능 높이 상한 — 이보다 높으면 어떤 방법으로도 못 넘는다
        if self.dims == 3:
            # 3D: 고도가 h + clearance 로 고정되므로 h + clearance <= z_max
            self.h_limit = self.z_max - self.flight_clearance
        else:
            # 4D: 고도 자유, 하한만 지키면 됨
            self.h_limit = self.z_max - self.model.min_flight_clearance

    # -- modal 별 능력 ---------------------------------------------------
    @property
    def allow_ground(self) -> bool:
        """지상 주행을 쓸 수 있는가."""
        return self.modal != "drone_only"

    @property
    def allow_fly(self) -> bool:
        """드론으로 전환해 날 수 있는가."""
        return self.modal in ("hybrid", "drone_only")

    @property
    def allow_climb(self) -> bool:
        """장애물을 밟고 넘을 수 있는가."""
        return self.modal == "rover_climb"

    @property
    def rover_h_limit(self) -> float:
        """로버가 올라설 수 있는 지형 높이의 상한 (m).

        기본은 ROVER_MAX(0.15) — 턱 정도만 넘는다.
        rover_climb 은 climb_mode.max_height(0.7)까지 타고 넘을 수 있다.
        이 한 줄이 '우회'와 '밟고넘기'를 가르는 지점이다.
        """
        if self.allow_climb:
            return self.model.rover_climb_max_h
        return ROVER_MAX

    # -- 격자 <-> 실수 고도 --------------------------------------------
    def z_of(self, iz: int) -> float:
        return iz * self.z_res

    def iz_of(self, z: float) -> int:
        return int(round(z / self.z_res))

    # -- 상태 유효성 ----------------------------------------------------
    def terrain(self, ix: int, iy: int) -> float:
        return float(self.hm.grid[iy, ix])

    def is_wall(self, ix: int, iy: int) -> bool:
        """어떤 모드로도 통과 불가한 셀인가."""
        return self.terrain(ix, iy) > self.h_limit

    def ground_ok(self, ix: int, iy: int) -> bool:
        """로버가 이 셀에 있을 수 있는가.

        rover_climb 이면 장애물 위(<=0.7m)도 '있을 수 있는 곳'이 된다.
        1e-9 여유는 0.70m 장애물이 부동소수 오차로 걸러지는 걸 막기 위함.
        """
        if not self.hm.in_bounds(ix, iy):
            return False
        return self.terrain(ix, iy) <= self.rover_h_limit + 1e-9

    def air_z_3d(self, ix: int, iy: int) -> float:
        """3D에서 이 셀 위를 날 때의 고도 (자동 결정)."""
        return self.terrain(ix, iy) + self.flight_clearance

    def air_ok_at(self, ix: int, iy: int, level: int) -> bool:
        """고정 고도 level 로 이 셀 위를 날 수 있는가 (3D).

        지형 위 최소 여유를 지키고 천장 아래여야 한다.
        """
        if not self.hm.in_bounds(ix, iy):
            return False
        z = self.air_levels[level]
        h = self.terrain(ix, iy)
        return (z >= h + self.model.min_flight_clearance - 1e-9) and (z <= self.z_max + 1e-9)

    def air_ok(self, ix: int, iy: int, iz: int = 0) -> bool:
        """이 셀 위를 (그 고도로) 날 수 있는가."""
        if not self.hm.in_bounds(ix, iy):
            return False
        h = self.terrain(ix, iy)
        if self.dims == 3:
            return self.air_z_3d(ix, iy) <= self.z_max
        z = self.z_of(iz)
        # 지형 위 최소 여유를 지키고, 천장 아래여야 한다
        return (z >= h + self.model.min_flight_clearance) and (z <= self.z_max)

    def _diag_ok_ground(self, ix: int, iy: int, dx: int, dy: int) -> bool:
        """대각선 지상 이동에서 모서리를 뚫고 지나가지 않는가.

        8방향 격자의 고전적 함정(corner-cutting): (0,0)->(1,1) 대각 이동이
        (1,0)과 (0,1)이 벽이어도 허용되면, 로봇이 벽 모서리를 대각으로
        통과하는 물리적으로 불가능한 경로가 나온다.
        실제로 이 버그 때문에 A* 경로가 높이 1.10m 벽을 관통했다.

        대각선 이동은 인접한 두 직교 셀이 '모두' 통과 가능할 때만 허용한다.
        """
        if dx == 0 or dy == 0:
            return True
        return self.ground_ok(ix + dx, iy) and self.ground_ok(ix, iy + dy)

    def _diag_ok_air(self, ix: int, iy: int, dx: int, dy: int, iz: int = 0) -> bool:
        """대각선 공중 이동의 모서리 검사 (지상과 같은 이유)."""
        if dx == 0 or dy == 0:
            return True
        return self.air_ok(ix + dx, iy, iz) and self.air_ok(ix, iy + dy, iz)

    # -- 상태 생성 ------------------------------------------------------
    def start_state(self) -> tuple:
        ix, iy = self.hm.to_idx(*self.hm.start)
        return (ix, iy, GROUND, 0) if self.dims == 3 else (ix, iy, 0, GROUND)

    def goal_state(self) -> tuple:
        ix, iy = self.hm.to_idx(*self.hm.goal)
        return (ix, iy, GROUND, 0) if self.dims == 3 else (ix, iy, 0, GROUND)

    def is_goal(self, s: tuple) -> bool:
        return s == self.goal_state()

    # -- 전이 -----------------------------------------------------------
    def neighbors(self, s: tuple):
        """상태 s에서 갈 수 있는 (다음상태, CostAccumulator) 목록을 생성.

        CostAccumulator를 반환하는 이유: 단순 스칼라 비용만 반환하면
        나중에 '비행 에너지가 얼마였나' 같은 분석을 못 한다.
        logging_config.yaml의 trial_summary가 그 분해를 요구한다.
        """
        if self.dims == 3:
            yield from self._neighbors_3d(s)
        else:
            yield from self._neighbors_4d(s)

    def _ground_edge(self, ix: int, iy: int, nx_: int, ny_: int,
                     dist: float) -> CostAccumulator:
        """지상 이동 한 스텝의 비용. 올라가는 경우 등반 비용을 더한다.

        높이가 오르는 스텝에서만 climb_move 를 쓴다. 평지끼리 이동하거나
        장애물에서 내려오는 스텝은 평지 주행과 같다.
        rover_climb 이 아닌 modal 에서는 ground_ok 가 0.15m 초과 셀을
        막으므로 dh 가 유의미하게 커지는 일이 없다.
        """
        dh = self.terrain(nx_, ny_) - self.terrain(ix, iy)
        if dh > 1e-9:
            return self.model.climb_move(dist, dh)
        return self.model.ground_move(dist)

    # ---- 3D ----
    def _neighbors_3d(self, s: tuple):
        """비행 구간 고도 고정 방식.

        상태는 (ix, iy, mode, level). 비행 중에는 level 이 바뀌지 않으므로
        고도 점프가 생기지 않고, 따라서 상승각 제약을 위반하지 않는다.
        고도 변경이 필요하면 착륙 후 다시 이륙해야 한다 — 그 비용은
        takeoff/landing 으로 정상 계상된다.
        """
        ix, iy, mode, level = s
        res = self.hm.resolution

        if mode == GROUND:
            # 1) 지상 이동
            if self.allow_ground:
                for dx, dy, k in _NEIGHBORS_8:
                    nx_, ny_ = ix + dx, iy + dy
                    if self.ground_ok(nx_, ny_) and self._diag_ok_ground(ix, iy, dx, dy):
                        yield (nx_, ny_, GROUND, 0), self._ground_edge(ix, iy, nx_, ny_, k * res)

            # 2) 이륙 — 어느 고도로 뜰지 선택한다.
            #    이게 3D 에서 유일하게 고도를 정하는 순간이다.
            if self.allow_fly:
                for lv in range(len(self.air_levels)):
                    if not self.air_ok_at(ix, iy, lv):
                        continue
                    yield (ix, iy, AIR, lv), self.model.takeoff(self.air_levels[lv])

        else:
            z = self.air_levels[level]
            # 3) 공중 이동 — 고도 고정이므로 수직 비용이 없다
            for dx, dy, k in _NEIGHBORS_8:
                nx_, ny_ = ix + dx, iy + dy
                if not self.air_ok_at(nx_, ny_, level):
                    continue
                # 대각 이동 모서리 검사 (같은 고도 기준)
                if dx != 0 and dy != 0:
                    if not (self.air_ok_at(ix + dx, iy, level) and
                            self.air_ok_at(ix, iy + dy, level)):
                        continue
                clr = min(z - self.terrain(ix, iy), z - self.terrain(nx_, ny_))
                yield ((nx_, ny_, AIR, level),
                       self.model.air_move_horizontal(k * res, clearance=clr))

            # 4) 착륙
            allow_landing = self.ground_ok(ix, iy) and self.allow_ground
            if self.modal == "drone_only":
                gx, gy = self.hm.to_idx(*self.hm.goal)
                allow_landing = allow_landing and (ix == gx and iy == gy)
            if allow_landing:
                yield (ix, iy, GROUND, 0), self.model.landing(z)

    # ---- 4D ----
    def _neighbors_4d(self, s: tuple):
        ix, iy, iz, mode = s
        res = self.hm.resolution

        if mode == GROUND:
            # 1) 지상 이동
            if self.allow_ground:
                for dx, dy, k in _NEIGHBORS_8:
                    nx_, ny_ = ix + dx, iy + dy
                    if self.ground_ok(nx_, ny_) and self._diag_ok_ground(ix, iy, dx, dy):
                        yield (nx_, ny_, 0, GROUND), self._ground_edge(ix, iy, nx_, ny_, k * res)
            # 2) 이륙 — 어느 고도로 뜰지 선택 가능 (이게 4D의 핵심)
            if not self.allow_fly:
                return
            h = self.terrain(ix, iy)
            iz_lo = self.iz_of(h + self.model.min_flight_clearance)
            iz_lo = max(iz_lo, 0)
            for nz_ in range(iz_lo, self.nz):
                if self.air_ok(ix, iy, nz_):
                    yield (ix, iy, nz_, AIR), self.model.takeoff(self.z_of(nz_))
        else:
            z_from = self.z_of(iz)
            # 3) 공중 이동 — 수평 이동 + 고도 변경을 함께 허용
            #
            # 고도 변경 폭은 '격자 칸수'가 아니라 '최대 상승각'으로 정한다.
            # 예전에는 diz in (-1,0,1)로 한 칸만 허용했는데, 그러면
            # z해상도가 촘촘할수록 최대 경사가 얕아져서
            # (z=0.10 -> 45도, z=0.05 -> 26.6도) 해상도를 높일수록
            # 해가 나빠지는 비단조 버그가 생겼다.
            for dx, dy, k in _NEIGHBORS_8:
                nx_, ny_ = ix + dx, iy + dy
                if not self.hm.in_bounds(nx_, ny_):
                    continue
                horiz = k * res
                max_dz = self.model.max_dz_for(horiz)
                max_steps = max(1, int(max_dz / self.z_res + 1e-9))
                for diz in range(-max_steps, max_steps + 1):
                    nz_ = iz + diz
                    if nz_ < 0 or nz_ >= self.nz:
                        continue
                    if not self.air_ok(nx_, ny_, nz_):
                        continue
                    if not self._diag_ok_air(ix, iy, dx, dy, nz_):
                        continue
                    z_to = self.z_of(nz_)
                    clr = min(z_from - self.terrain(ix, iy),
                              z_to - self.terrain(nx_, ny_))
                    acc = self.model.air_move_horizontal(horiz, clearance=clr)
                    if diz != 0:
                        acc = acc + self.model.air_move_vertical(z_to - z_from)
                    yield (nx_, ny_, nz_, AIR), acc
            # 3b) 제자리 고도 변경 — 수평 이동 없이 오르내리기.
            #     여기도 한 칸으로 묶으면 z해상도가 촘촘할 때 같은 고도차를
            #     내는 데 더 많은 상태를 거쳐야 해서 탐색이 불리해진다.
            #     물리적으로는 수직 상승/하강에 각도 제약이 없으므로
            #     '한 번에 일정 높이까지'를 허용한다.
            vert_steps = max(1, int(round(_VERTICAL_JUMP_M / self.z_res)))
            for diz in range(-vert_steps, vert_steps + 1):
                if diz == 0:
                    continue
                nz_ = iz + diz
                if 0 <= nz_ < self.nz and self.air_ok(ix, iy, nz_):
                    yield (ix, iy, nz_, AIR), self.model.air_move_vertical(
                        self.z_of(nz_) - z_from)
            # 4) 착륙
            if self.ground_ok(ix, iy):
                yield (ix, iy, 0, GROUND), self.model.landing(z_from)

    # -- 휴리스틱 (A*용) -------------------------------------------------
    def heuristic(self, s: tuple) -> float:
        """목표까지의 비용 하한 추정.

        admissible(과대평가 안 함) 해야 A*의 최적성이 보장된다.
        가장 싼 이동 수단인 지상 주행 비용만으로 하한을 잡는다.
        비행은 지상보다 비싸므로(2.0 > 0.5 Wh/m) 이건 안전한 하한이다.
        시간 항도 지상 주행 기준이 더 느리므로... 주의가 필요하다.

        비용 = alpha * E + gamma * T 이고
          지상: alpha*0.5*d + gamma*d/0.3 = (1.0*0.5 + 0.5/0.3) * d = 2.1667*d
          공중: alpha*2.0*d + gamma*d/0.5 = (1.0*2.0 + 0.5/0.5) * d = 3.0*d
        지상이 항상 싸므로 지상 단가로 하한을 잡으면 admissible.
        """
        gx, gy = self.hm.to_idx(*self.hm.goal)
        ix, iy = s[0], s[1]
        # 8방향 이동이므로 옥타일 거리 (유클리드보다 타이트)
        dx, dy = abs(ix - gx), abs(iy - gy)
        d_cells = (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)
        d = d_cells * self.hm.resolution
        return self._unit_cost_ground * d

    @property
    def _unit_cost_ground(self) -> float:
        """지상 주행 1미터당 비용 (가중치 적용 후)."""
        m = self.model
        return m.alpha * m.ground_wh_per_m + m.gamma / m.ground_speed

    # -- 경로 -> 비용 재계산 ----------------------------------------------
    def path_cost(self, path: list[tuple]) -> CostAccumulator:
        """상태 시퀀스의 총 비용을 다시 계산한다 (검증용).

        플래너가 보고한 비용과 이 값이 다르면 플래너에 버그가 있다.
        """
        total = CostAccumulator()
        for a, b in zip(path, path[1:]):
            found = None
            for nxt, acc in self.neighbors(a):
                if nxt == b:
                    found = acc
                    break
            if found is None:
                raise ValueError(f"경로에 유효하지 않은 전이가 있다: {a} -> {b}")
            total = total + found
        return total

    def summary(self) -> dict:
        n_xy = self.hm.nx * self.hm.ny
        return {
            "map": self.hm.name,
            "dims": self.dims,
            "nx": self.hm.nx, "ny": self.hm.ny, "nz": self.nz,
            "n_states": n_xy * self.nz * 2 if self.dims == 4 else n_xy * 2,
            "z_max": round(self.z_max, 3),
            "h_limit": round(self.h_limit, 3),
        }
