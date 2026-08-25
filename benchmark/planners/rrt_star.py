"""Hybrid RRT* — 지상/공중 모드를 함께 다루는 샘플링 기반 플래너.

목적이 'A* 채택 근거 확보'이므로, RRT*는 오히려 **최대한 강하게** 만든다.
약한 RRT*를 이겨봐야 "제대로 구현 안 한 거 아니냐"는 반박에 답할 수 없다.

강화 요소
  - Informed RRT* : 해를 하나 찾은 뒤에는 그 비용으로 정의되는 타원 안에서만
                    샘플링한다. 무의미한 영역을 버려서 수렴이 크게 빨라진다.
  - goal bias     : 일정 비율로 목표점을 직접 샘플 (hybrid_rrt_params.yaml)
  - rewire        : 새 노드 주변을 다시 연결해 트리를 개선 (RRT*의 핵심)
  - 재시도 예산   : 샘플 수를 sweep 할 수 있게 파라미터화

A*와의 공정성
  - 비용은 반드시 cost/energy.py 를 쓴다 (같은 비용 함수)
  - 충돌 검사는 같은 높이맵을 참조한다
  - 다만 RRT*는 연속 공간에서 동작하므로 격자 제약이 없다.
    이건 RRT*에게 유리한 조건이고, 의도적으로 그렇게 둔다.

상태 표현 (연속)
    3D: (x, y, mode)        mode: 0=ground, 1=air
    4D: (x, y, z, mode)     z는 연속 고도
"""
from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass, field

import numpy as np

from cost.energy import CostAccumulator
from planners.state_space import ProblemSpec, GROUND, AIR
from planners.grid_search import SearchResult


@dataclass
class Node:
    x: float
    y: float
    z: float          # 3D에서는 지형+clearance 로 자동 결정된 값을 넣어둔다
    mode: int
    parent: int | None = None      # 부모 노드 인덱스
    acc: CostAccumulator = field(default_factory=CostAccumulator)  # 루트부터 누적
    cost: float = 0.0              # 가중치 적용 비용 (= model.cost(acc))
    children: set[int] = field(default_factory=set)


class HybridRRTStar:
    """Hybrid RRT*.

    사용법:
        planner = HybridRRTStar(spec, seed=0)
        result = planner.plan(max_samples=5000, timeout=2.0)
    """

    def __init__(self, spec: ProblemSpec, seed: int = 0,
                 goal_bias: float = 0.1,
                 ground_sample_ratio: float = 0.8,
                 step_size: float = 1.0,
                 gamma_rrt: float = 12.0,
                 goal_tolerance: float = 0.3,
                 informed: bool = True):
        self.spec = spec
        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)

        # hybrid_rrt_params.yaml 대응 파라미터
        self.goal_bias = goal_bias
        self.ground_sample_ratio = ground_sample_ratio
        self.step_size = step_size          # steer 최대 전진 거리 (m)
        self.gamma_rrt = gamma_rrt          # rewire 반경 계수
        self.goal_tolerance = goal_tolerance
        self.informed = informed

        self.hm = spec.hm
        self.model = spec.model

        self.nodes: list[Node] = []
        self.best_goal_idx: int | None = None
        self.best_cost = float("inf")
        # 첫 해를 찾기까지 걸린 시간. timeout까지 계속 개선하므로
        # runtime_s 만 보면 항상 예산과 같아져서 '얼마나 빨리 찾았나'를 잴 수 없다.
        self.first_solution_s: float | None = None

        # 공간 해싱 — 이웃 탐색을 O(n) 선형 순회에서 O(1) 근사로 바꾼다.
        # 전체 노드를 매번 훑으면 전체가 O(n^2)이 되어 4만 샘플에서 수 분이 걸린다.
        # 셀 크기는 rewire 반경보다 크게 잡아야 인접 셀 9개만 봐도 충분하다.
        self._bucket_size = max(step_size, 1.0)
        self._buckets: dict[tuple[int, int, int], list[int]] = {}

    # ------------------------------------------------------------------
    # 기본 유틸
    # ------------------------------------------------------------------
    def _terrain(self, x: float, y: float) -> float:
        ix, iy = self.hm.to_idx(x, y)
        if not self.hm.in_bounds(ix, iy):
            return float("inf")
        return float(self.hm.grid[iy, ix])

    def _ground_ok(self, x: float, y: float) -> bool:
        ix, iy = self.hm.to_idx(x, y)
        return self.spec.ground_ok(ix, iy)

    def _air_ok(self, x: float, y: float, z: float) -> bool:
        ix, iy = self.hm.to_idx(x, y)
        if not self.hm.in_bounds(ix, iy):
            return False
        h = self._terrain(x, y)
        if self.spec.dims == 3:
            return h + self.spec.flight_clearance <= self.spec.z_max
        return (z >= h + self.model.min_flight_clearance) and (z <= self.spec.z_max)

    def _air_z(self, x: float, y: float, z_hint: float) -> float:
        """이 지점에서 실제로 날게 되는 고도.

        3D는 지형에 따라 자동 결정. 4D는 z_hint 를 쓰되 지형 여유를 보장.
        """
        h = self._terrain(x, y)
        if self.spec.dims == 3:
            return h + self.spec.flight_clearance
        lo = h + self.model.min_flight_clearance
        return min(max(z_hint, lo), self.spec.z_max)

    # ------------------------------------------------------------------
    # 충돌 검사 — 두 점 사이를 잘게 나눠 확인
    # ------------------------------------------------------------------
    def _collision_free(self, a: Node, x2: float, y2: float, z2: float,
                        mode: int) -> bool:
        # 보간 간격을 셀 크기의 절반 이하로 잡는다.
        # A*의 corner-cutting 버그와 같은 이유 — 간격이 성기면 두 샘플 사이의
        # 얇은 벽을 그냥 지나쳐 버린다. 여기서는 벽 최소 두께(0.6m)보다
        # 훨씬 촘촘하므로 관통이 발생하지 않는다.
        n = max(2, int(math.hypot(x2 - a.x, y2 - a.y) / (self.hm.resolution * 0.4)) + 1)
        for i in range(n + 1):
            t = i / n
            x = a.x + (x2 - a.x) * t
            y = a.y + (y2 - a.y) * t
            if mode == GROUND:
                if not self._ground_ok(x, y):
                    return False
            else:
                z = a.z + (z2 - a.z) * t
                if not self._air_ok(x, y, z):
                    return False
                # 지형을 뚫고 지나가지 않는지
                if z < self._terrain(x, y) + self.model.min_flight_clearance - 1e-9:
                    return False
        return True

    # ------------------------------------------------------------------
    # 엣지 비용 — A*와 같은 energy.py 를 쓴다
    # ------------------------------------------------------------------
    def _edge_cost(self, a: Node, x2: float, y2: float, z2: float,
                   mode2: int) -> CostAccumulator | None:
        """a 에서 (x2,y2,z2,mode2) 로 가는 비용. 불가능하면 None."""
        d = math.hypot(x2 - a.x, y2 - a.y)

        # 모드 전환 (같은 위치에서만 허용)
        if a.mode != mode2:
            if d > 1e-9:
                return None
            if mode2 == AIR:
                if not self._air_ok(a.x, a.y, z2):
                    return None
                return self.model.takeoff(z2)
            else:
                if not self._ground_ok(a.x, a.y):
                    return None
                return self.model.landing(a.z)

        # 같은 모드 내 이동
        if mode2 == GROUND:
            if not self._collision_free(a, x2, y2, z2, GROUND):
                return None
            return self.model.ground_move(d)

        # 공중 이동
        # 최대 상승각 제약 — A*(state_space)와 동일한 물리 제약을 건다.
        # 이게 없으면 RRT*만 임의 경사로 오를 수 있어 불공정해진다.
        if abs(z2 - a.z) > self.model.max_dz_for(d) + 1e-9:
            return None
        if not self._collision_free(a, x2, y2, z2, AIR):
            return None
        clr = min(a.z - self._terrain(a.x, a.y), z2 - self._terrain(x2, y2))
        acc = self.model.air_move_horizontal(d, clearance=clr)
        if abs(z2 - a.z) > 1e-9:
            acc = acc + self.model.air_move_vertical(z2 - a.z)
        return acc

    # ------------------------------------------------------------------
    # 샘플링
    # ------------------------------------------------------------------
    def _sample(self) -> tuple[float, float, float, int]:
        gx, gy = self.hm.goal

        # goal bias
        if self.rng.random() < self.goal_bias:
            return gx, gy, 0.0, GROUND

        if self.informed and math.isfinite(self.best_cost):
            p = self._sample_informed()
            if p is not None:
                return p

        # 균등 샘플링
        x = self.rng.uniform(0, self.hm.width_m)
        y = self.rng.uniform(0, self.hm.height_m)
        mode = GROUND if self.rng.random() < self.ground_sample_ratio else AIR
        z = 0.0
        if mode == AIR:
            h = self._terrain(x, y)
            lo = h + self.model.min_flight_clearance
            hi = self.spec.z_max
            z = self.rng.uniform(lo, hi) if hi > lo else lo
        return x, y, z, mode

    def _sample_informed(self):
        """Informed RRT* — 현재 최선 비용으로 정의되는 타원 내부에서 샘플.

        start와 goal을 초점으로 하고, 장축이 c_best / 단위비용 인 타원.
        비용을 '거리'로 환산해야 하므로 지상 주행 단가로 나눈다.
        (이건 하한 환산이라 타원이 실제보다 커질 수 있지만, 그건 안전한 쪽이다 —
         해를 놓치지 않는다)
        """
        sx, sy = self.hm.start
        gx, gy = self.hm.goal
        c_min = math.hypot(gx - sx, gy - sy)
        unit = self.spec._unit_cost_ground
        c_best_dist = self.best_cost / unit
        if c_best_dist <= c_min:
            return None

        # 타원 파라미터
        a = c_best_dist / 2.0                     # 장반경
        b = math.sqrt(max(c_best_dist ** 2 - c_min ** 2, 0.0)) / 2.0  # 단반경
        cx, cy = (sx + gx) / 2.0, (sy + gy) / 2.0
        theta = math.atan2(gy - sy, gx - sx)

        # 단위원 -> 타원 변환
        r = math.sqrt(self.rng.random())
        phi = self.rng.random() * 2 * math.pi
        ux, uy = r * math.cos(phi), r * math.sin(phi)
        ex, ey = a * ux, b * uy
        x = cx + ex * math.cos(theta) - ey * math.sin(theta)
        y = cy + ex * math.sin(theta) + ey * math.cos(theta)

        if not (0 <= x < self.hm.width_m and 0 <= y < self.hm.height_m):
            return None

        mode = GROUND if self.rng.random() < self.ground_sample_ratio else AIR
        z = 0.0
        if mode == AIR:
            h = self._terrain(x, y)
            lo = h + self.model.min_flight_clearance
            hi = self.spec.z_max
            if hi <= lo:
                return None
            z = self.rng.uniform(lo, hi)
        return x, y, z, mode

    # ------------------------------------------------------------------
    # 트리 연산
    # ------------------------------------------------------------------
    # -- 공간 해싱 --------------------------------------------------------
    def _bucket_key(self, x: float, y: float, mode: int) -> tuple[int, int, int]:
        b = self._bucket_size
        return (int(x // b), int(y // b), mode)

    def _bucket_candidates(self, x: float, y: float, mode: int,
                           radius: float) -> list[int]:
        """반경 radius 안에 있을 수 있는 노드 인덱스들 (같은 모드만).

        해당 셀과 주변 셀만 훑는다. radius가 셀 크기보다 크면 훑는 범위를 넓힌다.
        """
        b = self._bucket_size
        span = max(1, int(math.ceil(radius / b)))
        bx, by = int(x // b), int(y // b)
        out: list[int] = []
        for dx in range(-span, span + 1):
            for dy in range(-span, span + 1):
                lst = self._buckets.get((bx + dx, by + dy, mode))
                if lst:
                    out.extend(lst)
        return out

    def _nearest(self, x: float, y: float, mode: int) -> int:
        """같은 모드 노드 중 가장 가까운 것. 없으면 전체에서 가장 가까운 것.

        공간 해싱으로 주변부터 찾되, 못 찾으면 반경을 넓혀간다.
        """
        radius = self._bucket_size
        for _ in range(8):     # 반경을 배로 늘려가며 최대 8회 시도
            cands = self._bucket_candidates(x, y, mode, radius)
            if cands:
                best_i, best_d = -1, float("inf")
                for i in cands:
                    n = self.nodes[i]
                    d = (n.x - x) ** 2 + (n.y - y) ** 2
                    if d < best_d:
                        best_d, best_i = d, i
                # 찾은 최근접이 탐색 반경 안에 있으면 확정
                if best_i >= 0 and best_d <= radius * radius:
                    return best_i
                if best_i >= 0:
                    # 반경 밖이지만 후보는 있다 — 한 단계만 더 넓혀 확인
                    radius *= 2
                    cands2 = self._bucket_candidates(x, y, mode, radius)
                    for i in cands2:
                        n = self.nodes[i]
                        d = (n.x - x) ** 2 + (n.y - y) ** 2
                        if d < best_d:
                            best_d, best_i = d, i
                    return best_i
            radius *= 2

        # 해당 모드 노드가 아예 없으면 전체에서 최근접 (드문 경우)
        best_i, best_d = -1, float("inf")
        for i, n in enumerate(self.nodes):
            d = (n.x - x) ** 2 + (n.y - y) ** 2
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    def _near_indices(self, x: float, y: float, mode: int) -> list[int]:
        """rewire 대상 이웃 — k-nearest RRT* 방식.

        원래 반경 기반 r = gamma*(log n / n)^(1/d) 을 쓰되 하한을 step_size로
        걸었더니, 노드 밀도가 올라갈수록 반경 내 이웃이 선형으로 늘어
        전체가 O(n^2)가 됐다 (2만 샘플에 34초).

        k-nearest 변형은 이웃 수를 k = k_rrt * log(n) 으로 직접 묶는다.
        Karaman & Frazzoli(2011)에서 반경 기반과 동일한 최적성 보장을 갖는
        변형으로 제시된 방식이라, 성능을 위해 이론을 포기하는 게 아니다.
            k_rrt = e * (1 + 1/d)
        """
        n = len(self.nodes)
        if n < 2:
            return []
        d_dim = self.spec.dims - 1  # mode는 이산이라 차원에서 제외
        k_rrt = math.e * (1.0 + 1.0 / d_dim)
        k = max(1, int(math.ceil(k_rrt * math.log(n))))

        # 탐색 반경은 k개를 담을 만큼만 (밀도로 역산해 과하게 넓히지 않는다)
        r = self.gamma_rrt * (math.log(n) / n) ** (1.0 / d_dim)
        r = max(r, 2.0 * self.hm.resolution)

        cands = self._bucket_candidates(x, y, mode, r)
        if len(cands) < k:
            # 후보가 모자라면 반경을 넓혀 한 번 더
            cands = self._bucket_candidates(x, y, mode, r * 3.0)

        if len(cands) <= k:
            return cands
        # 가까운 순으로 k개만
        cands.sort(key=lambda i: (self.nodes[i].x - x) ** 2
                   + (self.nodes[i].y - y) ** 2)
        return cands[:k]

    def _steer(self, frm: Node, tx: float, ty: float):
        """frm 에서 (tx,ty) 방향으로 step_size 만큼만 전진한 점."""
        d = math.hypot(tx - frm.x, ty - frm.y)
        if d <= self.step_size:
            return tx, ty
        t = self.step_size / d
        return frm.x + (tx - frm.x) * t, frm.y + (ty - frm.y) * t

    def _add_node(self, node: Node) -> int:
        idx = len(self.nodes)
        self.nodes.append(node)
        if node.parent is not None:
            self.nodes[node.parent].children.add(idx)
        self._buckets.setdefault(
            self._bucket_key(node.x, node.y, node.mode), []).append(idx)
        return idx

    def _propagate(self, idx: int):
        """부모가 바뀐 노드의 자손들 비용을 갱신 (rewire 후)."""
        stack = [idx]
        while stack:
            i = stack.pop()
            ni = self.nodes[i]
            for c in list(ni.children):
                nc = self.nodes[c]
                e = self._edge_cost(ni, nc.x, nc.y, nc.z, nc.mode)
                if e is None:
                    continue
                nc.acc = ni.acc + e
                nc.cost = self.model.cost(nc.acc)
                stack.append(c)

    # ------------------------------------------------------------------
    # 메인 루프
    # ------------------------------------------------------------------
    def plan(self, max_samples: int = 5000,
             timeout: float | None = 2.0) -> SearchResult:
        t0 = time.perf_counter()

        sx, sy = self.hm.start
        gx, gy = self.hm.goal
        if not self._ground_ok(sx, sy):
            return SearchResult(found=False, planner="RRT*",
                                runtime_s=time.perf_counter() - t0)

        # 루트 노드
        self.nodes = []
        self._buckets = {}
        self.best_goal_idx = None
        self.best_cost = float("inf")
        self.first_solution_s = None
        self._add_node(Node(x=sx, y=sy, z=0.0, mode=GROUND))

        n_samples = 0
        for it in range(max_samples):
            if timeout is not None and (it & 0x3F) == 0:
                if time.perf_counter() - t0 > timeout:
                    break
            n_samples += 1

            sx_, sy_, sz_, smode = self._sample()
            i_near = self._nearest(sx_, sy_, smode)
            near = self.nodes[i_near]

            # 모드가 다르면 전환 노드를 먼저 만든다 (같은 위치에서 이/착륙)
            if near.mode != smode:
                z_new = self._air_z(near.x, near.y, sz_) if smode == AIR else 0.0
                e = self._edge_cost(near, near.x, near.y, z_new, smode)
                if e is None:
                    continue
                acc = near.acc + e
                nd = Node(x=near.x, y=near.y, z=z_new, mode=smode,
                          parent=i_near, acc=acc, cost=self.model.cost(acc))
                i_near = self._add_node(nd)
                near = nd

            # steer
            nx_, ny_ = self._steer(near, sx_, sy_)
            nz_ = self._air_z(nx_, ny_, sz_) if smode == AIR else 0.0
            if smode == GROUND and not self._ground_ok(nx_, ny_):
                continue
            if smode == AIR and not self._air_ok(nx_, ny_, nz_):
                continue

            # --- choose parent: 이웃 중 가장 싼 부모를 고른다 (RRT*의 핵심 1) ---
            cands = self._near_indices(nx_, ny_, smode) or [i_near]
            best_parent, best_acc, best_c = None, None, float("inf")
            for ci in cands:
                cn = self.nodes[ci]
                e = self._edge_cost(cn, nx_, ny_, nz_, smode)
                if e is None:
                    continue
                acc = cn.acc + e
                c = self.model.cost(acc)
                if c < best_c:
                    best_parent, best_acc, best_c = ci, acc, c
            if best_parent is None:
                continue

            new_node = Node(x=nx_, y=ny_, z=nz_, mode=smode,
                            parent=best_parent, acc=best_acc, cost=best_c)
            i_new = self._add_node(new_node)

            # --- rewire: 새 노드를 거치면 더 싸지는 이웃을 다시 연결 (핵심 2) ---
            for ci in cands:
                if ci == best_parent:
                    continue
                cn = self.nodes[ci]
                e = self._edge_cost(new_node, cn.x, cn.y, cn.z, cn.mode)
                if e is None:
                    continue
                acc = new_node.acc + e
                c = self.model.cost(acc)
                if c < cn.cost - 1e-9:
                    # 부모 교체
                    if cn.parent is not None:
                        self.nodes[cn.parent].children.discard(ci)
                    cn.parent = i_new
                    new_node.children.add(ci)
                    cn.acc, cn.cost = acc, c
                    self._propagate(ci)

            # --- 착륙 노드 자동 생성 ---
            # 이게 없으면 비행 후 지상으로 돌아오지 못한다.
            # _nearest()가 '같은 모드' 노드를 우선 찾기 때문에,
            # GROUND 샘플은 늘 출발지 쪽 GROUND 노드에만 연결을 시도한다.
            # 그래서 장애물 건너편 상공까지 날아가도 착륙 엣지가 생기지 않아
            # (실측: 이륙 1개, 착륙 0개) 목표(지상 상태)에 영원히 도달 못 했다.
            # 새 AIR 노드에서 착륙이 가능하면 착륙 노드를 함께 만들어 준다.
            landed_idx = None
            if smode == AIR and self._ground_ok(nx_, ny_):
                e_land = self._edge_cost(new_node, nx_, ny_, 0.0, GROUND)
                if e_land is not None:
                    acc_l = new_node.acc + e_land
                    ln = Node(x=nx_, y=ny_, z=0.0, mode=GROUND,
                              parent=i_new, acc=acc_l,
                              cost=self.model.cost(acc_l))
                    landed_idx = self._add_node(ln)

            # --- 목표 도달 확인 ---
            # 목표는 지상 상태여야 한다 (착륙해서 도착)
            for cand_idx in (i_new if smode == GROUND else None, landed_idx):
                if cand_idx is None:
                    continue
                cn = self.nodes[cand_idx]
                if math.hypot(cn.x - gx, cn.y - gy) > self.goal_tolerance:
                    continue
                e = self._edge_cost(cn, gx, gy, 0.0, GROUND)
                if e is None:
                    continue
                acc = cn.acc + e
                c = self.model.cost(acc)
                if c < self.best_cost:
                    gn = Node(x=gx, y=gy, z=0.0, mode=GROUND,
                              parent=cand_idx, acc=acc, cost=c)
                    self.best_goal_idx = self._add_node(gn)
                    self.best_cost = c
                    if self.first_solution_s is None:
                        self.first_solution_s = time.perf_counter() - t0

        runtime = time.perf_counter() - t0

        if self.best_goal_idx is None:
            return SearchResult(found=False, n_expanded=n_samples,
                                n_generated=len(self.nodes),
                                runtime_s=runtime, planner="RRT*")

        # 경로 복원
        path, i = [], self.best_goal_idx
        while i is not None:
            n = self.nodes[i]
            path.append((n.x, n.y, n.z, n.mode))
            i = n.parent
        path.reverse()

        gn = self.nodes[self.best_goal_idx]
        res = SearchResult(found=True, path=path, cost=gn.cost, acc=gn.acc,
                           n_expanded=n_samples, n_generated=len(self.nodes),
                           runtime_s=runtime, planner="RRT*")
        res.first_solution_s = self.first_solution_s
        return res


def rrt_star(spec: ProblemSpec, seed: int = 0, max_samples: int = 5000,
             timeout: float | None = 2.0, **kw) -> SearchResult:
    """단발 실행 헬퍼."""
    p = HybridRRTStar(spec, seed=seed, **kw)
    r = p.plan(max_samples=max_samples, timeout=timeout)
    if not hasattr(r, "first_solution_s"):
        r.first_solution_s = p.first_solution_s
    return r


def rrt_star_best_of(spec: ProblemSpec, n_seeds: int = 5,
                     max_samples: int = 5000, timeout: float | None = 2.0,
                     **kw) -> tuple[SearchResult, list[SearchResult]]:
    """여러 시드로 돌려 최선값을 반환 (RRT*에게 유리한 조건).

    A* 우위를 주장하려면 RRT*에게 이 정도 어드밴티지를 주고도 이겨야 한다.
    반환: (최선 결과, 전체 결과 목록)
    """
    runs = [rrt_star(spec, seed=s, max_samples=max_samples, timeout=timeout, **kw)
            for s in range(n_seeds)]
    ok = [r for r in runs if r.found]
    if not ok:
        return runs[0], runs
    return min(ok, key=lambda r: r.cost), runs
