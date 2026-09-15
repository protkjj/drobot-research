"""격자 탐색 플래너 — Dijkstra 와 A*.

둘을 한 파일에 둔 이유: A*는 Dijkstra에 휴리스틱 h(n)만 더한 것이라는 게
코드로 바로 보이게 하기 위해서다. 실제로 `_search()` 하나를 공유하고
휴리스틱 함수만 갈아끼운다.

    Dijkstra: 우선순위 = g(n)          (시작점부터의 실제 비용)
    A*      : 우선순위 = g(n) + h(n)   (h는 목표까지의 비용 하한 추정)

h(n) = 0 이면 A*는 Dijkstra와 완전히 같아진다. 이 성질을 검증에 쓴다.
"""
from __future__ import annotations

import heapq
import time
from dataclasses import dataclass, field

from cost.energy import CostAccumulator
from planners.state_space import ProblemSpec


@dataclass
class SearchResult:
    """플래너 실행 결과. 모든 플래너가 이 형식으로 반환한다."""

    found: bool
    path: list[tuple] = field(default_factory=list)
    cost: float = float("inf")           # 가중치 적용된 비용 C
    acc: CostAccumulator | None = None   # 비용 분해 (에너지/시간/거리)
    n_expanded: int = 0                  # 확장한 노드 수 (탐색 효율 지표)
    n_generated: int = 0                 # 생성한 노드 수
    runtime_s: float = 0.0
    planner: str = ""
    peak_open: int = 0                   # open list 최대 크기 (메모리 지표)
    timed_out: bool = False
    # RRT* 전용: 첫 해를 찾기까지 걸린 시간.
    # RRT*는 timeout까지 해를 계속 개선하므로 runtime_s는 늘 예산과 같다.
    # '얼마나 빨리 첫 해에 도달했나'를 보려면 이 값이 필요하다.
    first_solution_s: float | None = None

    def brief(self) -> str:
        if not self.found:
            return f"{self.planner}: 실패 ({self.runtime_s:.3f}s)"
        return (f"{self.planner}: C={self.cost:.3f} "
                f"E={self.acc.e_total:.2f}Wh T={self.acc.time_s:.1f}s "
                f"전환={self.acc.n_switches} "
                f"확장={self.n_expanded:,} {self.runtime_s:.3f}s")


def _search(spec: ProblemSpec, use_heuristic: bool, name: str,
            timeout: float | None = None) -> SearchResult:
    """Dijkstra / A* 공용 탐색 루틴.

    use_heuristic=False -> Dijkstra
    use_heuristic=True  -> A*
    """
    t0 = time.perf_counter()

    start = spec.start_state()
    goal = spec.goal_state()

    # 시작/목표가 애초에 유효한지
    if not spec.ground_ok(start[0], start[1]):
        return SearchResult(found=False, planner=name,
                            runtime_s=time.perf_counter() - t0)

    # g[s] = 시작점에서 s까지의 최소 비용 (스칼라)
    g: dict[tuple, float] = {start: 0.0}
    # 비용 분해도 함께 추적 (경로 복원 후 재계산해도 되지만, 여기서 들고 가면 검증이 쉽다)
    g_acc: dict[tuple, CostAccumulator] = {start: CostAccumulator()}
    parent: dict[tuple, tuple] = {}
    closed: set[tuple] = set()

    h0 = spec.heuristic(start) if use_heuristic else 0.0
    # (f, tie_breaker, state) — tie_breaker는 상태 튜플 비교를 피하기 위한 카운터
    counter = 0
    open_heap: list[tuple[float, int, tuple]] = [(h0, counter, start)]

    n_expanded = 0
    n_generated = 1
    peak_open = 1
    timed_out = False

    while open_heap:
        # 타임아웃 체크는 매 반복이 아니라 주기적으로 (perf_counter 호출도 비용)
        if timeout is not None and (n_expanded & 0x3FF) == 0:
            if time.perf_counter() - t0 > timeout:
                timed_out = True
                break

        f, _, s = heapq.heappop(open_heap)
        if s in closed:
            continue
        closed.add(s)
        n_expanded += 1

        if s == goal:
            # 경로 복원
            path = [s]
            while path[-1] in parent:
                path.append(parent[path[-1]])
            path.reverse()
            return SearchResult(
                found=True, path=path, cost=g[s], acc=g_acc[s],
                n_expanded=n_expanded, n_generated=n_generated,
                runtime_s=time.perf_counter() - t0, planner=name,
                peak_open=peak_open,
            )

        gs = g[s]
        for nxt, edge_acc in spec.neighbors(s):
            if nxt in closed:
                continue
            n_generated += 1
            new_acc = g_acc[s] + edge_acc
            new_g = spec.model.cost(new_acc)
            if new_g < g.get(nxt, float("inf")) - 1e-12:
                g[nxt] = new_g
                g_acc[nxt] = new_acc
                parent[nxt] = s
                hn = spec.heuristic(nxt) if use_heuristic else 0.0
                counter += 1
                heapq.heappush(open_heap, (new_g + hn, counter, nxt))
                if len(open_heap) > peak_open:
                    peak_open = len(open_heap)

    return SearchResult(
        found=False, n_expanded=n_expanded, n_generated=n_generated,
        runtime_s=time.perf_counter() - t0, planner=name,
        peak_open=peak_open, timed_out=timed_out,
    )


def dijkstra(spec: ProblemSpec, timeout: float | None = None) -> SearchResult:
    """참조 최적해. 휴리스틱 없이 전수 탐색하므로 반드시 최적이다.

    A*/RRT* 의 최적성 갭을 재는 기준선으로 쓴다.
    """
    return _search(spec, use_heuristic=False, name="Dijkstra", timeout=timeout)


def astar(spec: ProblemSpec, timeout: float | None = None) -> SearchResult:
    """A*. admissible 휴리스틱을 쓰므로 Dijkstra와 같은 비용의 해를 낸다.

    (경로 자체는 동률 때문에 다를 수 있지만 비용은 같아야 한다)
    """
    return _search(spec, use_heuristic=True, name="A*", timeout=timeout)
