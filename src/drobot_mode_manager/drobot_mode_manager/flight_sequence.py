"""비행 한 구간의 순서 — 변신 → 비행 → 복귀 변신. ROS 도 Gazebo 도 모른다.

왜 mode_manager 가 변신을 부르나
    변신은 비행 순서의 한 단계다. 암을 펴기 전에 뜨면 안 되고, 착륙해서 암을
    접기 전에 바퀴로 달리면 안 된다. 그 순서를 아는 곳이 mode_manager 하나라
    호출도 여기서 한다. 암을 '어떻게' 돌리는지는 transform_manager 가,
    '어떻게' 나는지는 백엔드가 맡는다. 이 모듈은 '어떤 순서로' 만 정한다.

    예전(2026-10-05~11-20)에는 drobot_controller 의 mode_switch_executor 가
    같은 /mode_switch_points 를 따로 받아 변신을 불렀다. 두 노드가 서로를
    몰라서 이륙점 0.9 m 앞에서 변신이, 0.35 m 에서 이륙이 따로 걸렸고 변신이
    끝나기 전에 이륙할 수 있었다 (documents/REQUIREMENTS_TRACE.md D-6).

실패 처리 — HANDOFF 의 비행 담당자 인터페이스 계약과 같다
    이륙 전 변신 실패
        require=True   뜨지 않는다. 반쯤 펴졌을 수 있으니 로버로 되돌린다.
        require=False  경고만 하고 계속한다. 시뮬에서는 암이 마찰 때문에
                       목표각에 늘 못 미쳐(잔차 0.15~0.30 rad) transform_manager
                       가 매번 timeout 을 돌려주므로, 막으면 아무것도 못 한다.
    비행 실패
        로버로 되돌리고 실패를 돌려준다. 그 자리에 드론 형태로 서 있으면
        지상에서 복구할 수단이 없다.
    착륙 후 복귀 변신 실패
        비행 자체는 끝났다 (로봇은 착륙점에 있다). 그래서 구간은 성공으로 두고
        rover_ready=False 로 알린다. 지상 주행을 다시 풀지는 부른 쪽이 정한다.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

# 변신 호출: (성공했나, 메시지)
TransformCall = Callable[[], "tuple[bool, str]"]


@dataclass
class SequenceResult:
    ok: bool                # 비행 구간을 성공으로 봐도 되나 (착륙점에 내려왔나)
    flew: bool              # 백엔드 비행을 시작했나
    rover_ready: bool       # 끝났을 때 로버 형태인가 (바퀴로 달려도 되나)
    steps: list[str] = field(default_factory=list)      # 실행한 순서 (로그·테스트용)
    warnings: list[str] = field(default_factory=list)


def run_segment(to_drone: TransformCall,
                fly: Callable[[], bool],
                to_rover: TransformCall,
                require_transform: bool) -> SequenceResult:
    """이륙점에서 착륙점까지 한 구간을 순서대로 수행한다."""
    r = SequenceResult(ok=False, flew=False, rover_ready=True)

    up, msg = to_drone()
    r.steps.append("to_drone")
    if not up:
        r.warnings.append(f"이륙 전 변신 실패 — {msg}")
        if require_transform:
            back, bmsg = to_rover()
            r.steps.append("to_rover")
            r.rover_ready = back
            if not back:
                r.warnings.append(f"로버 복귀도 실패 — {bmsg}")
            return r

    # 여기서부터는 드론 형태로 본다 (변신 실패를 경고로 넘긴 경우도 포함).
    r.rover_ready = False
    r.flew = True
    flown = fly()
    r.steps.append("fly")

    # 비행이 실패해도 접는다 — 위 '비행 실패' 참고
    down, dmsg = to_rover()
    r.steps.append("to_rover")
    r.rover_ready = down
    if not down:
        r.warnings.append(f"착륙 후 로버 복귀 실패 — {dmsg}")

    r.ok = flown
    return r


class TakeoffAttempts:
    """같은 이륙 자리에서 몇 번 실패했는지 센다 — 재계획에도 유지된다.

    왜 필요한가
        실패하면 전환점이 그대로 남고, 로봇은 이륙점에 서 있으므로 다음 /odom
        에서 곧바로 다시 '도달' 한다. 변신 실패처럼 매번 똑같이 실패하는 원인이면
        몇 초마다 끝없이 재시도한다 (2026-11-20 연기 시험에서 3 s 주기로 확인).

    왜 pair_id 가 아니라 자리로 세나
        플래너는 주행 중 0.5 s 마다 재계획해 /mode_switch_points 를 다시 낸다.
        pair_id 는 계획마다 0 부터 다시 붙으므로 기억의 열쇠가 될 수 없다. 같은
        장애물 앞의 이륙점은 재계획해도 거의 같은 자리에 찍힌다. 격자 경계에
        걸쳐 흔들리면 다른 자리로 세어 몇 번 더 시도할 수 있다.

    limit <= 0 이면 제한 없음 (예전 동작). 노드를 다시 띄우면 초기화된다.
    """

    def __init__(self, limit: int, cell: float = 0.25):
        self.limit = int(limit)
        self.cell = float(cell)
        self.count: dict[tuple[int, int], int] = {}

    def _key(self, x: float, y: float) -> tuple[int, int]:
        return (round(x / self.cell), round(y / self.cell))

    def allowed(self, x: float, y: float) -> bool:
        return self.limit <= 0 or self.count.get(self._key(x, y), 0) < self.limit

    def failed(self, x: float, y: float) -> int:
        """실패를 하나 더하고 그 자리의 누적 실패 횟수를 돌려준다."""
        k = self._key(x, y)
        self.count[k] = self.count.get(k, 0) + 1
        return self.count[k]
