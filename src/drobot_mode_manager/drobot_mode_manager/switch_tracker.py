"""전환 판정 로직 — ROS 를 모른다.

왜 떼어냈나
    패키지 가이드 04 절: "독립적으로 테스트 가능한 로직은 별도 파일로.
    버그가 탐색 문제인지 비용 문제인지 분리된다."
    여기서도 같다. 전환이 안 일어났을 때 원인이 둘로 갈린다.
        ① 지점을 잘못 집었다        ← 이 파일의 책임
        ② 띄우는 데 실패했다        ← 백엔드의 책임
    섞어두면 구분이 안 된다. 그래서 판정만 여기에 두고,
    ROS 배선과 실제 이착륙은 mode_manager.py 가 맡는다.

    부수 효과로 rclpy 없이 돌아가므로, 기록된 주행 JSON 을 그대로
    흘려보내 검증할 수 있다 (test/replay_recorded_run.py).

상태 기계
    GROUND ──이륙점 도달──> TAKING_OFF ──백엔드 완료──> FLYING
      ^                                                  │
      └── (쿨다운) ── LANDING <──착륙점 도달────────────────┘
"""
from __future__ import annotations

import math
from dataclasses import dataclass

# drobot_msgs/ModeSwitchPoint 의 상수와 같은 값. 여기서 다시 정의하는 이유는
# 이 파일이 ROS 메시지를 import 하지 않기 위해서다.
GROUND_TO_AIR = 0
AIR_TO_GROUND = 1

GROUND, TAKING_OFF, FLYING, LANDING = "GROUND", "TAKING_OFF", "FLYING", "LANDING"


@dataclass(frozen=True)
class SwitchPoint:
    """ModeSwitchPoint 에서 판정에 필요한 것만 추린 것."""
    x: float
    y: float
    switch_type: int
    flight_altitude: float
    energy_wh: float
    pair_id: int

    @property
    def is_takeoff(self) -> bool:
        return self.switch_type == GROUND_TO_AIR

    @property
    def label(self) -> str:
        return "이륙" if self.is_takeoff else "착륙"


@dataclass
class Arrival:
    """전환점에 닿았을 때 일어난 일."""
    index: int
    total: int
    point: SwitchPoint
    error_m: float        # 전환점과 실제 위치의 거리
    speed: float
    stopped: bool         # 정지 판정을 통과했나
    skipped: bool         # 상태가 안 맞아 건너뛴 경우
    state_from: str
    state_to: str


class SwitchTracker:
    def __init__(self, arrival_radius: float = 0.35,
                 velocity_threshold: float = 0.05,
                 cooldown_after_landing: float = 3.0,
                 max_flight_distance: float = 5.0,
                 min_ceiling_clearance: float = 0.5):
        self.arrival_radius = arrival_radius
        self.velocity_threshold = velocity_threshold
        self.cooldown_after_landing = cooldown_after_landing
        self.max_flight_distance = max_flight_distance
        self.min_ceiling_clearance = min_ceiling_clearance

        self.state = GROUND
        self.points: list[SwitchPoint] = []
        self.idx = 0
        self.landed_at: float | None = None
        self.closest = math.inf      # 다음 전환점까지 최소 접근 거리 (진단용)

    # ---------- 계획 ----------
    def set_plan(self, points: list[SwitchPoint]) -> bool:
        """새 계획을 받아들인다. 받아들였으면 True.

        Nav2 는 주행 중에도 재계획한다(기록을 보면 35~45회). 비행 중에
        계획이 바뀌면 진행 중인 전환이 꼬이므로 지상일 때만 갈아끼운다.
        """
        if self.state != GROUND:
            return False
        self.points = list(points)
        self.idx = 0
        self.closest = math.inf
        return True

    def validate(self) -> list[tuple[str, str]]:
        """계획이 C 의 실행 한계 안에 있는지 본다.

        플래너(B)는 비용만 보고 계획한다. 1회 비행거리와 천장 여유는
        실행 쪽 제약이라 여기서 걸러야 한다 — max_flight_distance 가
        플래너와 이쪽 yaml 에 모두 있는 이유다(SSOT).

        돌려주는 것: [(수준, 메시지)] — 수준은 error / warn / info
        """
        out: list[tuple[str, str]] = []
        pairs: dict[int, dict[str, SwitchPoint]] = {}
        for pt in self.points:
            pairs.setdefault(pt.pair_id, {})["up" if pt.is_takeoff else "down"] = pt

        for pid, d in sorted(pairs.items()):
            if "up" not in d or "down" not in d:
                have = "이륙" if "up" in d else "착륙"
                out.append(("error", f"쌍 {pid}: {have}만 있고 짝이 없다 — 계획이 불완전하다"))
                continue
            up, dn = d["up"], d["down"]
            dist = math.dist((up.x, up.y), (dn.x, dn.y))
            energy = up.energy_wh + dn.energy_wh
            note = ""
            if dist > self.max_flight_distance:
                note += f"  ⚠ 1회 비행거리 초과 (한계 {self.max_flight_distance} m)"
            if up.flight_altitude < self.min_ceiling_clearance:
                note += f"  ⚠ 천장 여유 미달 (최소 {self.min_ceiling_clearance} m)"
            out.append((
                "warn" if note else "info",
                f"쌍 {pid}: ({up.x:.2f}, {up.y:.2f}) → ({dn.x:.2f}, {dn.y:.2f})  "
                f"거리 {dist:.2f} m · 고도 {up.flight_altitude:.2f} m · "
                f"{energy:.2f} Wh{note}"))
        return out

    # ---------- 주행 중 판정 ----------
    def update(self, t: float, x: float, y: float, speed: float) -> Arrival | None:
        """현재 위치를 넣는다. 전환이 일어났으면 Arrival, 아니면 None.

        t 는 초 단위 실수면 무엇이든 된다 (ROS 시계든 기록 시각이든).
        """
        if self.idx >= len(self.points):
            return None
        if self.landed_at is not None:
            if t - self.landed_at < self.cooldown_after_landing:
                return None
            self.landed_at = None

        tgt = self.points[self.idx]
        d = math.dist((x, y), (tgt.x, tgt.y))
        self.closest = min(self.closest, d)
        if d > self.arrival_radius:
            return None

        was = self.state
        want = GROUND if tgt.is_takeoff else FLYING
        if was != want:
            # 이륙을 못 했는데 착륙점에 닿는 식. 계획과 실제가 어긋난 상태다.
            ev = Arrival(self.idx, len(self.points), tgt, d, speed,
                         speed <= self.velocity_threshold, True, was, was)
            self._advance(t, landed=False)
            return ev

        self.state = TAKING_OFF if tgt.is_takeoff else LANDING
        return Arrival(self.idx, len(self.points), tgt, d, speed,
                       speed <= self.velocity_threshold, False, was, self.state)

    def complete(self, t: float) -> str:
        """백엔드가 이착륙을 끝냈다고 알릴 때 부른다. 다음 상태를 돌려준다."""
        if self.state == TAKING_OFF:
            self.state = FLYING
            self._advance(t, landed=False)
        elif self.state == LANDING:
            self.state = GROUND
            self._advance(t, landed=True)
        return self.state

    def _advance(self, t: float, landed: bool):
        self.idx += 1
        self.closest = math.inf
        if landed:
            self.landed_at = t

    # ---------- 진단 ----------
    @property
    def done(self) -> bool:
        return self.idx >= len(self.points)

    @property
    def remaining(self) -> int:
        return max(0, len(self.points) - self.idx)
