#!/usr/bin/env python3
"""변신 → 비행 → 복귀 변신 순서 테스트 — ROS 없이 돈다.

변신·비행을 가짜 함수로 바꿔, 무엇을 어떤 순서로 불렀는지와 실패했을 때
어디서 멈추는지만 본다.
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drobot_mode_manager.flight_sequence import TakeoffAttempts, run_segment  # noqa: E402


class Calls:
    """부른 순서를 기록하는 가짜 변신·비행."""

    def __init__(self, up=True, fly=True, down=True):
        self.log = []
        self._up, self._fly, self._down = up, fly, down

    def to_drone(self):
        self.log.append("to_drone")
        return self._up, "드론 변신 완료" if self._up else "timeout"

    def fly(self):
        self.log.append("fly")
        return self._fly

    def to_rover(self):
        self.log.append("to_rover")
        return self._down, "로버 변신 완료" if self._down else "timeout"

    def run(self, require):
        return run_segment(self.to_drone, self.fly, self.to_rover, require)


def test_normal_order():
    """변신을 끝낸 뒤에 날고, 착륙한 뒤에 접는다."""
    c = Calls()
    r = c.run(require=True)
    assert c.log == ["to_drone", "fly", "to_rover"]
    assert r.steps == c.log
    assert r.ok and r.flew and r.rover_ready
    assert r.warnings == []


def test_takeoff_transform_fails_required():
    """변신이 실패하면 뜨지 않고, 반쯤 펴졌을 수 있으니 되돌린다."""
    c = Calls(up=False)
    r = c.run(require=True)
    assert c.log == ["to_drone", "to_rover"]
    assert not r.flew and not r.ok
    assert r.rover_ready
    assert "이륙 전 변신 실패" in r.warnings[0]


def test_takeoff_transform_fails_not_required():
    """시뮬 설정(require=False)에서는 경고만 하고 그대로 난다."""
    c = Calls(up=False)
    r = c.run(require=False)
    assert c.log == ["to_drone", "fly", "to_rover"]
    assert r.flew and r.ok
    assert "이륙 전 변신 실패" in r.warnings[0]


def test_flight_fails_still_folds():
    """비행이 실패해도 로버로 되돌린다 — 드론 형태로 서 있으면 복구 수단이 없다."""
    c = Calls(fly=False)
    r = c.run(require=True)
    assert c.log == ["to_drone", "fly", "to_rover"]
    assert r.flew and not r.ok
    assert r.rover_ready


def test_landing_transform_fails():
    """착륙 뒤 복귀 변신 실패는 구간 실패가 아니다. 로봇은 이미 착륙점에 있다."""
    c = Calls(down=False)
    r = c.run(require=True)
    assert r.ok
    assert not r.rover_ready
    assert "착륙 후 로버 복귀 실패" in r.warnings[0]


def test_rollback_also_fails():
    """되돌리기까지 실패하면 그것도 알린다."""
    c = Calls(up=False, down=False)
    r = c.run(require=True)
    assert not r.flew and not r.rover_ready
    assert len(r.warnings) == 2


def test_sim_today():
    """오늘 시뮬 상태 — 암이 목표각에 못 가서 두 변신 모두 timeout.

    require=False 면 그래도 비행 구간은 끝까지 간다 (막으면 아무것도 못 한다).
    """
    c = Calls(up=False, down=False)
    r = c.run(require=False)
    assert c.log == ["to_drone", "fly", "to_rover"]
    assert r.ok and r.flew
    assert not r.rover_ready
    assert len(r.warnings) == 2


def test_attempts_stop_after_limit():
    """같은 자리에서 limit 번 실패하면 더 시도하지 않는다."""
    a = TakeoffAttempts(limit=3)
    for n in (1, 2, 3):
        assert a.allowed(1.975, 3.825)
        assert a.failed(1.975, 3.825) == n
    assert not a.allowed(1.975, 3.825)


def test_attempts_survive_replan_jitter():
    """재계획으로 이륙점이 몇 cm 흔들려도 같은 자리로 센다."""
    a = TakeoffAttempts(limit=2)
    a.failed(1.975, 3.825)
    a.failed(1.990, 3.810)
    assert not a.allowed(1.980, 3.830)


def test_attempts_other_place_is_separate():
    """다른 장애물 앞의 이륙점은 따로 센다."""
    a = TakeoffAttempts(limit=1)
    a.failed(1.975, 3.825)
    assert a.allowed(2.075, 8.775)


def test_attempts_unlimited():
    """limit 0 은 예전 동작 — 제한 없음."""
    a = TakeoffAttempts(limit=0)
    for _ in range(10):
        a.failed(1.0, 1.0)
    assert a.allowed(1.0, 1.0)


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__, "-v"]))
