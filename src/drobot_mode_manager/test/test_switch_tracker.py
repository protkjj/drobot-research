#!/usr/bin/env python3
"""전환 판정기 단위 테스트 — ROS 없이 돈다.

패키지 가이드 04 절("독립적으로 테스트 가능한 로직은 별도 파일 + 단위 테스트").
여기서 확인하는 것은 판정 규칙이지 시뮬레이션이 아니다.

실행
    python3 src/drobot_mode_manager/test/test_switch_tracker.py
    (또는 pytest)
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drobot_mode_manager.switch_tracker import (   # noqa: E402
    AIR_TO_GROUND, FLYING, GROUND, GROUND_TO_AIR, SwitchPoint, SwitchTracker)


def plan(up_xy=(1.5, 1.6), down_xy=(1.3, 4.5), e_up=6.6, e_down=4.6, alt=0.8):
    """base_map_h0.5 실측 기록과 같은 모양의 계획."""
    return [
        SwitchPoint(up_xy[0], up_xy[1], GROUND_TO_AIR, alt, e_up, 0),
        SwitchPoint(down_xy[0], down_xy[1], AIR_TO_GROUND, alt, e_down, 0),
    ]


def test_arrival_and_cycle():
    """정상 흐름: 이륙점 도달 -> FLYING -> 착륙점 도달 -> GROUND."""
    tr = SwitchTracker()
    assert tr.set_plan(plan())

    assert tr.update(0.0, 0.0, 0.0, 0.0) is None          # 멀리 있음
    ev = tr.update(1.0, 1.5, 1.6, 0.0)                    # 이륙점 도달
    assert ev is not None and not ev.skipped and ev.blocked is None
    assert ev.point.is_takeoff and ev.stopped
    assert tr.complete(1.0) == FLYING

    assert tr.update(2.0, 1.3, 4.5, 0.0) is not None      # 착륙점 도달
    assert tr.complete(2.0) == GROUND
    assert tr.done


def test_cooldown_blocks_immediate_retrigger():
    """착륙 직후 쿨다운 동안에는 판정을 멈춘다."""
    tr = SwitchTracker(cooldown_after_landing=3.0)
    tr.set_plan(plan() + [SwitchPoint(1.3, 4.5, GROUND_TO_AIR, 0.8, 6.6, 1)])
    tr.update(0.0, 1.5, 1.6, 0.0); tr.complete(0.0)
    tr.update(1.0, 1.3, 4.5, 0.0); tr.complete(1.0)        # 착륙 -> 쿨다운 시작
    assert tr.update(2.0, 1.3, 4.5, 0.0) is None           # 1초 뒤 -> 아직
    assert tr.update(5.0, 1.3, 4.5, 0.0) is not None       # 4초 뒤 -> 풀림


def test_arrival_radius_is_respected():
    """도달반경 밖에서는 전환이 일어나지 않는다."""
    tr = SwitchTracker(arrival_radius=0.35)
    tr.set_plan(plan())
    assert tr.update(0.0, 1.5, 2.0, 0.0) is None           # 0.40 m -> 밖
    assert abs(tr.closest - 0.40) < 1e-6
    assert tr.update(1.0, 1.5, 1.9, 0.0) is not None       # 0.30 m -> 안


def test_validate_flags_limits():
    """1 회 비행거리와 천장 여유를 넘기면 경고가 난다."""
    tr = SwitchTracker(max_flight_distance=2.0, min_ceiling_clearance=1.0)
    tr.set_plan(plan())                                    # 거리 2.91, 고도 0.80
    levels = [lv for lv, _ in tr.validate()]
    assert "warn" in levels
    text = tr.validate()[0][1]
    assert "비행거리 초과" in text and "천장 여유 미달" in text

    tr2 = SwitchTracker()                                  # 기본 한계 5.0 / 0.5
    tr2.set_plan(plan())
    assert tr2.validate()[0][0] == "info"                  # 둘 다 통과


def test_validate_catches_unpaired():
    """이륙만 있고 착륙이 없으면 error."""
    tr = SwitchTracker()
    tr.set_plan([SwitchPoint(1.5, 1.6, GROUND_TO_AIR, 0.8, 6.6, 0)])
    lv, text = tr.validate()[0]
    assert lv == "error" and "짝이 없다" in text


def test_battery_unknown_passes_in_sim():
    """INA226 이 없으면(기본값) 통과시킨다 — 시뮬을 막지 않기 위해."""
    tr = SwitchTracker()
    tr.set_plan(plan())
    tr.set_battery(None, present=False)
    ev = tr.update(0.0, 1.5, 1.6, 0.0)
    assert ev is not None and ev.blocked is None


def test_battery_unknown_blocks_on_hardware():
    """require_battery=True 면 모르는 상태로는 안 띄운다."""
    tr = SwitchTracker(require_battery=True)
    tr.set_plan(plan())
    tr.set_battery(float("nan"), present=True)             # NaN 도 '모름'
    ev = tr.update(0.0, 1.5, 1.6, 0.0)
    assert ev is not None and ev.blocked is not None
    assert tr.state == GROUND                              # 전이하지 않았다


def test_battery_emergency_threshold():
    """비상 임계 아래면 이륙 금지."""
    tr = SwitchTracker(battery_emergency_threshold=15.0, battery_capacity_wh=100.0)
    tr.set_plan(plan())
    tr.set_battery(0.10, present=True)                     # 10%
    ev = tr.update(0.0, 1.5, 1.6, 0.0)
    assert ev.blocked and "비상 임계" in ev.blocked
    assert tr.state == GROUND

    # 같은 이유로는 한 번만 알린다 (로그 폭주 방지)
    assert tr.update(0.1, 1.5, 1.6, 0.0) is None


def test_battery_safety_margin():
    """잔량이 1 사이클 소모량 x 여유에 못 미치면 금지."""
    # 1 사이클 = 6.6 + 4.6 = 11.2 Wh, 여유 1.2 -> 13.44 Wh 필요
    tr = SwitchTracker(battery_capacity_wh=100.0, battery_safety_margin=1.2,
                       battery_emergency_threshold=0.0)
    tr.set_plan(plan())

    tr.set_battery(0.13, present=True)                     # 13 Wh < 13.44 -> 금지
    ev = tr.update(0.0, 1.5, 1.6, 0.0)
    assert ev.blocked and "잔량" in ev.blocked
    assert tr.state == GROUND

    tr.set_battery(0.14, present=True)                     # 14 Wh > 13.44 -> 허용
    ev = tr.update(1.0, 1.5, 1.6, 0.0)
    assert ev is not None and ev.blocked is None
    assert tr.state == "TAKING_OFF"


def test_battery_does_not_block_landing():
    """비행 중이면 배터리가 바닥나도 착륙은 막지 않는다."""
    tr = SwitchTracker(battery_emergency_threshold=15.0)
    tr.set_plan(plan())
    tr.set_battery(1.0, present=True)
    tr.update(0.0, 1.5, 1.6, 0.0); tr.complete(0.0)        # 이륙
    assert tr.state == FLYING

    tr.set_battery(0.01, present=True)                     # 1% 로 급락
    ev = tr.update(1.0, 1.3, 4.5, 0.0)
    assert ev is not None and ev.blocked is None           # 착륙은 통과
    assert tr.complete(1.0) == GROUND


def test_plan_rejected_while_flying():
    """비행 중에 새 계획이 오면 받아들이지 않는다 (재계획 중 꼬임 방지)."""
    tr = SwitchTracker()
    tr.set_plan(plan())
    tr.update(0.0, 1.5, 1.6, 0.0); tr.complete(0.0)
    assert tr.state == FLYING
    assert tr.set_plan(plan(up_xy=(9.0, 9.0))) is False
    assert tr.points[0].x == 1.5                           # 옛 계획 유지


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    fail = 0
    for fn in fns:
        try:
            fn()
            print(f"  ✅ {fn.__name__}")
        except AssertionError as e:
            fail += 1
            print(f"  ❌ {fn.__name__}  — {e or 'assert 실패'}")
        except Exception as e:
            fail += 1
            print(f"  ❌ {fn.__name__}  — {type(e).__name__}: {e}")
    print(f"\n{len(fns) - fail}/{len(fns)} 통과")
    sys.exit(1 if fail else 0)
