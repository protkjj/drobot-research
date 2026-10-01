#!/usr/bin/env python3
"""비행 프로파일과 백엔드 단위 테스트 — ROS 도 Gazebo 도 없이 돈다.

Gazebo 호출은 set_pose 를 가짜로 바꿔치기해서 '무엇을 몇 번 불렀는지'만 본다.
실제로 로봇이 뜨는지는 시뮬에서 확인할 일이고, 여기서 보는 것은
"언제 어디로 옮기라고 시켰는가" 다.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drobot_mode_manager.backends import (        # noqa: E402
    GazeboBackend, NullBackend, Px4Backend, cli_sender, make_backend)
from drobot_mode_manager.flight_profile import (  # noqa: E402
    FlightSegment, pose_at, sample)

# base_map_h0.5 의 실제 계획과 같은 구간
SEG = FlightSegment(x0=1.53, y0=1.62, x1=1.33, y1=4.53, altitude=0.8,
                    yaw=1.5708, takeoff_time=5.0, landing_time=4.0, speed=0.5)


class FakeLog:
    def __init__(self): self.lines = []
    def info(self, m): self.lines.append(("info", m))
    def warn(self, m): self.lines.append(("warn", m))
    def error(self, m): self.lines.append(("error", m))


def test_segment_timing():
    """거리 2.92 m / 0.5 m/s = 5.83 s 순항, 총 14.83 s."""
    assert abs(SEG.distance - 2.9169) < 1e-3
    assert abs(SEG.cruise_time - 5.8338) < 1e-3
    assert abs(SEG.total_time - 14.8338) < 1e-3


def test_profile_endpoints():
    """시작은 이륙점 지면, 끝은 착륙점 지면."""
    p0, p1 = pose_at(SEG, 0.0), pose_at(SEG, SEG.total_time)
    assert (abs(p0.x - 1.53) < 1e-6 and abs(p0.y - 1.62) < 1e-6
            and abs(p0.z) < 1e-6)
    assert (abs(p1.x - 1.33) < 1e-6 and abs(p1.y - 4.53) < 1e-6
            and abs(p1.z) < 1e-6)


def test_profile_three_phases():
    """이륙 중엔 제자리에서 고도만, 순항 중엔 고도 유지하며 수평 이동."""
    mid_up = pose_at(SEG, 2.5)                     # 이륙 절반
    assert abs(mid_up.x - 1.53) < 1e-6 and abs(mid_up.z - 0.4) < 1e-6

    top = pose_at(SEG, 5.0)                        # 이륙 완료
    assert abs(top.z - 0.8) < 1e-6

    mid_cr = pose_at(SEG, 5.0 + SEG.cruise_time / 2)
    assert abs(mid_cr.z - 0.8) < 1e-6              # 고도 유지
    assert abs(mid_cr.x - (1.53 + 1.33) / 2) < 1e-6

    mid_dn = pose_at(SEG, 5.0 + SEG.cruise_time + 2.0)   # 착륙 절반
    assert abs(mid_dn.x - 1.33) < 1e-6
    assert abs(mid_dn.z - 0.4) < 1e-6


def test_profile_clamps_out_of_range():
    """시계가 조금 넘쳐도 외삽하지 않는다."""
    assert pose_at(SEG, -5.0).z == 0.0
    assert abs(pose_at(SEG, 999.0).y - 4.53) < 1e-6


def test_profile_never_goes_underground():
    """전 구간에서 z >= 0. 지면 아래로 내려가면 로봇이 바닥에 박힌다."""
    assert all(p.z >= -1e-9 for p in sample(SEG, 0.01))


def test_zero_speed_does_not_hang():
    """speed=0 설정 실수가 무한 순항이 되지 않는다."""
    s = FlightSegment(0, 0, 0, 5, altitude=0.8, speed=0.0)
    assert s.cruise_time == 0.0 and math.isfinite(s.total_time)


def test_quaternion_from_yaw():
    """yaw 만 있는 쿼터니언. 90도면 z=w=sin45=0.7071."""
    p = pose_at(FlightSegment(0, 0, 1, 0, altitude=0.5, yaw=math.pi / 2), 0.0)
    qz, qw = p.quat_zw
    assert abs(qz - 0.70711) < 1e-4 and abs(qw - 0.70711) < 1e-4


def test_hop_moves_once_at_the_end():
    """hop 은 비행 시간을 기다린 뒤 착륙점으로 한 번만 옮긴다.

    중간에 공중으로 올려두면 Nav2 가 "로봇이 지도 밖 허공에 있다" 는
    상태로 돌아가다 0.3초 만에 죽는다 (backends.py 주석 참고).
    자세 변경이 한 번이어야 그 모순 구간이 없다.
    """
    sent, slept = [], []
    be = GazeboBackend(lambda p: (sent.append(p), True)[1], style="hop")
    assert be.execute(SEG, FakeLog(), slept.append)

    assert len(sent) == 1, "자세를 두 번 이상 바꾸면 안 된다"
    assert abs(sent[0].z) < 1e-6                        # 지면에 내려놓는다
    assert abs(sent[0].x - 1.33) < 1e-6 and abs(sent[0].y - 4.53) < 1e-6
    # 대기 시간 합이 구간 총 시간과 같아야 한다 (에너지·시간 집계가 이걸 쓴다)
    assert abs(sum(slept) - SEG.total_time) < 1e-6


def test_animate_walks_the_profile():
    """animate 는 dt 간격으로 전부 보낸다."""
    sent = []
    be = GazeboBackend(lambda p: (sent.append(p), True)[1], style="animate", dt=0.5)
    assert be.execute(SEG, FakeLog(), lambda _: None)
    assert len(sent) == len(sample(SEG, 0.5))
    assert abs(sent[-1].z) < 1e-6


def test_set_pose_failure_is_reported():
    """set_pose 가 실패하면 False 를 돌려주고 로그에 남긴다."""
    be = GazeboBackend(lambda p: False, style="hop")
    log = FakeLog()
    assert be.execute(SEG, log, lambda _: None) is False
    assert any(lv == "error" for lv, _ in log.lines)


def test_null_backend_moves_nothing():
    log = FakeLog()
    assert NullBackend().execute(SEG, log, lambda _: None)
    assert "움직이지 않음" in log.lines[0][1]


def test_px4_backend_refuses():
    """미구현을 성공으로 위장하지 않는다."""
    log = FakeLog()
    assert Px4Backend().execute(SEG, log, lambda _: None) is False
    assert any(lv == "error" for lv, _ in log.lines)


def test_make_backend_dispatch():
    ok = lambda p: True            # noqa: E731
    assert make_backend("sim", ok).name == "gazebo"
    assert make_backend("px4").name == "px4"
    assert make_backend("none").name == "null"
    # 전송 함수 없이 gazebo 를 만들려 하면 바로 막는다
    try:
        make_backend("sim")
        raise AssertionError("send 없이 생성이 통과했다")
    except ValueError:
        pass


def test_cli_sender_builds_command(monkeypatch=None):
    """CLI 대비책이 만드는 gz 명령이 깨지지 않는다 (실행은 하지 않는다)."""
    import drobot_mode_manager.backends as bk

    captured = {}

    class FakeRun:
        stdout, stderr = "data: true", ""

    def fake_run(cmd, **kw):
        captured["cmd"] = cmd
        return FakeRun()

    real = bk.subprocess.run
    bk.subprocess.run = fake_run
    try:
        send = cli_sender("base_map_h0.5", model="drobot")
        assert send(pose_at(SEG, SEG.total_time)) is True
    finally:
        bk.subprocess.run = real

    cmd = captured["cmd"]
    assert "/world/base_map_h0.5/set_pose" in cmd
    req = cmd[cmd.index("--req") + 1]
    assert 'name: "drobot"' in req
    assert "x: 1.3300" in req and "y: 4.5300" in req and "z: 0.0000" in req
    assert "orientation" in req


def test_world_name_with_dot_is_not_a_ros_name():
    """월드 이름에 '.' 이 들어가면 ROS 서비스 경로로 못 쓴다.

    base_map_h0.5 가 실제로 그렇다. mode_manager 는 이 경우 gz CLI 로
    떨어져야 하고, 그 분기를 빼먹어 한 번 죽은 적이 있다.
    여기서는 '왜 CLI 가 필요한가' 를 규칙으로 못 박아둔다.
    """
    import re
    ros_ok = re.compile(r"^[A-Za-z_~{][A-Za-z0-9_~{}/]*$")
    assert not ros_ok.match("/world/base_map_h0.5/set_pose".lstrip("/"))
    assert ros_ok.match("/world/base_map_h05/set_pose".lstrip("/"))


if __name__ == "__main__":
    fns = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    fail = 0
    for fn in fns:
        try:
            fn(); print(f"  ✅ {fn.__name__}")
        except AssertionError as e:
            fail += 1; print(f"  ❌ {fn.__name__} — {e or 'assert 실패'}")
        except Exception as e:
            fail += 1; print(f"  ❌ {fn.__name__} — {type(e).__name__}: {e}")
    print(f"\n{len(fns) - fail}/{len(fns)} 통과")
    sys.exit(1 if fail else 0)
