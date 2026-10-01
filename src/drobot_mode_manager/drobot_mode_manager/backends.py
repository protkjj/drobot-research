"""이착륙을 실제로 수행하는 쪽 — "어떻게 띄울지".

"언제 띄울지" 는 switch_tracker, "언제 어디에 있을지" 는 flight_profile,
"실제로 옮기기" 가 여기다. 셋을 나눈 이유는 전환이 실패했을 때
어디가 틀렸는지 갈라내기 위해서다.

왜 set_pose 인가 — 대안을 다 못 쓴다
    시뮬 로봇은 Gazebo DiffDrive 지상 차량이다. 멀티콥터 물리도 PX4 SITL 도
    없고(px4_msgs 서브모듈도 체크아웃 안 됨), /cmd_vel 로는 뜰 방법이 없다.
    남는 건 모델 자세를 직접 쓰는 것뿐이다.

    ★ 이것은 비행 시뮬레이션이 아니다 ★
    추력도 자세 제어도 없이 로봇을 그 자리에 놓는 운동학 재생이다.
    보고서에 "비행을 시뮬레이션했다" 고 쓰면 안 된다.
    "계획된 비행 구간을 실행한 것으로 간주하고 로봇을 이동시켰다" 가 맞다.
    에너지 수치는 여기서 나오지 않고 energy_params 모델에서 나온다.

두 가지 방식
    animate  프로파일을 따라 dt 마다 자세를 다시 박는다. 기본값.
    hop      이륙 대기 -> 착륙점으로 한 번 -> 착륙 대기. 자세를 2번만 쓴다.

    animate 를 기본으로 둔 이유는 보기 좋아서가 아니다. hop 으로 가만히
    기다리는 동안 Nav2 컨트롤러가 /cmd_vel 을 계속 내보내서 DiffDrive 가
    로봇을 이륙점에서 밀어낸다. 매 스텝 자세를 다시 쓰면 그게 덮인다.
    hop 은 전송이 느릴 때(CLI) 쓰는 대비책이다.

전송은 주입받는다
    set_pose 를 어떻게 보내는지는 이 파일이 모른다. 호출자가 함수를 넘긴다.
        ROS 서비스  ros_gz_interfaces/srv/SetEntityPose (빠름, 기본)
        gz CLI      subprocess 로 gz service 호출 (느림, 대비책)
    이렇게 둔 이유: 20Hz 로 자세를 쓰려면 프로세스를 띄우면 안 되는데,
    ROS 서비스를 쓰려면 bridge 설정이 필요하고 그건 bringup(공동)이다.
    둘 중 되는 쪽을 골라 쓸 수 있어야 한다. 테스트에서는 가짜를 넣는다.
"""
from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

from drobot_mode_manager.flight_profile import FlightSegment, Pose, pose_at, sample

GZ_BIN = "/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz"


class Backend:
    """공통 인터페이스. execute() 가 True 면 전환이 끝난 것으로 본다."""

    name = "base"

    def execute(self, seg: FlightSegment, log, sleep) -> bool:
        raise NotImplementedError


class NullBackend(Backend):
    """아무것도 하지 않는다 — 단계 1 의 동작.

    전환 지점 판정만 보고 싶을 때 쓴다. 로봇은 제자리에 있고,
    상태만 GROUND -> FLYING -> GROUND 로 넘어간다.
    """

    name = "null"

    def execute(self, seg: FlightSegment, log, sleep) -> bool:
        log.info(f"  [null] 실제로 움직이지 않음 "
                 f"(고도 {seg.altitude:.2f} m · 거리 {seg.distance:.2f} m · "
                 f"{seg.total_time:.1f} s 상당)")
        return True


class GazeboBackend(Backend):
    """Gazebo 모델 자세를 직접 써서 옮긴다.

    send: Callable[[Pose], bool]  — 자세 하나를 실제로 보내는 함수.
          True 를 돌려주면 성공으로 본다. 호출자가 넘긴다.
    """

    name = "gazebo"

    def __init__(self, send, style: str = "animate", dt: float = 0.05):
        self.send = send
        self.style = style
        self.dt = dt
        self.last_error = ""

    def execute(self, seg: FlightSegment, log, sleep) -> bool:
        log.info(f"  [gazebo:{self.style}] 고도 {seg.altitude:.2f} m · "
                 f"거리 {seg.distance:.2f} m · {seg.total_time:.1f} s")

        if self.style == "hop":
            # 이륙 대기 -> 착륙점(공중) -> 순항 대기 -> 지면
            sleep(seg.takeoff_time)
            if not self._put(pose_at(seg, seg.takeoff_time + seg.cruise_time), log):
                return False
            sleep(seg.cruise_time)
            if not self._put(pose_at(seg, seg.total_time), log):
                return False
            sleep(seg.landing_time)
            return True

        poses = sample(seg, self.dt)
        for i, p in enumerate(poses):
            if not self._put(p, log, i + 1, len(poses)):
                return False
            sleep(self.dt)
        return True

    def _put(self, p: Pose, log, i: int = 0, n: int = 0) -> bool:
        if self.send(p):
            return True
        where = f" ({i}/{n})" if n else ""
        log.error(f"  자세 전송 실패{where} — {self.last_error or '응답 없음'}")
        return False


def cli_sender(world: str, model: str = "drobot", gz_bin: str = GZ_BIN,
               timeout_ms: int = 2000):
    """gz service CLI 로 자세를 보내는 함수를 만든다 (대비책).

    프로세스를 매번 띄우므로 animate(20Hz)에는 못 쓴다. hop 전용이다.
    """
    bin_ = gz_bin if Path(gz_bin).exists() else (shutil.which("gz") or "gz")

    def send(p: Pose) -> bool:
        qz, qw = p.quat_zw
        req = (f'name: "{model}", '
               f'position: {{x: {p.x:.4f}, y: {p.y:.4f}, z: {p.z:.4f}}}, '
               f'orientation: {{x: 0, y: 0, z: {qz:.6f}, w: {qw:.6f}}}')
        cmd = [bin_, "service", "-s", f"/world/{world}/set_pose",
               "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
               "--timeout", str(timeout_ms), "--req", req]
        try:
            r = subprocess.run(cmd, capture_output=True, text=True,
                               timeout=timeout_ms / 1000.0 + 2.0)
        except (subprocess.TimeoutExpired, FileNotFoundError):
            return False
        return "true" in (r.stdout or "").lower()

    return send


class Px4Backend(Backend):
    """Phase 2 실물. 아직 비어 있다.

    px4_msgs · px4-ros2-interface-lib 서브모듈이 체크아웃되지 않았고
    시뮬에도 PX4 SITL 이 없다. 여기를 채우는 것이 실물 단계의 일이다.
    """

    name = "px4"

    def execute(self, seg: FlightSegment, log, sleep) -> bool:
        log.error("  [px4] 미구현 — px4_msgs 서브모듈과 SITL 이 필요하다")
        return False


def make_backend(name: str, send=None, style: str = "animate",
                 dt: float = 0.05) -> Backend:
    """name: sim|gazebo -> Gazebo, px4 -> PX4(미구현), 그 외 -> 아무것도 안 함."""
    if name in ("sim", "gazebo"):
        if send is None:
            raise ValueError("gazebo 백엔드에는 자세 전송 함수가 필요하다")
        return GazeboBackend(send, style, dt)
    if name == "px4":
        return Px4Backend()
    return NullBackend()
