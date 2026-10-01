"""비행 구간의 시간·자세 프로파일 — ROS 도 Gazebo 도 모른다.

무엇을 계산하나
    이륙점과 착륙점, 목표 고도가 주어지면 "언제 어디에 있어야 하는가"를 낸다.
        1) 이륙  제자리에서 고도 alt 까지        takeoff_time 초
        2) 순항  고도를 유지한 채 수평 이동      거리 / speed 초
        3) 착륙  제자리에서 지면까지             landing_time 초

왜 떼어냈나
    백엔드(Gazebo / PX4)가 달라져도 "언제 어디" 는 같다. 섞어두면
    백엔드를 바꿀 때마다 타이밍을 다시 검증해야 한다.
    그리고 이렇게 두면 시뮬 없이 맥북에서 검산된다.

주의 — 이것은 비행 역학이 아니다
    가속·바람·추력 한계를 전혀 모형화하지 않은 등속 보간이다.
    "로봇을 이 시각에 이 자리에 놓는다" 는 운동학 재생이다.
    에너지 수치는 여기서 나오지 않고 energy_params 의 모델에서 나온다.
    실제 비행 역학이 필요해지면 PX4 백엔드로 가야 한다.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose:
    """프로파일 위의 한 점. t 는 구간 시작으로부터의 초."""
    t: float
    x: float
    y: float
    z: float
    yaw: float

    @property
    def quat_zw(self) -> tuple[float, float]:
        """yaw 만 있는 쿼터니언의 (z, w). x=y=0 이다."""
        return math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)


@dataclass(frozen=True)
class FlightSegment:
    """한 비행 구간의 입력."""
    x0: float
    y0: float
    x1: float
    y1: float
    altitude: float
    yaw: float = 0.0
    takeoff_time: float = 5.0
    landing_time: float = 4.0
    speed: float = 0.5           # m/s, 수평 순항

    @property
    def distance(self) -> float:
        return math.dist((self.x0, self.y0), (self.x1, self.y1))

    @property
    def cruise_time(self) -> float:
        # speed 가 0 이면 영원히 안 끝난다. 설정 실수를 여기서 막는다.
        return self.distance / self.speed if self.speed > 1e-6 else 0.0

    @property
    def total_time(self) -> float:
        return self.takeoff_time + self.cruise_time + self.landing_time


def pose_at(seg: FlightSegment, t: float) -> Pose:
    """구간 시작 후 t 초일 때 있어야 할 자리.

    t 가 범위를 벗어나면 양 끝으로 잘라낸다 — 호출자가 시계를 조금
    넘기더라도 엉뚱한 외삽이 나오지 않게.
    """
    t = max(0.0, min(t, seg.total_time))
    t_up, t_cr = seg.takeoff_time, seg.cruise_time

    if t <= t_up:                                   # 1) 이륙
        f = t / t_up if t_up > 1e-6 else 1.0
        return Pose(t, seg.x0, seg.y0, seg.altitude * f, seg.yaw)

    if t <= t_up + t_cr:                            # 2) 순항
        f = (t - t_up) / t_cr if t_cr > 1e-6 else 1.0
        return Pose(t,
                    seg.x0 + (seg.x1 - seg.x0) * f,
                    seg.y0 + (seg.y1 - seg.y0) * f,
                    seg.altitude, seg.yaw)

    f = (t - t_up - t_cr) / seg.landing_time if seg.landing_time > 1e-6 else 1.0
    return Pose(t, seg.x1, seg.y1, seg.altitude * (1.0 - f), seg.yaw)   # 3) 착륙


def sample(seg: FlightSegment, dt: float = 0.05) -> list[Pose]:
    """dt 간격으로 전 구간을 떠낸다 (animate 백엔드용)."""
    n = max(1, int(round(seg.total_time / dt)))
    poses = [pose_at(seg, i * dt) for i in range(n)]
    poses.append(pose_at(seg, seg.total_time))      # 끝점은 반드시 포함
    return poses
