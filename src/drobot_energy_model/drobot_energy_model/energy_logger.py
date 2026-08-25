#!/usr/bin/env python3
"""에너지 로거 — INA226 4채널 전력을 기록하고 구간별로 적분한다.

목적: Phase 2 실측 데이터를 JMP 회귀분석에 바로 넣을 수 있는 형태로 만든다.

두 종류의 파일을 쓴다.

1) raw_log.csv       — 원본 보관용. 100Hz 시계열 그대로.
2) segment_summary.csv — JMP 분석용. 한 행이 하나의 실험 구간.

왜 나누는가:
    100Hz raw 를 JMP 에 바로 넣고 회귀하면 행이 수십만 개가 되고
    모델 해석도 어려워진다. 구간별로 적분해서 한 행으로 만들면
    "이 조건에서 이만큼 썼다"가 바로 회귀 대상이 된다.

측정 채널 (INA226 4개)
    AIR   : ESC 4개 전체 입력 — 비행 전력
    ROVER : DRV8871 2개 전체 입력 — 주행 전력
    SERVO : UBEC 입력 — 변형 arm 서보 전력
    LOGIC : Pololu 5V 계통 — Teensy/센서 전력

    검산: PM02D 총 전력 ≈ AIR + ROVER + SERVO + LOGIC

idle power 보정
    측정 에너지에는 Pixhawk/Teensy/센서 등 기본 소비전력이 포함된다.
    이걸 그대로 E_motion 에 넣고 비용함수의 시간 항도 더하면
    시간이 이중 반영된다. 그래서 idle 구간을 따로 재서 빼야 한다.
    `~/energy_logger/measure_idle` 서비스로 idle 측정 모드에 들어간다.

사용법
    ros2 run drobot_energy_model energy_logger

    # 구간 시작/종료 (실험 스크립트나 손으로 호출)
    ros2 topic pub --once /energy/segment_control std_msgs/String \\
      "data: 'start|run01|seg03|ground|speed=0.3,slope=5.0'"
    ros2 topic pub --once /energy/segment_control std_msgs/String "data: 'stop'"
"""
from __future__ import annotations

import csv
import os
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

# 채널 이름 — INA226 4개
CHANNELS = ("air", "rover", "servo", "logic")


@dataclass
class ChannelState:
    """한 채널의 최신 측정값과 적분 상태."""

    voltage: float = 0.0
    current: float = 0.0
    power: float = 0.0            # W
    energy_j: float = 0.0         # 구간 누적 에너지 (J)
    last_stamp: float | None = None

    def update(self, v: float, i: float, now: float) -> None:
        """새 측정값을 반영하고 사다리꼴 적분으로 에너지를 누적한다."""
        p_new = v * i
        if self.last_stamp is not None:
            dt = now - self.last_stamp
            # 샘플이 튀거나 멈췄을 때 이상값이 섞이지 않도록 상한을 둔다
            if 0.0 < dt < 1.0:
                # 사다리꼴: 이전 전력과 현재 전력의 평균 × dt
                self.energy_j += 0.5 * (self.power + p_new) * dt
        self.voltage = v
        self.current = i
        self.power = p_new
        self.last_stamp = now

    def reset_energy(self) -> None:
        self.energy_j = 0.0


@dataclass
class Segment:
    """한 실험 구간. segment_summary.csv 의 한 행이 된다."""

    run_id: str
    segment_id: str
    mode: str                     # ground / air / switch_r2a / switch_a2r / idle
    t_start: float
    meta: dict = field(default_factory=dict)   # speed, slope 등 실험 조건
    v_samples: list = field(default_factory=list)


class EnergyLogger(Node):
    def __init__(self) -> None:
        super().__init__("energy_logger")

        self.declare_parameter("data_root", "data")
        self.declare_parameter("raw_log_enabled", True)
        # INA226 노드가 어떤 토픽으로 내보내는지에 맞춰 조정한다.
        # 기본은 채널별로 voltage/current 를 따로 받는 형태.
        self.declare_parameter("topic_prefix", "/ina226")

        data_root = Path(self.get_parameter("data_root").value)
        self.raw_enabled = bool(self.get_parameter("raw_log_enabled").value)
        prefix = self.get_parameter("topic_prefix").value

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out_dir = data_root / stamp
        self.out_dir.mkdir(parents=True, exist_ok=True)

        self.channels = {c: ChannelState() for c in CHANNELS}
        self.segment: Segment | None = None
        self.idle_power = {c: 0.0 for c in CHANNELS}   # measure_idle 로 채운다

        # ---- 구독: 채널마다 전압/전류 ----
        # INA226 드라이버가 어떤 메시지를 쓰는지에 따라 여기를 맞추면 된다.
        # 지금은 std_msgs/Float32 두 개(전압, 전류)를 가정한다.
        for c in CHANNELS:
            self.create_subscription(
                Float32, f"{prefix}/{c}/voltage",
                lambda msg, ch=c: self._on_voltage(ch, msg), 50)
            self.create_subscription(
                Float32, f"{prefix}/{c}/current",
                lambda msg, ch=c: self._on_current(ch, msg), 50)

        # ---- 구간 제어 ----
        # 형식: "start|run_id|segment_id|mode|k=v,k=v"  또는  "stop"
        self.create_subscription(
            String, "/energy/segment_control", self._on_control, 10)

        # ---- idle 측정 서비스 ----
        self.create_service(Trigger, "~/measure_idle", self._on_measure_idle)

        # ---- 출력 파일 ----
        self.raw_path = self.out_dir / "raw_log.csv"
        self.seg_path = self.out_dir / "segment_summary.csv"
        self._init_csv()

        # 100Hz 로 raw 기록
        self.create_timer(0.01, self._tick)

        self.get_logger().info(f"에너지 로거 시작. 출력 디렉터리: {self.out_dir}")
        self.get_logger().info(
            "구간 시작:  ros2 topic pub --once /energy/segment_control "
            "std_msgs/String \"data: 'start|run01|seg01|ground|speed=0.3'\"")

    # ------------------------------------------------------------------
    # CSV
    # ------------------------------------------------------------------
    def _init_csv(self) -> None:
        if self.raw_enabled:
            with open(self.raw_path, "w", newline="") as f:
                w = csv.writer(f)
                cols = ["timestamp", "run_id", "segment_id", "mode"]
                for c in CHANNELS:
                    cols += [f"V_{c}", f"I_{c}", f"P_{c}"]
                cols.append("P_total")
                w.writerow(cols)

        with open(self.seg_path, "w", newline="") as f:
            w = csv.writer(f)
            # JMP 에 그대로 import 하는 테이블.
            # 설명변수(speed, slope 등)는 meta 로 들어온 것을 뒤에 붙인다.
            w.writerow([
                "run_id", "segment_id", "mode",
                "t_start", "t_end", "duration_s",
                "E_air_J", "E_rover_J", "E_servo_J", "E_logic_J", "E_total_J",
                "E_total_Wh",
                "E_air_corrected_J", "E_rover_corrected_J",
                "Vbat_mean", "Vbat_min", "Vbat_max",
                "meta",
            ])

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------
    def _on_voltage(self, ch: str, msg: Float32) -> None:
        st = self.channels[ch]
        st.update(float(msg.data), st.current, time.time())
        if ch == "air" and self.segment is not None:
            self.segment.v_samples.append(float(msg.data))

    def _on_current(self, ch: str, msg: Float32) -> None:
        st = self.channels[ch]
        st.update(st.voltage, float(msg.data), time.time())

    def _on_control(self, msg: String) -> None:
        parts = msg.data.split("|")
        cmd = parts[0].strip().lower()

        if cmd == "stop":
            self._finish_segment()
            return

        if cmd != "start":
            self.get_logger().warn(f"알 수 없는 명령: {msg.data}")
            return

        if self.segment is not None:
            self.get_logger().warn("이전 구간이 열려 있다. 먼저 닫는다.")
            self._finish_segment()

        run_id = parts[1] if len(parts) > 1 else "run"
        seg_id = parts[2] if len(parts) > 2 else "seg"
        mode = parts[3] if len(parts) > 3 else "unknown"
        meta = {}
        if len(parts) > 4 and parts[4]:
            for kv in parts[4].split(","):
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    meta[k.strip()] = v.strip()

        for st in self.channels.values():
            st.reset_energy()
        self.segment = Segment(
            run_id=run_id, segment_id=seg_id, mode=mode,
            t_start=time.time(), meta=meta)
        self.get_logger().info(f"구간 시작: {run_id}/{seg_id} mode={mode} {meta}")

    def _on_measure_idle(self, request, response):
        """현재 전력을 idle 기준으로 저장한다.

        로봇을 정지 상태로 두고 호출할 것. 5초간 평균을 낸다.
        """
        del request
        samples = {c: [] for c in CHANNELS}
        t_end = time.time() + 5.0
        self.get_logger().info("idle 측정 시작 — 5초간 로봇을 정지 상태로 둘 것")
        while time.time() < t_end:
            for c in CHANNELS:
                samples[c].append(self.channels[c].power)
            time.sleep(0.02)

        for c in CHANNELS:
            vals = [v for v in samples[c] if v > 0.0]
            self.idle_power[c] = sum(vals) / len(vals) if vals else 0.0

        txt = ", ".join(f"{c}={self.idle_power[c]:.2f}W" for c in CHANNELS)
        self.get_logger().info(f"idle 전력: {txt}")
        response.success = True
        response.message = txt
        return response

    # ------------------------------------------------------------------
    # 기록
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if not self.raw_enabled:
            return
        seg = self.segment
        row = [
            f"{time.time():.4f}",
            seg.run_id if seg else "",
            seg.segment_id if seg else "",
            seg.mode if seg else "",
        ]
        p_total = 0.0
        for c in CHANNELS:
            st = self.channels[c]
            row += [f"{st.voltage:.4f}", f"{st.current:.4f}", f"{st.power:.4f}"]
            p_total += st.power
        row.append(f"{p_total:.4f}")

        with open(self.raw_path, "a", newline="") as f:
            csv.writer(f).writerow(row)

    def _finish_segment(self) -> None:
        seg = self.segment
        if seg is None:
            return

        t_end = time.time()
        dur = t_end - seg.t_start

        e = {c: self.channels[c].energy_j for c in CHANNELS}
        e_total = sum(e.values())

        # idle 보정 — 비용함수의 시간 항과 이중 계상되지 않게 뺀다
        e_air_corr = max(0.0, e["air"] - self.idle_power["air"] * dur)
        e_rover_corr = max(0.0, e["rover"] - self.idle_power["rover"] * dur)

        vs = seg.v_samples
        v_mean = sum(vs) / len(vs) if vs else 0.0
        v_min = min(vs) if vs else 0.0
        v_max = max(vs) if vs else 0.0

        meta_str = ";".join(f"{k}={v}" for k, v in seg.meta.items())

        with open(self.seg_path, "a", newline="") as f:
            csv.writer(f).writerow([
                seg.run_id, seg.segment_id, seg.mode,
                f"{seg.t_start:.4f}", f"{t_end:.4f}", f"{dur:.4f}",
                f"{e['air']:.4f}", f"{e['rover']:.4f}",
                f"{e['servo']:.4f}", f"{e['logic']:.4f}", f"{e_total:.4f}",
                f"{e_total / 3600.0:.6f}",
                f"{e_air_corr:.4f}", f"{e_rover_corr:.4f}",
                f"{v_mean:.4f}", f"{v_min:.4f}", f"{v_max:.4f}",
                meta_str,
            ])

        self.get_logger().info(
            f"구간 종료: {seg.run_id}/{seg.segment_id} "
            f"{dur:.2f}s, {e_total:.1f}J ({e_total/3600.0:.4f}Wh) "
            f"| air={e['air']:.1f} rover={e['rover']:.1f} "
            f"servo={e['servo']:.1f} logic={e['logic']:.1f}")
        self.segment = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EnergyLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._finish_segment()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
