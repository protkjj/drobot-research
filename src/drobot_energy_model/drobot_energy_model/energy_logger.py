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
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
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
    energy_j: float = 0.0         # 구간 누적 에너지 (J) — 구간마다 리셋
    energy_total_j: float = 0.0   # 전원 투입 후 총 에너지 (J) — 리셋 안 함
    last_stamp: float | None = None

    def update(self, v: float, i: float, now: float) -> None:
        """새 측정값을 반영하고 사다리꼴 적분으로 에너지를 누적한다."""
        p_new = v * i
        if self.last_stamp is not None:
            dt = now - self.last_stamp
            # 샘플이 튀거나 멈췄을 때 이상값이 섞이지 않도록 상한을 둔다
            if 0.0 < dt < 1.0:
                # 사다리꼴: 이전 전력과 현재 전력의 평균 × dt
                de = 0.5 * (self.power + p_new) * dt
                self.energy_j += de
                # 배터리 잔량 추정용. 구간이 바뀌어도 이어져야 하므로
                # reset_energy() 가 건드리지 않는다.
                self.energy_total_j += de
        self.voltage = v
        self.current = i
        self.power = p_new
        self.last_stamp = now

    @property
    def has_data(self) -> bool:
        """측정값을 한 번이라도 받았나. INA226 이 없으면 끝까지 False."""
        return self.last_stamp is not None

    def reset_energy(self) -> None:
        """구간 에너지만 0으로. energy_total_j 는 그대로 둔다."""
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
        # ---- /battery_state 발행 (인터페이스 규약 C -> C) ----
        # mode_manager 가 이륙 전에 배터리를 확인한다. 그 입력이 이 토픽이다.
        self.declare_parameter("battery_capacity_wh", 100.0)   # <- 실측
        self.declare_parameter("battery_initial_percentage", 1.0)
        self.declare_parameter("battery_publish_hz", 1.0)
        # 팩 전압을 읽을 채널. logic 은 5V 강압 뒤라 팩 전압이 아니다.
        self.declare_parameter("pack_channel", "air")
        # ina226 | reference — reference 는 INA226 없이 표로 전력을 합성한다
        self.declare_parameter("source", "ina226")
        self.declare_parameter("pack_voltage_nominal", 22.2)
        for _k, _v in (("air_idle", 0.0), ("air_active", 50.0),
                       ("rover_idle", 0.5), ("rover_active", 18.0),
                       ("servo_idle", 0.6), ("servo_active", 6.0),
                       ("logic_idle", 5.0), ("logic_active", 5.0)):
            self.declare_parameter(f"reference_power.{_k}", _v)

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

        # ---- 배터리 상태 발행 ----
        self.cap_wh = float(self.get_parameter("battery_capacity_wh").value)
        self.batt_init = float(self.get_parameter("battery_initial_percentage").value)
        self.pack_channel = self.get_parameter("pack_channel").value
        hz = float(self.get_parameter("battery_publish_hz").value)
        self.batt_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self.create_timer(1.0 / hz, self._publish_battery)

        # ---- reference 모드 ----
        self.source = self.get_parameter("source").value
        self.v_nom = float(self.get_parameter("pack_voltage_nominal").value)
        self.ref_w = {k: float(self.get_parameter(f"reference_power.{k}").value)
                      for k in ("air_idle", "air_active", "rover_idle",
                                "rover_active", "servo_idle", "servo_active",
                                "logic_idle", "logic_active")}
        self.mode_state = "GROUND"     # mode_manager 의 /mode_state
        self.driving = False           # /cmd_vel 이 0 이 아닌가
        if self.source == "reference":
            self.create_subscription(String, "/mode_state",
                                     self._on_mode_state, 10)
            self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 20)
            self.create_timer(0.01, self._synth)     # 100Hz, INA226 과 같은 속도
            self.get_logger().warn(
                "source=reference — INA226 없이 표로 전력을 합성한다. "
                "여기서 나오는 에너지는 측정값이 아니라 추정치다.")

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
    def _on_mode_state(self, msg: String) -> None:
        self.mode_state = msg.data

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.driving = abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3

    def _synth(self) -> None:
        """INA226 대신 표에서 전력을 만들어 넣는다.

        실제 센서와 같은 경로(ChannelState.update)로 넣는 이유는,
        나중에 source=ina226 으로 바꿔도 적분·CSV·배터리 계산이
        그대로 쓰이게 하기 위해서다. 바뀌는 건 입력뿐이다.
        """
        flying = self.mode_state in ("TAKING_OFF", "FLYING", "LANDING")
        now = time.time()
        watts = {
            "air":   self.ref_w["air_active"] if flying else self.ref_w["air_idle"],
            "rover": (self.ref_w["rover_active"]
                      if (not flying and self.driving) else self.ref_w["rover_idle"]),
            "servo": self.ref_w["servo_active"] if flying else self.ref_w["servo_idle"],
            "logic": self.ref_w["logic_active"],
        }
        for c, w in watts.items():
            # logic 은 5V 강압 뒤 계통이라 팩 전압이 아니다
            v = 5.0 if c == "logic" else self.v_nom
            self.channels[c].update(v, w / v, now)

    def _publish_battery(self) -> None:
        """배터리 상태를 내보낸다 — mode_manager 가 이륙 가부를 판단하는 근거.

        잔량은 쿨롱 카운팅(소비 에너지 적산)으로 낸다. 전압 곡선으로 추정하는
        방법도 있지만 리튬 팩은 중간 구간이 평탄해서 오차가 크다.
        대신 "시작할 때 battery_initial_percentage 였다" 는 가정이 들어간다.

        INA226 이 아직 없으면 present=False 로 내보낸다.
        이게 중요한 이유: 측정값이 0 이면 "아무것도 안 썼다" 가 되어
        잔량 100% 로 보인다. 그 상태로 mode_manager 가 이륙을 허가하면
        측정 장비가 없다는 사실이 "배터리 가득" 으로 둔갑한다.
        그래서 모른다는 것을 명시적으로 알린다.
        """
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        # 측정값인지 추정값인지를 메시지에 박아둔다. 받는 쪽(mode_manager)과
        # 나중에 로그를 보는 사람이 구분할 수 있어야 한다.
        msg.serial_number = ("REFERENCE-NOT-MEASURED"
                             if self.source == "reference" else "INA226")

        live = [c for c in CHANNELS if self.channels[c].has_data]
        if not live:
            msg.present = False
            msg.percentage = float("nan")
            msg.voltage = float("nan")
            msg.current = float("nan")
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
            self.batt_pub.publish(msg)
            return

        msg.present = True
        pack = self.channels.get(self.pack_channel)
        msg.voltage = float(pack.voltage) if pack and pack.has_data else float("nan")

        # 총 전력은 네 채널의 합 (파일 상단 '검산' 참고).
        # 팩 전류는 총 전력 / 팩 전압으로 환산한다 — 채널마다 측정 지점의
        # 전압이 달라서 전류를 그냥 더하면 안 되기 때문이다.
        p_total = sum(self.channels[c].power for c in CHANNELS)
        if msg.voltage and msg.voltage == msg.voltage and msg.voltage > 1e-6:
            msg.current = -float(p_total / msg.voltage)   # 방전은 음수 (ROS 규약)
        else:
            msg.current = float("nan")

        used_wh = sum(self.channels[c].energy_total_j for c in CHANNELS) / 3600.0
        remain = self.cap_wh * self.batt_init - used_wh
        msg.percentage = float(max(0.0, min(1.0, remain / self.cap_wh)))
        msg.charge = float("nan")          # Ah 는 셀 구성을 몰라 환산 불가
        msg.capacity = float("nan")
        msg.design_capacity = float("nan")
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING if p_total > 0.0
            else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING)
        self.batt_pub.publish(msg)

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
