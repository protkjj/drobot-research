#!/usr/bin/env python3
"""모드 전환 관리자 — 계획에 적힌 이·착륙 지점을 실제 전환으로 옮긴다.

왜 이 노드가 필요한가
    B(플래너)가 /mode_switch_points 로 "여기서 이륙, 저기서 착륙" 을 낸다.
    그런데 그걸 실행하는 주체가 없어서, 지금 로봇은 비행 구간을 만나면
    지상 컨트롤러(DWB)로 장애물을 통과하려다 실패한다. 이 노드가 그 빈칸이다.

PX4 는 아직 없다 — 확인한 사실
    시뮬에 PX4 SITL 도 멀티콥터 물리도 없다 (Gazebo 플러그인이 DiffDrive).
    px4_msgs·px4-ros2-interface-lib 서브모듈도 체크아웃되지 않았다.
    그래서 빌드 매니페스트의 "PX4 이착륙" 은 Phase 2 실물용으로 두고,
    지금은 같은 표의 C 완료 기준인 "시뮬 전환" 을 만든다.

이 파일이 하는 일 / 안 하는 일
    한다    ROS 배선 — 토픽 구독, TF 조회, 파라미터 로드, 로그
    안 한다 전환 판정 — switch_tracker.py (ROS 를 모르는 순수 로직)
            실제 이착륙 — 백엔드 (단계 2)

    이렇게 나눈 이유: 전환이 안 일어났을 때 "지점을 잘못 집은 건지,
    띄우는 데 실패한 건지" 를 구분할 수 있어야 하기 때문이다.

단계 1 = 지금
    아직 로봇을 세우지도 띄우지도 않는다. 관측하고 기록만 한다.
    검증: 기록된 주행을 흘려보내는 test/replay_recorded_run.py
"""
from __future__ import annotations

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

import tf2_ros

from drobot_msgs.msg import ModeSwitchPlan

from drobot_mode_manager.switch_tracker import SwitchPoint, SwitchTracker

# 플래너가 rclcpp::QoS(1).transient_local() 로 낸다
# (hybrid_astar_planner.cpp:130). 늦게 뜬 이 노드도 마지막 계획을 받아야
# 하므로 정확히 맞춰야 한다. 안 맞으면 연결 자체가 안 된다.
LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class ModeManager(Node):
    def __init__(self):
        super().__init__("drobot_mode_manager")

        # 값은 yaml, 코드엔 fallback 기본값만 — 패키지 가이드 06 절.
        # 진실의 출처는 config/mode_switch_params.yaml 이다.
        p = self.declare_parameter
        self.backend = p("backend", "sim").value
        # 규약(인터페이스 규약 ①)은 base_link 인데 저장소 실물은
        # base_footprint 다 (ekf.yaml:17). 정렬은 A 트랙 몫이라
        # 여기서는 파라미터로 받아 맞춰 쓴다.
        self.base_frame = p("base_frame", "base_footprint").value
        self.map_frame = p("map_frame", "map").value

        self.tracker = SwitchTracker(
            arrival_radius=p("arrival_radius", 0.35).value,
            velocity_threshold=p("velocity_threshold", 0.05).value,
            cooldown_after_landing=p("cooldown_after_landing", 3.0).value,
            max_flight_distance=p("max_flight_distance", 5.0).value,
            min_ceiling_clearance=p("min_ceiling_clearance", 0.5).value,
            battery_emergency_threshold=p("battery_emergency_threshold", 15.0).value,
            battery_safety_margin=p("battery_safety_margin", 1.2).value,
            battery_capacity_wh=p("battery_capacity_wh", 100.0).value,
            # INA226 이 아직 없다. 시뮬에서는 배터리를 몰라도 진행한다.
            # 실물에서는 True 로 올린다 — 그때는 '모르면 안 띄운다'가 맞다.
            require_battery=p("require_battery", False).value,
        )
        self.warned_tf = False
        self.warned_batt = False

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.create_subscription(ModeSwitchPlan, "/mode_switch_points",
                                 self._on_plan, LATCHED)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        # 인터페이스 규약 C -> C: energy_logger 가 내고 여기서 받는다
        self.create_subscription(BatteryState, "/battery_state",
                                 self._on_battery, 10)
        self.state_pub = self.create_publisher(String, "/mode_state", 10)

        t = self.tracker
        self.get_logger().info(
            f"모드 관리자 시작 — 백엔드 {self.backend} · "
            f"도달반경 {t.arrival_radius} m · 정지판정 {t.velocity_threshold} m/s")
        if self.backend == "sim":
            self.get_logger().info(
                "단계 1: 전환 지점 판정만 한다. 아직 세우지도 띄우지도 않는다.")

    # ---------- 계획 ----------
    def _on_plan(self, msg: ModeSwitchPlan):
        pts = [SwitchPoint(x=s.position.x, y=s.position.y,
                           switch_type=int(s.switch_type),
                           flight_altitude=s.flight_altitude,
                           energy_wh=s.estimated_energy_cost,
                           pair_id=int(s.pair_id))
               for s in msg.switch_points]

        if not self.tracker.set_plan(pts):
            self.get_logger().warn(
                f"비행 중({self.tracker.state})에 새 계획 {len(pts)}개 도착 "
                f"— 무시하고 현재 것을 유지한다")
            return

        self.get_logger().info(
            f"계획 수신 — 전환점 {len(pts)}개 · "
            f"예상 비행 에너지 {msg.total_flight_energy:.2f} Wh")
        log = {"error": self.get_logger().error,
               "warn": self.get_logger().warn,
               "info": self.get_logger().info}
        for level, text in self.tracker.validate():
            log[level]("  " + text)

    def _on_battery(self, msg: BatteryState):
        """배터리 상태를 판정기에 넘긴다.

        present=False 로 오면 '모른다'로 들어간다. 0% 로 넣지 않는 이유는
        '방전'과 '측정 장비 없음'이 전혀 다른 상황이기 때문이다.
        """
        self.tracker.set_battery(msg.percentage, msg.present)
        if not msg.present and not self.warned_batt:
            self.get_logger().warn(
                "배터리 상태 미상 (INA226 없음) — "
                f"require_battery={self.tracker.require_battery}")
            self.warned_batt = True

    # ---------- 위치 ----------
    def _robot_xy(self, odom: Odometry) -> tuple[float, float]:
        """로봇 위치를 map 기준으로. TF 가 없으면 odom 으로 떨어진다.

        전환점은 map 좌표인데 /odom 은 odom 좌표다. 원래는 TF 로 변환해야
        맞다. 그런데 지금 저장소에는 map 프레임이 없어서(A 트랙 미완)
        조회가 실패한다. 그때는 odom 을 그대로 쓰되 한 번 경고한다.
        map→odom 이 항등에 가까운 초기에는 오차가 작지만 SLAM 이 보정을
        시작하면 어긋나므로, 조용히 넘어가면 안 된다.
        """
        try:
            tf = self.tf_buf.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.05))
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception:
            if not self.warned_tf:
                self.get_logger().warn(
                    f"TF {self.map_frame}→{self.base_frame} 없음 — /odom 좌표로 "
                    f"대체한다. (map 프레임 발행은 slam_toolbox = A 트랙 몫)")
                self.warned_tf = True
            return odom.pose.pose.position.x, odom.pose.pose.position.y

    def _on_odom(self, msg: Odometry):
        self.state_pub.publish(String(data=self.tracker.state))

        x, y = self._robot_xy(msg)
        v = msg.twist.twist.linear
        speed = math.hypot(v.x, v.y)
        t = self.get_clock().now().nanoseconds * 1e-9

        ev = self.tracker.update(t, x, y, speed)
        if ev is None:
            return

        if ev.skipped:
            self.get_logger().warn(
                f"{ev.point.label}점에 닿았는데 상태가 {ev.state_from} 다 — 건너뛴다")
            return

        if ev.blocked is not None:
            # 전환점은 그대로 둔다. 배터리가 회복되거나(충전) 파라미터가
            # 바뀌면 다음 도달에서 다시 판정된다.
            self.get_logger().error(
                f"이륙 금지 ({ev.point.x:.2f}, {ev.point.y:.2f}) — {ev.blocked}")
            return

        self.get_logger().info(
            f"[{ev.index + 1}/{ev.total}] {ev.point.label}점 도달 "
            f"({ev.point.x:.2f}, {ev.point.y:.2f})  오차 {ev.error_m:.2f} m · "
            f"속도 {ev.speed:.3f} m/s · "
            f"{'정지함' if ev.stopped else '주행 중 — 단계 2 에서 정지 명령이 필요하다'}")

        # 단계 1 의 백엔드는 아무것도 하지 않고 즉시 완료를 보고한다.
        # 단계 2 에서 여기가 실제 이착륙으로 바뀐다.
        after = self.tracker.complete(t)
        alt = f" (목표 고도 {ev.point.flight_altitude:.2f} m)" if ev.point.is_takeoff else ""
        self.get_logger().info(f"  → {after}{alt}  [단계 1: 실제로 움직이지 않음]")


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 전환점을 못 잡았으면 왜 못 잡았는지 숫자로 남긴다.
        # "도달반경이 작아서" 인지 "로봇이 거기까지 못 가서" 인지 갈린다.
        t = node.tracker
        if t.points and not t.done:
            node.get_logger().warn(
                f"미처리 전환점 {t.remaining}개 — 다음 지점까지 최소 접근 "
                f"{t.closest:.2f} m (도달반경 {t.arrival_radius} m)")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
