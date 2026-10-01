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
    한다    ROS 배선 — 토픽 구독, TF 조회, 파라미터 로드, 자세 전송, 로그
    안 한다 전환 판정   switch_tracker.py   (ROS 를 모름)
            비행 타이밍 flight_profile.py   (ROS 도 Gazebo 도 모름)
            이착륙 실행 backends.py         (전송 방식을 주입받음)

    넷으로 나눈 이유: 전환이 안 됐을 때 "지점을 잘못 집었나 / 타이밍이
    틀렸나 / 못 띄웠나 / 전송이 실패했나" 를 갈라내기 위해서다.
    덤으로 시뮬 없이 맥북에서 앞의 셋이 전부 검증된다.

자세 전송
    ros_gz_interfaces/srv/SetEntityPose 를 먼저 쓰고, 없으면 gz CLI 로
    떨어진다. 서비스는 bridge 설정이 필요한데 그 파일이 bringup(공동)이라
    없을 수도 있기 때문이다. 떨어질 때는 style 을 hop 으로 낮춘다
    (CLI 는 프로세스를 매번 띄워서 20Hz 를 못 낸다).
"""
from __future__ import annotations

import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

import tf2_ros

from drobot_msgs.msg import ModeSwitchPlan

from drobot_mode_manager.backends import cli_sender, make_backend
from drobot_mode_manager.flight_profile import FlightSegment, Pose
from drobot_mode_manager.switch_tracker import GROUND, SwitchPoint, SwitchTracker

try:
    from ros_gz_interfaces.srv import SetEntityPose
except ImportError:          # bridge 패키지가 없으면 CLI 로 간다
    SetEntityPose = None

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
        self.yaw = 0.0                 # 자세를 쓸 때 방향을 보존하려고 추적
        self.flight_result: bool | None = None   # 백엔드 스레드가 채운다
        self.flight_seg: FlightSegment | None = None

        # ---- 비행 실행 설정 ----
        self.world = p("world", "base_map_h0.5").value
        self.model = p("model_name", "drobot").value
        self.style = p("flight_style", "animate").value
        self.dt = float(p("flight_dt", 0.05).value)
        # energy_params.yaml 과 같아야 하는 값들. 중복이라 시작할 때 대조한다.
        self.takeoff_time = float(p("takeoff_time", 5.0).value)
        self.landing_time = float(p("landing_time", 4.0).value)
        self.flight_speed = float(p("flight_speed", 0.5).value)
        # 비행 중 지상 컨트롤러를 묶어두는 비율 (%). 0 은 쓸 수 없다 —
        # Nav2 에서 speed_limit 0.0 은 NO_SPEED_LIMIT, 즉 '제한 해제' 다.
        self.flight_speed_pct = float(p("flight_speed_limit_pct", 1.0).value)
        self.speed_topic = p("speed_limit_topic", "/speed_limit").value

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.create_subscription(ModeSwitchPlan, "/mode_switch_points",
                                 self._on_plan, LATCHED)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        # 인터페이스 규약 C -> C: energy_logger 가 내고 여기서 받는다
        self.create_subscription(BatteryState, "/battery_state",
                                 self._on_battery, 10)
        self.state_pub = self.create_publisher(String, "/mode_state", 10)
        # 비행 중 지상 컨트롤러 묶기. 목표를 취소하는 대신 속도 상한을 내린다 —
        # 취소하면 목표를 보낸 쪽(record_run)이 'canceled' 로 받아 실험이 끊긴다.
        self.speed_pub = self.create_publisher(SpeedLimit, self.speed_topic, 10)

        t = self.tracker
        self.get_logger().info(
            f"모드 관리자 시작 — 백엔드 {self.backend} · "
            f"도달반경 {t.arrival_radius} m · 정지판정 {t.velocity_threshold} m/s")
        self._check_ssot()
        self._setup_backend()
        # 비행 스레드 결과를 메인 스레드에서 회수한다 (tracker 를 한 쪽에서만 건드리려고)
        self.create_timer(0.1, self._finish_flight)

    # ---------- SSOT ----------
    def _check_ssot(self):
        """energy_params.yaml 과 겹치는 값이 어긋나지 않았는지 본다.

        takeoff_time·landing_time·비행 속도는 B 의 비용 모델과 C 의 실행이
        같은 값을 써야 한다. 다르면 "계획은 5초로 계산했는데 실행은 8초"
        같은 상태가 조용히 생긴다. 패키지 가이드 06 절이 경고하는 바로 그것이라,
        고치지는 않고 어긋났다는 사실만 시끄럽게 알린다.
        """
        try:
            import yaml
            from ament_index_python.packages import get_package_share_directory
            f = (Path(get_package_share_directory("drobot_hybrid_planner"))
                 / "config" / "energy_params.yaml")
            ep = yaml.safe_load(open(f))["energy_model"]["ros__parameters"]
        except Exception as e:
            self.get_logger().info(f"energy_params 대조 건너뜀 ({type(e).__name__})")
            return

        for label, mine, theirs in (
                ("takeoff_time", self.takeoff_time, ep["mode_switch"]["takeoff_time"]),
                ("landing_time", self.landing_time, ep["mode_switch"]["landing_time"]),
                ("flight_speed", self.flight_speed, ep["air_mode"]["speed"])):
            if abs(float(mine) - float(theirs)) > 1e-6:
                self.get_logger().warn(
                    f"SSOT 불일치 {label}: 여기 {mine} vs energy_params {theirs} "
                    f"— 계획과 실행이 다른 값을 쓴다")

    # ---------- 백엔드 ----------
    def _setup_backend(self):
        """자세 전송 경로를 고르고 백엔드를 만든다."""
        if self.backend not in ("sim", "gazebo"):
            self.be = make_backend(self.backend)
            self.get_logger().info(f"백엔드 {self.be.name} — 로봇을 움직이지 않는다")
            return

        srv = f"/world/{self.world}/set_pose"
        self.pose_cli = None
        if SetEntityPose is not None:
            self.pose_cli = self.create_client(SetEntityPose, srv)
            if not self.pose_cli.wait_for_service(timeout_sec=3.0):
                self.pose_cli = None

        if self.pose_cli is not None:
            send, style = self._send_via_service, self.style
            self.get_logger().info(f"자세 전송: ROS 서비스 {srv}")
        else:
            # CLI 는 호출마다 프로세스를 띄운다. animate(20Hz)는 무리라
            # hop 으로 낮춘다 — 느린데 그대로 두면 비행이 늘어져 깨진다.
            send, style = cli_sender(self.world, self.model), "hop"
            self.get_logger().warn(
                f"ROS 서비스 {srv} 없음 — gz CLI 로 떨어진다. "
                f"style 을 {self.style} -> hop 으로 낮춘다. "
                f"(서비스를 쓰려면 ros_gz_bridge 에 set_pose 를 등록해야 하는데 "
                f"그 설정은 bringup 소유다)")

        self.be = make_backend(self.backend, send, style, self.dt)
        self.get_logger().info(
            f"백엔드 {self.be.name}:{style} — 월드 {self.world} · 모델 {self.model}")

    def _send_via_service(self, p: Pose) -> bool:
        req = SetEntityPose.Request()
        req.entity.name = self.model
        req.entity.type = 2                      # ros_gz_interfaces: MODEL
        req.pose.position.x, req.pose.position.y, req.pose.position.z = p.x, p.y, p.z
        qz, qw = p.quat_zw
        req.pose.orientation.z, req.pose.orientation.w = qz, qw
        fut = self.pose_cli.call_async(req)
        # 비행 스레드에서 부르므로 여기서 spin 하면 안 된다.
        # 메인 스레드의 executor 가 돌려주길 기다린다.
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 1.0:
            time.sleep(0.002)
        if not fut.done():
            self.be.last_error = "서비스 응답 시간 초과"
            return False
        return bool(fut.result() and fut.result().success)

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

        # 자세를 쓸 때 방향을 보존하려고 현재 yaw 를 들고 있는다.
        # 안 그러면 비행 뒤 로봇이 엉뚱한 쪽을 보고 착륙해 Nav2 가 헤맨다.
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y ** 2 + q.z ** 2))

        # 비행 중에는 전환 판정을 멈춘다. 백엔드가 자세를 쓰고 있어서
        # 위치가 계속 바뀌고, 그걸 '도달' 로 잘못 읽으면 안 된다.
        if self.flight_seg is not None:
            return

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

        if ev.point.is_takeoff:
            self._start_flight(ev.point)
        else:
            # 착륙점 도달은 비행이 이미 끝난 뒤에만 일어난다
            # (비행 구간 전체를 백엔드가 한 번에 수행하기 때문).
            after = self.tracker.complete(t)
            self.get_logger().info(f"  → {after}")

    def _limit_ground_speed(self, pct: float | None):
        """지상 컨트롤러 속도 상한을 건다. None 이면 해제.

        왜 목표 취소가 아닌가
            비행 중에도 controller_server 는 /cmd_vel 을 계속 낸다. 가만히
            두면 DiffDrive 가 로봇을 이륙점에서 밀어낸다. 그렇다고 목표를
            취소하면 목표를 보낸 쪽(record_run)이 canceled 를 받고 실험이
            거기서 끝나버린다. 속도 상한은 목표를 살려둔 채 로봇만 묶는다.

        왜 0 이 아니라 1% 인가
            Nav2 에서 speed_limit 0.0 은 NO_SPEED_LIMIT — '제한 없음' 이다
            (nav2_costmap_2d::NO_SPEED_LIMIT). 0 을 보내면 묶이는 게 아니라
            풀린다. 그래서 0 에 가까운 양수를 쓴다.
        """
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = True
        msg.speed_limit = 0.0 if pct is None else float(pct)
        self.speed_pub.publish(msg)
        self.get_logger().info(
            "  지상 속도 제한 해제" if pct is None
            else f"  지상 속도 {pct:.1f}% 로 제한 (비행 중)")

    # ---------- 비행 실행 ----------
    def _start_flight(self, up: SwitchPoint):
        """이륙점에서 짝 착륙점까지를 한 번에 수행한다.

        왜 한 번에 하나: 이륙과 착륙을 따로 돌리면 그 사이에 로봇이
        '공중에 있는데 Nav2 가 지상 주행을 명령하는' 상태가 생긴다.
        비행 구간 전체를 백엔드가 쥐고 있다가 지면에 내려놓는 편이 안전하다.
        """
        down = next((q for q in self.tracker.points
                     if q.pair_id == up.pair_id and not q.is_takeoff), None)
        if down is None:
            self.get_logger().error(
                f"  쌍 {up.pair_id} 의 착륙점이 없다 — 비행을 건너뛴다")
            self.tracker.complete(self._now())
            return

        seg = FlightSegment(
            x0=up.x, y0=up.y, x1=down.x, y1=down.y,
            altitude=up.flight_altitude, yaw=self.yaw,
            takeoff_time=self.takeoff_time, landing_time=self.landing_time,
            speed=self.flight_speed)
        self.flight_seg = seg
        self.flight_result = None
        self._limit_ground_speed(self.flight_speed_pct)

        # 백엔드는 구간 길이만큼(여기선 약 15초) 잠든다.
        # 콜백에서 그냥 부르면 노드 전체가 멈추므로 스레드로 뺀다.
        threading.Thread(target=self._run_flight, args=(seg,), daemon=True).start()

    def _run_flight(self, seg: FlightSegment):
        try:
            ok = self.be.execute(seg, self.get_logger(), time.sleep)
        except Exception as e:                      # 스레드에서 죽으면 조용히 멈춘다
            self.get_logger().error(f"  비행 중 예외 — {type(e).__name__}: {e}")
            ok = False
        self.flight_result = ok

    def _finish_flight(self):
        """비행 스레드 결과를 메인 스레드에서 회수한다.

        tracker 를 스레드 두 곳에서 건드리지 않으려고 이렇게 나눴다.
        """
        if self.flight_seg is None or self.flight_result is None:
            return
        ok, seg = self.flight_result, self.flight_seg
        self.flight_seg = None
        self.flight_result = None

        # 성공이든 실패든 제한은 반드시 푼다. 안 풀면 로봇이 영영 1% 속도로
        # 기어다니고, 원인을 찾기 어려운 상태가 된다.
        self._limit_ground_speed(None)

        if not ok:
            # 실패를 성공으로 넘기지 않는다. 전환점을 그대로 두면
            # 로봇이 다시 그 자리에 올 때 재시도한다.
            self.get_logger().error("  비행 실패 — 상태를 GROUND 로 되돌린다")
            self.tracker.state = GROUND
            return

        # TAKING_OFF -> FLYING 까지만 넘긴다.
        # 착륙은 다음 /odom 에서 '착륙점 도달' 로 정상 판정된다 — 백엔드가
        # 로봇을 이미 그 자리에 놓았기 때문이다. 여기서 억지로 GROUND 까지
        # 밀면 도달 판정을 건너뛰어, 실제 위치와 상태가 어긋나도 모르게 된다.
        after = self.tracker.complete(self._now())
        self.get_logger().info(
            f"  비행 완료 → {after}  "
            f"({seg.x1:.2f}, {seg.y1:.2f}) · {seg.total_time:.1f} s "
            f"— 착륙 판정은 다음 odom 에서")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9


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
