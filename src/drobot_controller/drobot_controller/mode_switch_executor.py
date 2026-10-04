#!/usr/bin/env python3
# Copyright 2026 leo11dk
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""경로의 모드 전환점에 도달하면 변신 모션을 실행한다.

무엇을 잇는가
    플래너(HybridAStarPlanner)는 경로와 함께 /mode_switch_points 로
    ModeSwitchPlan 을 퍼블리시한다. 거기에는 이륙(GROUND_TO_AIR)과
    착륙(AIR_TO_GROUND) 지점이 맵 좌표로 들어 있다.

    transform_manager 는 /transform_to_drone, /transform_to_rover 서비스로
    암 4 개를 접고 편다.

    둘 사이가 비어 있었다. 이 노드가 그 사이를 잇는다 — 로봇이 전환점
    반경 안에 들어오면 해당 서비스를 부른다.

무엇을 하지 않는가
    실제로 띄우지 않는다. 추력 물리가 아직 없어서(프로펠러 링크 자체가
    URDF 에 없다) 변신만 하고 비행은 하지 못한다. 이 노드의 범위는
    "전환 명령이 내려오면 전환 모션까지" 다. 비행 제어가 생기면 여기에
    이륙/착륙 호출을 덧붙이면 된다.

    또 로봇을 멈추지 않는다. 멈춤은 BT 나 컨트롤러의 일이고, 이 노드가
    cmd_vel 을 건드리면 Nav2 와 싸운다. 2026-10-05 현재는 로컬 코스트맵이
    flyover 를 막아 로봇이 전환점 근처에서 어차피 멈춘다.

왜 TF 를 쓰는가
    전환점은 map 프레임이고 /odom 은 odom 프레임이다. 두 프레임은 EKF 가
    보정하는 만큼 어긋나므로, 거리 비교는 반드시 같은 프레임에서 해야 한다.
    map -> base_footprint 변환을 받아 로봇 위치를 map 으로 가져온다.

중복 실행 방지
    플래너는 재계획할 때마다 같은 전환점을 다시 퍼블리시한다. 한 번 실행한
    전환은 (pair_id, switch_type) 으로 기억해 두고 다시 부르지 않는다.
    새 경로가 와서 전환점 목록이 바뀌면 기억을 비운다 — 다른 경로의
    pair_id 0 은 이전 경로의 pair_id 0 과 다른 지점이기 때문이다.
"""

import math

import rclpy
from drobot_msgs.msg import ModeSwitchPlan, ModeSwitchPoint
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener


class ModeSwitchExecutor(Node):
    """전환점 도달 -> 변신 서비스 호출."""

    def __init__(self):
        super().__init__('mode_switch_executor')

        # 전환점에 이만큼 가까워지면 실행한다.
        # 너무 작으면 컨트롤러가 그 점을 정확히 밟지 않아 영영 안 걸리고,
        # 너무 크면 엉뚱한 자리에서 접힌다. footprint 길이(0.457 m)의
        # 두 배 정도로 잡았다.
        self.declare_parameter('trigger_radius', 0.9)
        # 변신은 수 초 걸린다. 그 사이 콜백이 겹치지 않도록 잠근다.
        self.declare_parameter('service_timeout', 15.0)
        self.declare_parameter('plan_topic', '/mode_switch_points')
        self.declare_parameter('robot_frame', 'base_footprint')

        self._radius = self.get_parameter('trigger_radius').value
        self._svc_timeout = self.get_parameter('service_timeout').value
        self._robot_frame = self.get_parameter('robot_frame').value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._plan = None
        self._plan_key = None      # 현재 경로의 전환점 목록 지문
        self._done = set()         # 실행 완료한 (pair_id, switch_type)
        self._busy = False

        group = ReentrantCallbackGroup()

        # 플래너는 transient_local 로 퍼블리시한다 (hybrid_astar_planner.cpp).
        # 구독도 맞춰야 늦게 떠도 마지막 계획을 받는다.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ModeSwitchPlan,
            self.get_parameter('plan_topic').value,
            self._on_plan, qos, callback_group=group)

        self._to_drone = self.create_client(
            Trigger, '/transform_to_drone', callback_group=group)
        self._to_rover = self.create_client(
            Trigger, '/transform_to_rover', callback_group=group)

        self.create_timer(0.2, self._tick, callback_group=group)

        self.get_logger().info(
            f'mode_switch_executor 시작 — 반경 {self._radius:.2f} m, '
            f'로봇 프레임 {self._robot_frame}')

    # ------------------------------------------------------------------
    def _on_plan(self, msg: ModeSwitchPlan):
        key = tuple(
            (p.pair_id, p.switch_type, round(p.position.x, 3), round(p.position.y, 3))
            for p in msg.switch_points)
        if key != self._plan_key:
            # 경로가 바뀌었다 — 이전 경로의 실행 기록은 의미가 없다.
            self._plan_key = key
            self._done.clear()
            n = len(msg.switch_points)
            if n:
                self.get_logger().info(
                    f'새 전환 계획: 전환점 {n} 개, '
                    f'예상 비행 에너지 {msg.total_flight_energy:.3f} Wh')
                for p in msg.switch_points:
                    kind = ('이륙' if p.switch_type == ModeSwitchPoint.GROUND_TO_AIR
                            else '착륙')
                    self.get_logger().info(
                        f'  [{p.pair_id}] {kind} ({p.position.x:.2f}, '
                        f'{p.position.y:.2f}) 고도 {p.flight_altitude:.2f} m')
            else:
                self.get_logger().info('새 전환 계획: 전환점 없음 (순수 지상 경로)')
        self._plan = msg

    # ------------------------------------------------------------------
    def _robot_xy(self, frame_id: str):
        """전환점과 같은 프레임에서 로봇 위치를 얻는다. 실패하면 None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                frame_id or 'map', self._robot_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.2))
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().warn(f'TF 실패: {exc}', throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        return t.x, t.y

    def _tick(self):
        if self._busy or self._plan is None or not self._plan.switch_points:
            return

        here = self._robot_xy(self._plan.header.frame_id)
        if here is None:
            return

        for p in self._plan.switch_points:
            tag = (p.pair_id, p.switch_type)
            if tag in self._done:
                continue
            d = math.hypot(p.position.x - here[0], p.position.y - here[1])
            if d <= self._radius:
                self._execute(p, d)
                return

    # ------------------------------------------------------------------
    def _execute(self, p: ModeSwitchPoint, dist: float):
        takeoff = p.switch_type == ModeSwitchPoint.GROUND_TO_AIR
        kind = '이륙' if takeoff else '착륙'
        client = self._to_drone if takeoff else self._to_rover
        name = '/transform_to_drone' if takeoff else '/transform_to_rover'

        if not client.service_is_ready():
            self.get_logger().warn(
                f'{name} 아직 없음 — transform_manager 가 떠 있는지 확인하라',
                throttle_duration_sec=5.0)
            return

        self._busy = True
        self.get_logger().info(
            f'[{p.pair_id}] {kind} 지점 도달 (거리 {dist:.2f} m) — {name} 호출')

        future = client.call_async(Trigger.Request())

        def _done(fut):
            self._busy = False
            try:
                res = fut.result()
            except Exception as exc:                   # noqa: BLE001
                self.get_logger().error(f'[{p.pair_id}] {kind} 호출 실패: {exc}')
                return
            if res.success:
                self.get_logger().info(f'[{p.pair_id}] {kind} 변신 완료')
            else:
                # 실패해도 기록은 남긴다 — 같은 지점에서 무한 재시도하면
                # 로그만 쌓이고 상태는 안 바뀐다. 암이 목표에 못 가는 것은
                # 마찰 문제(drobot.urdf.xacro)로 따로 추적한다.
                self.get_logger().warn(
                    f'[{p.pair_id}] {kind} 변신 실패: {res.message}')
            self._done.add((p.pair_id, p.switch_type))

        future.add_done_callback(_done)


def main():
    rclpy.init()
    node = ModeSwitchExecutor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
