#!/usr/bin/env python3
# Copyright 2026 leo11dk
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""로버 ↔ 드론 변신(암 회전) 시퀀스.

암 4 개를 동시에 목표 각도로 보내고, 실제로 그 각도에 도달할 때까지 기다린다.

왜 노드가 필요한가
    암 조인트는 Gazebo 의 JointPositionController 플러그인이 직접 제어하고
    (drobot_description/urdf/gazebo.xacro), ros_gz_bridge 가 ROS 토픽으로
    열어 둔다. 즉 명령을 쏠 수단은 이미 다 있다 — 없는 것은 '쏘는 쪽' 뿐이다.

    쉘에서 ros2 topic pub 을 4 번 돌리면 매번 노드를 새로 띄우고 상대를
    발견하느라 1~2 초씩 걸려, 암이 2 초 간격으로 차례차례 올라간다
    (2026-10-05 실측). 퍼블리셔를 들고 있는 노드 하나가 한 번에 쏘면
    그 지연이 없다.

완료 판정
    /joint_states 의 실제 각도가 목표의 ±tolerance 안에 들어오면 완료로 본다.
    열린 루프로 '3 초 기다리면 되겠지' 하지 않는 이유는, 암이 장애물에 걸리거나
    토크가 모자라 목표에 못 가는 경우를 비행 시퀀스가 알아야 하기 때문이다.
    못 가면 timeout 으로 실패를 돌려준다 — 접히지 않은 채 이륙하면 안 된다.
"""

import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

ARM_JOINTS = ['lf_arm_rev', 'lr_arm_rev', 'rf_arm_rev', 'rr_arm_rev']


class TransformManager(Node):
    """암 4 개를 동시에 접고 펴는 노드."""

    def __init__(self):
        super().__init__('transform_manager')

        # URDF 의 limit 과 맞춘다: lower=0, upper=1.5708 (90도)
        self.declare_parameter('drone_angle', 1.5708)
        self.declare_parameter('rover_angle', 0.0)
        # 서보 스펙상 0 -> 90 도가 약 3.1 초 (velocity 0.5 rad/s). 여유를 둔다.
        self.declare_parameter('timeout', 8.0)
        self.declare_parameter('tolerance', 0.05)      # rad, 약 2.9 도
        self.declare_parameter('cmd_topic_prefix', '/arm/')
        self.declare_parameter('cmd_topic_suffix', '/cmd_pos')

        self._drone = self.get_parameter('drone_angle').value
        self._rover = self.get_parameter('rover_angle').value
        self._timeout = self.get_parameter('timeout').value
        self._tol = self.get_parameter('tolerance').value
        prefix = self.get_parameter('cmd_topic_prefix').value
        suffix = self.get_parameter('cmd_topic_suffix').value

        # 서비스 콜백이 완료를 기다리는 동안 /joint_states 콜백이 계속 돌아야
        # 한다. 같은 Reentrant 그룹 + MultiThreadedExecutor 조합이 필요하다.
        group = ReentrantCallbackGroup()

        self._pubs = {
            j: self.create_publisher(Float64, f'{prefix}{j}{suffix}', 10)
            for j in ARM_JOINTS
        }
        self._positions = {}
        self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10,
            callback_group=group)

        self.create_service(
            Trigger, 'transform_to_drone',
            lambda req, res: self._handle(res, self._drone, '드론'),
            callback_group=group)
        self.create_service(
            Trigger, 'transform_to_rover',
            lambda req, res: self._handle(res, self._rover, '로버'),
            callback_group=group)

        self.get_logger().info(
            f'변신 노드 시작: 드론={self._drone:.4f} rad, 로버={self._rover:.4f} rad, '
            f'허용오차={self._tol:.3f} rad, timeout={self._timeout:.1f} s')

    # ---- 콜백 ---------------------------------------------------------
    def _on_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self._pubs:
                self._positions[name] = pos

    # ---- 동작 ---------------------------------------------------------
    def _publish_all(self, angle: float):
        """암 4 개에 같은 목표를 한 번에 쏜다 (순차 지연 없음)."""
        msg = Float64()
        msg.data = float(angle)
        for pub in self._pubs.values():
            pub.publish(msg)

    def _remaining(self, target: float):
        """아직 목표에 도달하지 못한 조인트와 그 오차."""
        out = {}
        for j in ARM_JOINTS:
            if j not in self._positions:
                out[j] = None              # 아직 상태를 못 받음
            else:
                err = abs(self._positions[j] - target)
                if err > self._tol:
                    out[j] = err
        return out

    def _handle(self, res, target: float, label: str):
        self.get_logger().info(f'{label} 모드로 변신 시작 (목표 {target:.4f} rad)')
        self._publish_all(target)

        deadline = time.monotonic() + self._timeout
        while time.monotonic() < deadline:
            if not rclpy.ok():
                res.success = False
                res.message = '종료 중'
                return res
            left = self._remaining(target)
            if not left:
                elapsed = self._timeout - (deadline - time.monotonic())
                res.success = True
                res.message = f'{label} 변신 완료 ({elapsed:.1f} s)'
                self.get_logger().info(res.message)
                return res
            time.sleep(0.05)

        left = self._remaining(target)
        detail = ', '.join(
            f'{j}=상태없음' if e is None else f'{j} 오차 {e:.3f} rad'
            for j, e in left.items())
        res.success = False
        res.message = f'{label} 변신 timeout ({self._timeout:.1f} s) — {detail}'
        self.get_logger().warn(res.message)
        return res


def main(args=None):
    rclpy.init(args=args)
    node = TransformManager()
    # 서비스가 완료를 기다리는 동안 joint_states 를 계속 받아야 하므로
    # 단일 스레드 실행기를 쓰면 영원히 timeout 난다.
    executor = rclpy.executors.MultiThreadedExecutor()
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
