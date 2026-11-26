#!/usr/bin/env python3
"""센서 메시지의 시각이 ROS 시계와 얼마나 어긋나는지 잰다.

왜 필요한가
    ElevationLayer 는 점군 시각의 TF 를 0.1 s 만 기다리고, 없으면 그 프레임을 버린다
    (lookupToGlobal). 2026-11-20 확인에서 점군 시각이 그때 있던 가장 최근 TF 보다 평균
    0.74 s(최대 1.65 s) 앞서 있었다. 그런데 tf2_monitor(use_sim_time) 로는 TF 자체가
    ROS 시계보다 5 ms 밖에 늦지 않았다. 그러면 남는 설명은 '점군이 ROS 시계보다 미래
    시각으로 찍혀 온다' 이다. 센서마다 (메시지 시각 - 지금 ROS 시각) 을 재서 가른다.

읽는 법
    양수   메시지가 미래 시각으로 찍혀 있다 → 그만큼 TF 를 기다려야 한다
    음수   메시지가 늦게 도착한다 (보통 수십 ms)
    /clock 간격이 크면 ROS 시계가 띄엄띄엄 간다 → 시계를 쓰는 노드(EKF 등)가 밀린다

사용 (시뮬이 떠 있을 때, 8 초)
    python3 tools/stamp_offset_probe.py
"""
import statistics
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan, PointCloud2

TOPICS = [
    ("/camera/depth/points", PointCloud2, qos_profile_sensor_data),
    ("/scan", LaserScan, qos_profile_sensor_data),
    ("/imu", Imu, qos_profile_sensor_data),
    ("/odom", Odometry, 10),
    ("/odometry/filtered", Odometry, 10),
]


def main(seconds: float = 8.0) -> None:
    rclpy.init()
    node = Node("stamp_offset_probe", parameter_overrides=[Parameter("use_sim_time", value=True)])
    data = {t: [] for t, _, _ in TOPICS}

    def make_cb(topic):
        def cb(msg):
            now = node.get_clock().now().nanoseconds * 1e-9
            if now <= 0:
                return
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            data[topic].append(st - now)
        return cb

    for topic, typ, qos in TOPICS:
        node.create_subscription(typ, topic, make_cb(topic), qos)
    clock_wall = []
    node.create_subscription(Clock, "/clock", lambda m: clock_wall.append(time.monotonic()), 10)

    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)

    print("\n메시지 시각 - 지금 ROS 시각  (양수 = 미래 시각으로 찍혀 옴)")
    for topic, _, _ in TOPICS:
        v = data[topic]
        if v:
            print(f"  {topic:24s} {len(v):5d} 개  평균 {statistics.mean(v):+.3f} s  "
                  f"최소 {min(v):+.3f}  최대 {max(v):+.3f}")
        else:
            print(f"  {topic:24s} 받은 것 없음")
    if len(clock_wall) > 2:
        gaps = [b - a for a, b in zip(clock_wall, clock_wall[1:])]
        print(f"\n/clock {len(clock_wall)} 개 / {seconds:.0f} s(실제 시간) — 간격 평균 "
              f"{statistics.mean(gaps) * 1000:.0f} ms, 최대 {max(gaps) * 1000:.0f} ms")
    else:
        print("\n/clock 을 거의 못 받았다")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
