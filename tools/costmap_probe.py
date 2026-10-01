#!/usr/bin/env python3
"""costmap 의 실제 cost 값을 선분을 따라 찍는다.

왜 필요한가
    플래너가 왜 거기서 이륙하고 거기에 착륙하는지는 costmap 이 그 셀을
    뭐라고 보는지에 달려 있다. 추측 대신 값을 직접 읽는다.

    cost 등급 (인터페이스 규약 ③)
        0   free               평지
        100 rover_traversable  주행 가능(지형 거침)
        200 fly_over           비행 필요
        254 impassable         통과 불가 (LETHAL)
        255 NO_INFORMATION     미지
    그 사이 값은 inflation_layer 가 만든 것이다.

사용 (컨테이너 안)
    python3 /app/tools/costmap_probe.py --x0 2.12 --y0 1.5 --x1 2.12 --y1 5.5
    python3 /app/tools/costmap_probe.py --topic /local_costmap/costmap --x0 ...
"""
from __future__ import annotations

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid

LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def label(v: int) -> str:
    if v < 0:
        return "미지(-1)"
    if v == 0:
        return "free"
    if v <= 99:
        return f"inflation({v})"
    if v == 100:
        return "rover_traversable"
    return f"cost {v}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/global_costmap/costmap")
    ap.add_argument("--x0", type=float, required=True)
    ap.add_argument("--y0", type=float, required=True)
    ap.add_argument("--x1", type=float, required=True)
    ap.add_argument("--y1", type=float, required=True)
    ap.add_argument("--step", type=float, default=0.1)
    ap.add_argument("--timeout", type=float, default=10.0)
    a = ap.parse_args()

    rclpy.init()
    node = Node("costmap_probe")
    box = {}

    node.create_subscription(OccupancyGrid, a.topic,
                             lambda m: box.setdefault("g", m), LATCHED)
    import time
    t0 = time.time()
    while "g" not in box and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "g" not in box:
        sys.exit(f"{a.topic} 를 못 받았다")

    g = box["g"]
    res = g.info.resolution
    ox, oy = g.info.origin.position.x, g.info.origin.position.y
    W, H = g.info.width, g.info.height
    print(f"{a.topic}  {W}x{H} @ {res} m  원점 ({ox:.2f}, {oy:.2f})")
    print(f"{'거리':>6} {'x':>7} {'y':>7} {'cell':>12} {'값':>5}  등급")

    d = math.dist((a.x0, a.y0), (a.x1, a.y1))
    n = max(1, int(d / a.step))
    for i in range(n + 1):
        f = i / n
        x = a.x0 + (a.x1 - a.x0) * f
        y = a.y0 + (a.y1 - a.y0) * f
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        if not (0 <= mx < W and 0 <= my < H):
            print(f"{d*f:6.2f} {x:7.2f} {y:7.2f} {'맵 밖':>12}")
            continue
        v = g.data[my * W + mx]
        print(f"{d*f:6.2f} {x:7.2f} {y:7.2f} {f'({mx},{my})':>12} {v:5d}  {label(v)}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
