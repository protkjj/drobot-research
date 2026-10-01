#!/usr/bin/env python3
"""포인트클라우드가 전역 좌표 어디에 떨어지는지 직접 잰다.

왜 필요한가
    ElevationLayer 가 점을 하나도 안 쌓는데, 구독·주기·TF 변환은 모두
    정상이었다. 그렇다면 점이 콜백 안에서 버려지는 것이다. 레이어와
    똑같은 계산(TF 변환 -> 높이 필터 -> 격자 매핑)을 밖에서 재현해
    어느 단계에서 몇 개가 떨어지는지 센다.

사용 (컨테이너 안)
    python3 /app/tools/cloud_probe.py
    python3 /app/tools/cloud_probe.py --frame map --min-z -0.5 --max-z 3.0
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import tf2_ros
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/camera/depth/points")
    ap.add_argument("--frame", default="map", help="전역 프레임 (레이어의 global_frame)")
    ap.add_argument("--min-z", type=float, default=-0.5)
    ap.add_argument("--max-z", type=float, default=3.0)
    ap.add_argument("--timeout", type=float, default=15.0)
    a = ap.parse_args()

    rclpy.init()
    node = Node("cloud_probe")
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])

    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)

    box = {}
    node.create_subscription(PointCloud2, a.topic,
                             lambda m: box.update(msg=m), qos_profile_sensor_data)

    t0 = time.time()
    while "msg" not in box and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "msg" not in box:
        sys.exit(f"{a.topic} 를 못 받았다")

    msg = box["msg"]
    print(f"{a.topic}  {msg.width}x{msg.height}  frame '{msg.header.frame_id}'  "
          f"stamp {msg.header.stamp.sec}.{msg.header.stamp.nanosec // 1000000:03d}")

    # TF — 레이어와 같은 방식 (센서 타임스탬프 기준)
    try:
        tf = buf.lookup_transform(a.frame, msg.header.frame_id,
                                  rclpy.time.Time.from_msg(msg.header.stamp),
                                  rclpy.duration.Duration(seconds=1.0))
        print(f"  TF {msg.header.frame_id} -> {a.frame}  OK")
    except Exception as e:
        sys.exit(f"  TF 실패: {e}")

    R = quat_to_mat(tf.transform.rotation)
    t = np.array([tf.transform.translation.x,
                  tf.transform.translation.y,
                  tf.transform.translation.z])
    print(f"  센서 위치 ({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})")

    pts = np.array(list(point_cloud2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=False)))
    if pts.dtype.names:                      # 구조화 배열이면 펼친다
        pts = np.stack([pts["x"], pts["y"], pts["z"]], axis=-1)
    pts = pts.reshape(-1, 3).astype(float)
    total = len(pts)

    finite = np.isfinite(pts).all(axis=1)
    p = pts[finite]
    print(f"\n  전체 {total}  ·  유한값 {len(p)}  ·  버려짐 {total - len(p)}")
    if len(p) == 0:
        sys.exit("  유한한 점이 하나도 없다")

    print(f"  센서 프레임 범위  x [{p[:,0].min():.2f}, {p[:,0].max():.2f}]  "
          f"y [{p[:,1].min():.2f}, {p[:,1].max():.2f}]  "
          f"z [{p[:,2].min():.2f}, {p[:,2].max():.2f}]")

    g = (R @ p.T).T + t
    print(f"  전역 프레임 범위  x [{g[:,0].min():.2f}, {g[:,0].max():.2f}]  "
          f"y [{g[:,1].min():.2f}, {g[:,1].max():.2f}]  "
          f"z [{g[:,2].min():.2f}, {g[:,2].max():.2f}]")

    keep = (g[:, 2] >= a.min_z) & (g[:, 2] <= a.max_z)
    print(f"\n  높이 필터 [{a.min_z}, {a.max_z}] 통과 {keep.sum()} / {len(g)}"
          f"  ({100.0 * keep.sum() / max(1, len(g)):.1f}%)")
    if keep.sum() == 0:
        print("  ← 여기서 전부 버려진다")
        return

    k = g[keep]
    print(f"\n  통과한 점의 전역 z 분포")
    for lo, hi in ((-10, -0.1), (-0.1, 0.1), (0.1, 0.3), (0.3, 0.55),
                   (0.55, 0.8), (0.8, 1.5), (1.5, 100)):
        n = ((k[:, 2] >= lo) & (k[:, 2] < hi)).sum()
        if n:
            print(f"    {lo:6.2f} ~ {hi:6.2f} m : {n:7d}  {'#' * min(40, n // 2000)}")

    print(f"\n  통과한 점의 xy 범위  x [{k[:,0].min():.2f}, {k[:,0].max():.2f}]  "
          f"y [{k[:,1].min():.2f}, {k[:,1].max():.2f}]")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
