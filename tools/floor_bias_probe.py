#!/usr/bin/env python3
"""바닥이 왜 4~8 cm 떠 보이는지 — 깊이 점군에서 바닥 점의 높이 오차를 잰다.

왜 필요한가
    카메라가 잰 바닥 높이가 0 이 아니라 0.04~0.08 m 로 깔린다 (2026-10-05 기록).
    시뮬 깊이 카메라에는 노이즈 설정이 없으므로 '노이즈의 최댓값' 은 원인이 아니다.
    URDF 로 계산하면 왼쪽 바퀴 바닥이 -2.0 mm, 오른쪽이 +5.1 mm 라 평지에서 몸체가
    약 1.36° 옆으로 기운다(오른쪽이 내려앉음). EKF 는 two_d_mode 라 TF 는 수평으로
    본다. 그러면 로봇 옆쪽 바닥 점이 '옆 거리 × sin(roll)' 만큼 틀어진다 — 오른쪽은
    위로, 왼쪽은 아래로. ElevationLayer 는 칸의 최댓값을 쓰므로 위로 틀어진 쪽만 남는다.
    이 스크립트는 그 가설을 숫자로 확인한다.

무엇을 하나
    /camera/depth/points 를 몇 프레임 받아 ElevationLayer 와 같은 방식(메시지 시각의
    TF)으로 map 좌표로 옮긴다. prior map 에서 장애물과 0.3 m 이상 떨어진 바닥 칸에
    떨어진 점만 남긴다 (그 점의 참 높이는 0). 로봇 기준 앞(fwd)·옆(lat, 왼쪽 +) 좌표로
    바꿔 높이 오차를 표로 내고, z = a·fwd + b·lat + c 를 맞춘다.
        pitch ≈ atan(a)        (앞이 숙여지면 + — 앞쪽 바닥이 높게 보인다)
        roll  ≈ -asin(b)       (오른쪽이 내려앉으면 +.  URDF 예측 +1.36°)
        c                      (높이 자체의 어긋남 — base_footprint 높이 오차)
    roll 이 예측과 맞고 c 가 작으면 기울기가 원인이다.

사용 (시뮬이 떠 있고 로봇이 평지에 서 있을 때)
    python3 tools/floor_bias_probe.py --world base_map_h0.5_nogap --frames 20
    python3 tools/floor_bias_probe.py --selftest      # 시뮬 없이 계산만 확인
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
MAP_DIR = REPO / "src/drobot_description/maps"


# ---------------------------------------------------------------------------
# 계산 (ROS 를 모른다 — --selftest 로 확인)
# ---------------------------------------------------------------------------
def load_floor_mask(path: Path, margin_m: float = 0.3):
    """prior map 에서 '장애물과 margin 이상 떨어진 바닥' 칸 마스크."""
    rows = [ln for ln in path.read_text().splitlines() if ln.strip() and not ln.startswith("#")]
    w, h, res, ox, oy = rows[0].split()
    w, h, res, ox, oy = int(w), int(h), float(res), float(ox), float(oy)
    grid = np.array([[float(v) for v in ln.split()] for ln in rows[1:1 + h]])
    obstacle = grid > 1e-3
    k = int(math.ceil(margin_m / res))
    grown = obstacle.copy()
    for dy in range(-k, k + 1):           # 정사각형으로 키운다 (scipy 없이)
        for dx in range(-k, k + 1):
            grown |= np.roll(np.roll(obstacle, dy, axis=0), dx, axis=1)
    return ~grown, res, ox, oy


def floor_points(pts_map: np.ndarray, mask, res, ox, oy) -> np.ndarray:
    ix = np.floor((pts_map[:, 0] - ox) / res).astype(int)
    iy = np.floor((pts_map[:, 1] - oy) / res).astype(int)
    ok = (ix >= 0) & (iy >= 0) & (ix < mask.shape[1]) & (iy < mask.shape[0])
    sel = np.zeros(len(pts_map), bool)
    sel[ok] = mask[iy[ok], ix[ok]]
    return pts_map[sel]


def to_robot(pts_map: np.ndarray, rx: float, ry: float, ryaw: float) -> np.ndarray:
    """map → 로봇 기준 (fwd, lat, z). lat 은 왼쪽이 +."""
    dx, dy = pts_map[:, 0] - rx, pts_map[:, 1] - ry
    c, s = math.cos(ryaw), math.sin(ryaw)
    return np.c_[c * dx + s * dy, -s * dx + c * dy, pts_map[:, 2]]


def fit(robot_pts: np.ndarray):
    A = np.c_[robot_pts[:, 0], robot_pts[:, 1], np.ones(len(robot_pts))]
    (a, b, c), *_ = np.linalg.lstsq(A, robot_pts[:, 2], rcond=None)
    return {"pitch_deg": math.degrees(math.atan(a)),
            "roll_deg": math.degrees(-math.asin(max(-1.0, min(1.0, b)))),
            "offset_m": c}


def report(robot_pts: np.ndarray) -> dict:
    z = robot_pts[:, 2]
    print(f"\n바닥 점 {len(z):,} 개 — 참 높이는 0")
    print(f"  평균 {100*z.mean():+.1f} cm · 상위 95 % {100*np.percentile(z, 95):+.1f} cm · "
          f"최댓값 {100*z.max():+.1f} cm")
    print("\n  옆 거리(왼쪽 +)별 평균 높이 오차")
    for lo in np.arange(-3.0, 3.0, 0.5):
        m = (robot_pts[:, 1] >= lo) & (robot_pts[:, 1] < lo + 0.5)
        if m.sum() > 50:
            print(f"    {lo:+.1f} ~ {lo + 0.5:+.1f} m : {100*z[m].mean():+5.1f} cm  ({m.sum():,} 점)")
    print("\n  앞 거리별 평균 높이 오차")
    for lo in np.arange(0.0, 6.0, 1.0):
        m = (robot_pts[:, 0] >= lo) & (robot_pts[:, 0] < lo + 1.0)
        if m.sum() > 50:
            print(f"    {lo:.0f} ~ {lo + 1:.0f} m : {100*z[m].mean():+5.1f} cm  ({m.sum():,} 점)")
    f = fit(robot_pts)
    print(f"\n  평면 맞춤: roll {f['roll_deg']:+.2f}° (오른쪽이 내려앉으면 +, URDF 예측 +1.36°) · "
          f"pitch {f['pitch_deg']:+.2f}° (앞이 숙여지면 +) · 높이 어긋남 {100*f['offset_m']:+.1f} cm")
    if abs(f["roll_deg"] - 1.36) < 0.4 and abs(f["offset_m"]) < 0.01:
        print("  → 몸체 기울기(좌우 바퀴 높이 차) 가설과 맞는다")
    else:
        print("  → 기울기만으로는 설명되지 않는다 — 표를 보고 다른 원인을 찾을 것")
    return f


def selftest() -> int:
    """알려진 roll·pitch 로 만든 가짜 점군에서 그 값을 되찾는지."""
    rng = np.random.default_rng(0)
    roll, pitch = math.radians(1.36), math.radians(0.2)
    fwd = rng.uniform(0.9, 4.0, 20000)
    lat = rng.uniform(-1.5, 1.5, 20000) * fwd / 4
    # 실제 바닥(z=0) 점이 기울어진 몸체에서는 어떻게 보이나 — TF 는 수평이라고 본다
    z = -lat * math.sin(roll) + fwd * math.tan(pitch)
    f = fit(np.c_[fwd, lat, z])
    ok = abs(f["roll_deg"] - 1.36) < 0.02 and abs(f["pitch_deg"] - 0.2) < 0.02
    print(f"selftest: roll {f['roll_deg']:.3f}° (1.36) · pitch {f['pitch_deg']:.3f}° (0.20) → "
          f"{'통과' if ok else '실패'}")
    mask = np.ones((10, 10), bool)
    pts = np.array([[0.12, 0.12, 0.0], [-1.0, 0.0, 0.0], [0.6, 0.6, 0.0]])
    kept = floor_points(pts, mask, 0.1, 0.0, 0.0)
    ok &= len(kept) == 2
    r = to_robot(np.array([[2.0, 2.0, 0.0]]), 2.0, 1.0, math.pi / 2)   # 로봇이 +y 를 보면 앞 1 m
    ok &= abs(r[0, 0] - 1.0) < 1e-9 and abs(r[0, 1]) < 1e-9
    print("selftest:", "전부 통과" if ok else "실패")
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# ROS
# ---------------------------------------------------------------------------
def quat_to_mat(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def compose(a, b):
    """a(부모→중간) ∘ b(중간→자식) 을 TransformStamped 하나로."""
    from geometry_msgs.msg import TransformStamped
    Ra, Rb = quat_to_mat(a.transform.rotation), quat_to_mat(b.transform.rotation)
    ta = np.array([a.transform.translation.x, a.transform.translation.y, a.transform.translation.z])
    tb = np.array([b.transform.translation.x, b.transform.translation.y, b.transform.translation.z])
    R, t = Ra @ Rb, Ra @ tb + ta
    out = TransformStamped()
    out.header = b.header
    out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = t
    # 회전행렬 -> 쿼터니언
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = out.transform.rotation
    q.x, q.y, q.z, q.w = x, y, z, w
    return out


def run_ros(args) -> int:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    import tf2_ros

    mask, res, ox, oy = load_floor_mask(MAP_DIR / f"{args.world}.heightmap", args.margin)
    rclpy.init()
    node = Node("floor_bias_probe")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    buf = tf2_ros.Buffer()
    # TF 를 자기 스레드에서 받는다. 아니면 이 노드가 점군을 처리하는 동안 TF 메시지가
    # 밀려 버퍼가 늦어지고, 점군 시각의 TF 를 못 찾는다 (처음 판에서 0.6 s 늦게 보였다).
    tf_listener = tf2_ros.TransformListener(buf, node, spin_thread=True)  # noqa: F841
    got = []
    lags = []   # 점군 시각 - 그때 있던 가장 최근 odom->base_footprint 시각 (s)

    def on_cloud(msg):
        if len(got) >= args.frames:
            return
        stamp = Time.from_msg(msg.header.stamp)
        try:
            t = buf.lookup_transform("map", msg.header.frame_id, stamp,
                                     timeout=Duration(seconds=1.0))
            rb = buf.lookup_transform("map", "base_footprint", stamp,
                                      timeout=Duration(seconds=1.0))
        except Exception:
            # 점군 시각의 TF 가 아직 없다 (TF 가 카메라보다 늦다). 로봇이 서 있는 동안에는
            # 가장 최근 TF 를 써도 같은 자리라 결과가 같다. 얼마나 늦은지 기록한다.
            # ElevationLayer 는 0.1 s 만 기다리고 그 프레임을 버린다 (lookupToGlobal).
            # 체인 전체의 '가장 최근 공통 시각' 도 없을 수 있다 — slam 의 map->odom 은 스캔마다
            # 한 번, 미래 시각(+transform_timeout)으로 찍힌다. 링크마다 따로 최근 값을 받아
            # 이어 붙인다 (로봇이 서 있으니 같은 자세다).
            try:
                m2o = buf.lookup_transform("map", "odom", Time())
                o2c = buf.lookup_transform("odom", msg.header.frame_id, Time())
                o2b = buf.lookup_transform("odom", "base_footprint", Time())
            except Exception as e:
                node.get_logger().warn(f"TF 없음 — {e}")
                return
            t, rb = compose(m2o, o2c), compose(m2o, o2b)
            lag = (stamp - Time.from_msg(o2b.header.stamp)).nanoseconds * 1e-9
            lags.append(lag)
        p = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        p = p[np.isfinite(p).all(axis=1)]
        R = quat_to_mat(t.transform.rotation)
        tr = t.transform.translation
        pm = p @ R.T + np.array([tr.x, tr.y, tr.z])
        q = rb.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        fl = floor_points(pm, mask, res, ox, oy)
        got.append(to_robot(fl, rb.transform.translation.x, rb.transform.translation.y, yaw))
        node.get_logger().info(f"프레임 {len(got)}/{args.frames} — 바닥 점 {len(fl):,}")

    node.create_subscription(PointCloud2, "/camera/depth/points", on_cloud, qos_profile_sensor_data)
    while rclpy.ok() and len(got) < args.frames:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        print("받은 프레임이 없다")
        return 1
    if lags:
        print(f"\nTF 지연: {len(lags)}/{len(got)} 프레임에서 점군 시각의 TF 가 1 s 안에 오지 않아 "
              f"가장 최근 TF 로 대신했다 — TF 가 평균 {np.mean(lags):.2f} s, 최대 {np.max(lags):.2f} s 늦었다.")
        print("  ElevationLayer 는 0.1 s 만 기다리고 그 프레임을 버린다 — 카메라 높이가 지도에 안 들어간다.")
    report(np.vstack(got))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--world", default="base_map_h0.5_nogap")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--margin", type=float, default=0.3, help="장애물에서 이만큼 떨어진 바닥만 (m)")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    return selftest() if args.selftest else run_ros(args)


if __name__ == "__main__":
    sys.exit(main())
