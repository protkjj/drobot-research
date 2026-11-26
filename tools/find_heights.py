#!/usr/bin/env python3
"""지금 지도에서 어떤 높이대의 칸이 어디 있는지 — 덩어리로 묶어 prior map 과 비교한다.

왜 필요한가
    2026-11-20 4 회차 실행에서 첫 계획의 비행 고도 후보가 0.80·1.70 m 두 개였다.
    1.70 m 는 높이 0.81~0.90 m 인 칸이 있다는 뜻인데 nogap 월드에는 그런 물체가 없다
    (바닥 0, 상자 0.5, 벽 3.0). 그 칸이 어디서 왔는지 — 센서가 잘못 잰 것인지 — 찾는다.
    후보가 하나 늘면 계획이 1.2 s → 3.3 s 가 된다.

읽는 것
    /global_costmap/elevation_layer/elevation_grid (OccupancyGrid)
        값 = 높이 / fly_over_max(1.55 m) × 100,  -1 = 모름
    prior map (maps/<world>.heightmap) — 그 칸의 정답 높이

사용 (시뮬이 떠 있을 때)
    python3 tools/find_heights.py --world base_map_h0.5_nogap              # 0.75~1.00 m
    python3 tools/find_heights.py --world base_map_h0.5_nogap --lo 0.15 --hi 0.45
"""
import argparse
from collections import deque
from pathlib import Path

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

REPO = Path(__file__).resolve().parents[1]
MAP_DIR = REPO / "src/drobot_description/maps"


def load_prior(world: str):
    rows = [ln for ln in (MAP_DIR / f"{world}.heightmap").read_text().splitlines()
            if ln.strip() and not ln.startswith("#")]
    w, h, res, ox, oy = rows[0].split()
    w, h, res, ox, oy = int(w), int(h), float(res), float(ox), float(oy)
    grid = [[float(v) for v in ln.split()] for ln in rows[1:1 + h]]

    def at(x, y):
        i, j = int((x - ox) / res), int((y - oy) / res)
        return grid[j][i] if 0 <= i < w and 0 <= j < h else None
    return at


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--world", default="base_map_h0.5_nogap")
    ap.add_argument("--lo", type=float, default=0.75, help="찾을 높이 하한 (m)")
    ap.add_argument("--hi", type=float, default=1.00, help="찾을 높이 상한 (m)")
    ap.add_argument("--fly-over-max", type=float, default=1.55, help="격자 값의 스케일 (nav2_params)")
    ap.add_argument("--topic", default="/global_costmap/elevation_layer/elevation_grid")
    a = ap.parse_args()

    rclpy.init()
    node = Node("find_heights")
    got = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, a.topic, lambda m: got.append(m), qos)
    for _ in range(100):
        if got:
            break
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        print(f"{a.topic} 를 10 s 안에 못 받았다")
        return 1

    m = got[-1]
    W, H, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    unit = a.fly_over_max / 100.0
    hit = set()
    for k, v in enumerate(m.data):
        if v >= 0 and a.lo <= v * unit <= a.hi:
            hit.add((k % W, k // W))
    prior = load_prior(a.world)
    print(f"{a.topic} {W}x{H} @ {res:.3f} m — 높이 {a.lo:.2f}~{a.hi:.2f} m 인 칸 {len(hit)} 개")
    if not hit:
        return 0

    # 4 방향으로 이어진 칸을 덩어리로
    seen, clusters = set(), []
    for c in sorted(hit):
        if c in seen:
            continue
        q, comp = deque([c]), []
        seen.add(c)
        while q:
            i, j = q.popleft()
            comp.append((i, j))
            for n in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if n in hit and n not in seen:
                    seen.add(n)
                    q.append(n)
        clusters.append(comp)
    clusters.sort(key=len, reverse=True)

    print(f"\n덩어리 {len(clusters)} 개 (큰 것부터 10 개)")
    print("   칸 수   x 범위 (m)        y 범위 (m)        잰 높이 (m)    prior 높이 (m)")
    for comp in clusters[:10]:
        xs = [ox + (i + 0.5) * res for i, _ in comp]
        ys = [oy + (j + 0.5) * res for _, j in comp]
        hs = [m.data[j * W + i] * unit for i, j in comp]
        ps = [prior(x, y) for x, y in zip(xs, ys)]
        ps = [p for p in ps if p is not None]
        pr = f"{min(ps):.2f}~{max(ps):.2f}" if ps else "맵 밖"
        print(f"  {len(comp):5d}   {min(xs):5.2f}~{max(xs):5.2f}      {min(ys):5.2f}~{max(ys):5.2f}"
              f"      {min(hs):.2f}~{max(hs):.2f}      {pr}")
    print("\nprior 높이와 잰 높이가 크게 다르면 그 칸은 센서가 잘못 잰 것이다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
