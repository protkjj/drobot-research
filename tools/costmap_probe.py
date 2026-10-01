#!/usr/bin/env python3
"""costmap 의 실제 cost 값을 선분을 따라 찍는다.

왜 필요한가
    플래너가 왜 거기서 이륙하고 거기에 착륙하는지는 costmap 이 그 셀을
    뭐라고 보는지에 달려 있다. 추측 대신 값을 직접 읽는다.

어느 토픽을 읽나 — /…/costmap 이 아니라 /…/costmap_raw
    Nav2 는 /…/costmap (OccupancyGrid) 으로 낼 때 0~255 를 0~100 으로 환산한다
    (nav2_costmap_2d/src/costmap_2d_publisher.cpp 의 cost_translation_table_):
        0 -> 0,  1~252 -> 1 + 97*(c-1)/251,  253 -> 99,  254 -> 100,  255 -> -1
    그래서 그 토픽에서는 rover(100) 가 39, fly_over(200) 가 77, LETHAL 이 100 으로
    보이고, inflation 값과 섞여 등급을 구분할 수 없다.
    /…/costmap_raw (nav2_msgs/Costmap) 은 환산 없이 0~255 를 그대로 담는다.
    (이전 버전은 /…/costmap 을 읽고 100 을 rover 로 표시했다 — 실제로는 LETHAL.)

    cost 등급 (인터페이스 규약 ③, elevation_layer 의 cost_values)
        0   free               평지
        100 rover_traversable  주행 가능(지형 거침)
        200 fly_over           비행 필요
        253 inscribed          inflation_layer 가 장애물 바로 옆에 쓰는 값
        254 impassable         통과 불가 (LETHAL)
        255 NO_INFORMATION     미지
    그 밖의 값은 inflation_layer 가 만든 것이다. 100·200 도 inflation 이
    우연히 그 값을 만들 수 있다 (master costmap 은 레이어별 최댓값이라 구분 불가).

'플래너 판독' 열
    HybridAStarPlanner 의 CostmapTerrainSource (drobot_hybrid_planner
    state_space.hpp/.cpp) 는 master costmap 의 cost 를 이렇게 등급으로 읽는다:
        <= 50 free · <= 150 rover · <= 253 fly_over · 그 외 impassable
    inflation 값이 지형 등급으로 읽히는 셀에 ⚠ 를 붙인다.

사용 (컨테이너 안)
    python3 /app/tools/costmap_probe.py --x0 2.12 --y0 1.5 --x1 2.12 --y1 5.5
    python3 /app/tools/costmap_probe.py --topic /local_costmap/costmap_raw --x0 ...
    python3 /app/tools/costmap_probe.py --mode elevation \\
        --topic /global_costmap/elevation_grid --x0 ...
"""
from __future__ import annotations

import argparse
import math
import sys
import time

# --- 등급 값 (elevation_layer cost_values, nav2_costmap_2d 상수) ------------
FREE, ROVER, FLY_OVER = 0, 100, 200
INSCRIBED, LETHAL, NO_INFORMATION = 253, 254, 255
TERRAIN_CLASSES = {FREE: "free", ROVER: "rover_traversable", FLY_OVER: "fly_over",
                   LETHAL: "impassable(LETHAL)", NO_INFORMATION: "미지"}

# --- CostmapTerrainSource::Config 기본값 (state_space.hpp) ------------------
PLANNER_FREE_MAX, PLANNER_ROVER_MAX, PLANNER_FLYOVER_MAX = 50, 150, 253


def label_raw_cost(c: int) -> str:
    """costmap_raw 값(0~255)의 뜻."""
    if c in TERRAIN_CLASSES:
        return TERRAIN_CLASSES[c]
    if c == INSCRIBED:
        return "inflation(inscribed 253)"
    return f"inflation({c})"


def planner_reading(c: int) -> str:
    """CostmapTerrainSource::heightAt 이 이 cost 를 어느 등급으로 읽는가."""
    if c <= PLANNER_FREE_MAX:
        return "free"
    if c <= PLANNER_ROVER_MAX:
        return "rover"
    if c <= PLANNER_FLYOVER_MAX:
        return "fly_over"
    return "impassable"


def misread_mark(c: int) -> str:
    """inflation 값인데 플래너가 free 가 아닌 지형으로 읽으면 표시한다."""
    is_inflation = c not in TERRAIN_CLASSES
    return " ⚠" if is_inflation and planner_reading(c) != "free" else ""


def label_elev(v: int, fly_over_max: float) -> str:
    """elevation_grid 토픽 값. cost 가 아니라 '높이' 다.

    ElevationLayer::publishElevationGrid 가 이렇게 채운다:
        관측 안 됨          -> -1
        관측됨              -> (max_z / fly_over_max) * 100
    그래서 값을 cost 로 읽으면 안 된다 (한 번 그렇게 오독했다).
    """
    if v < 0:
        return "미관측"
    h = v / 100.0 * fly_over_max
    if v >= 100:
        return f"{h:.2f} m 이상 (상한 포화)"
    return f"{h:.2f} m"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default=None,
                    help="기본: cost 모드 /global_costmap/costmap_raw, "
                         "elevation 모드 /global_costmap/elevation_grid")
    ap.add_argument("--x0", type=float, required=True)
    ap.add_argument("--y0", type=float, required=True)
    ap.add_argument("--x1", type=float, required=True)
    ap.add_argument("--y1", type=float, required=True)
    ap.add_argument("--step", type=float, default=0.1)
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--mode", choices=["cost", "elevation"], default="cost",
                    help="cost: costmap_raw 등급 / elevation: elevation_grid 의 높이")
    ap.add_argument("--fly-over-max", type=float, default=1.2,
                    help="elevation 모드의 스케일 상한 (nav2_params 의 fly_over_max)")
    a = ap.parse_args()

    topic = a.topic or ("/global_costmap/costmap_raw" if a.mode == "cost"
                        else "/global_costmap/elevation_grid")
    if a.mode == "cost" and not topic.endswith("_raw"):
        sys.exit(f"{topic} 는 0~100 으로 환산된 토픽이라 등급을 구분할 수 없다. "
                 f"{topic}_raw 를 쓸 것 (모듈 설명 참고)")

    # ROS 는 여기서 import 한다 — 위 함수들은 ROS 없는 곳에서도 시험할 수 있게.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

    latched = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

    rclpy.init()
    node = Node("costmap_probe")
    box = {}

    if a.mode == "cost":
        from nav2_msgs.msg import Costmap
        node.create_subscription(Costmap, topic, lambda m: box.setdefault("m", m), latched)
    else:
        from nav_msgs.msg import OccupancyGrid
        node.create_subscription(OccupancyGrid, topic, lambda m: box.setdefault("m", m), latched)

    t0 = time.time()
    while "m" not in box and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    if "m" not in box:
        sys.exit(f"{topic} 를 못 받았다")

    m = box["m"]
    # 두 메시지의 격자 정보 필드 이름이 다르다
    if a.mode == "cost":
        info = m.metadata
        W, H = info.size_x, info.size_y
    else:
        info = m.info
        W, H = info.width, info.height
    res = info.resolution
    ox, oy = info.origin.position.x, info.origin.position.y

    print(f"{topic}  {W}x{H} @ {res} m  원점 ({ox:.2f}, {oy:.2f})  모드 {a.mode}")
    if a.mode == "cost":
        print(f"{'거리':>6} {'x':>7} {'y':>7} {'cell':>12} {'값':>5}  "
              f"{'의미':<26} 플래너 판독")
    else:
        print(f"{'거리':>6} {'x':>7} {'y':>7} {'cell':>12} {'값':>5}  "
              f"높이 (상한 {a.fly_over_max} m)")

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
        v = m.data[my * W + mx]
        head = f"{d*f:6.2f} {x:7.2f} {y:7.2f} {f'({mx},{my})':>12} {v:5d}  "
        if a.mode == "cost":
            print(head + f"{label_raw_cost(v):<26} {planner_reading(v)}{misread_mark(v)}")
        else:
            print(head + label_elev(v, a.fly_over_max))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
