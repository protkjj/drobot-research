"""A* / RRT* 경로를 RViz 용 토픽으로 발행한다.

왜 이게 필요한가
    RRT* 는 Nav2 플러그인이 아니다 (C++ 에는 A* 만 등록돼 있고 RRT* 는
    benchmark/planners/rrt_star.py 에만 있다). 그래서 Nav2 에 목표를 주는
    방식으로는 RRT* 경로를 RViz 에 띄울 수 없다.

    대신 Python 벤치마크로 두 경로를 같은 맵·같은 비용함수에서 계산해
    nav_msgs/Path 로 발행한다. 포스터 캡션에는 "Nav2 실시간 실행" 이 아니라
    "같은 문제를 푼 결과를 시각화" 로 적어야 정확하다.

발행 토픽
    /viz/map            nav_msgs/OccupancyGrid   지형 (높이 등급별 cost)
    /viz/path_astar     nav_msgs/Path
    /viz/path_rrt       nav_msgs/Path            최선 시드
    /viz/path_rrt_seeds nav_msgs/Path x N        시드별 (비결정성 표시용)
    /viz/markers        visualization_msgs/MarkerArray  시작·목표·장애물 높이

전부 transient_local 이라 RViz 를 나중에 켜도 받는다.

실행 (컨테이너 안)
    python3 publish_paths.py --map base_map_h0.5
    python3 publish_paths.py --map base_map_h1.8 --energy derived
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# 벤치마크 코드는 저장소 루트의 benchmark/ 에 있다.
# 컨테이너에서는 /app/benchmark, 호스트에서는 <repo>/benchmark.
_HERE = Path(__file__).resolve()
for cand in (Path("/app/benchmark"), _HERE.parents[3] / "benchmark"):
    if cand.exists():
        sys.path.insert(0, str(cand))
        break

from cost.energy import EnergyModel            # noqa: E402
from envs.heightmap import (                   # noqa: E402
    load_base_map, build_all, ROVER_MAX, H_3D_LIMIT, H_4D_LIMIT)
from planners.state_space import ProblemSpec, AIR   # noqa: E402
from planners.grid_search import astar         # noqa: E402
from planners.rrt_star import rrt_star         # noqa: E402
from planners.smoothing import smooth_result   # noqa: E402

LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

FRAME = "map"
N_SEEDS = 5
RRT_BUDGET = 2.0


def to_path_msg(waypoints, stamp) -> PathMsg:
    m = PathMsg()
    m.header.frame_id = FRAME
    m.header.stamp = stamp
    for x, y, z, mode in waypoints:
        p = PoseStamped()
        p.header = m.header
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        # 비행 구간은 실제 고도로 띄운다 — RViz 에서 3D 로 보인다
        p.pose.position.z = float(z) if mode == AIR else 0.02
        p.pose.orientation.w = 1.0
        m.poses.append(p)
    return m


def to_occupancy(hm, stamp) -> OccupancyGrid:
    """높이맵을 OccupancyGrid 로. 값은 높이 등급을 그대로 반영한다.

    RViz 기본 Map 색상은 0=흰색, 100=검정이므로
    등급이 올라갈수록 어두워진다.
    """
    g = OccupancyGrid()
    g.header.frame_id = FRAME
    g.header.stamp = stamp
    g.info.resolution = float(hm.resolution)
    g.info.width = int(hm.nx)
    g.info.height = int(hm.ny)
    g.info.origin.position.x = 0.0
    g.info.origin.position.y = 0.0
    g.info.origin.orientation.w = 1.0

    data = []
    for row in hm.grid:
        for h in row:
            if h <= ROVER_MAX:
                data.append(0)         # 평지
            elif h <= H_3D_LIMIT:
                data.append(40)        # 비행으로 넘을 수 있음
            elif h <= H_4D_LIMIT:
                data.append(70)        # 4D 전용
            else:
                data.append(100)       # 통과 불가
    g.data = data
    return g


def make_markers(hm, stamp) -> MarkerArray:
    arr = MarkerArray()

    def base(i, kind):
        m = Marker()
        m.header.frame_id = FRAME
        m.header.stamp = stamp
        m.ns = "viz"
        m.id = i
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    s = base(0, Marker.CUBE)
    s.pose.position.x, s.pose.position.y, s.pose.position.z = hm.start[0], hm.start[1], 0.1
    s.scale.x = s.scale.y = s.scale.z = 0.25
    s.color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
    arr.markers.append(s)

    g = base(1, Marker.SPHERE)
    g.pose.position.x, g.pose.position.y, g.pose.position.z = hm.goal[0], hm.goal[1], 0.15
    g.scale.x = g.scale.y = g.scale.z = 0.35
    g.color = ColorRGBA(r=0.95, g=0.75, b=0.1, a=1.0)
    arr.markers.append(g)

    # 장애물 높이를 글자로 — 어느 맵인지 한눈에 보이게
    hs = sorted({float(v) for v in hm.grid.flatten()} - {0.0})
    mid = [h for h in hs if h < 2.9]
    t = base(2, Marker.TEXT_VIEW_FACING)
    t.pose.position.x, t.pose.position.y, t.pose.position.z = \
        hm.width_m / 2, hm.height_m + 0.5, 0.5
    t.scale.z = 0.45
    t.color = ColorRGBA(r=0.1, g=0.1, b=0.1, a=1.0)
    t.text = f"{hm.name}   장애물 높이 {mid[0] if mid else 0:.2f} m"
    arr.markers.append(t)
    return arr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="base_map_h0.5")
    ap.add_argument("--energy", choices=["default", "derived"], default="derived")
    ap.add_argument("--seeds", type=int, default=N_SEEDS)
    a = ap.parse_args()

    cfg = Path("/app/src/drobot_hybrid_planner/config")
    if not cfg.exists():
        cfg = _HERE.parents[3] / "src/drobot_hybrid_planner/config"
    yml = cfg / ("energy_params_derived.yaml" if a.energy == "derived"
                 else "energy_params.yaml")

    model = EnergyModel.from_yaml(yml)
    hm = (load_base_map(a.map) if a.map.startswith("base_map")
          else build_all(res=0.1)[a.map])
    spec = ProblemSpec(hm=hm, model=model, dims=3)

    print(f"맵 {hm.name}  {hm.width_m}x{hm.height_m} m   에너지 세트 {a.energy}")

    ra = astar(spec, timeout=120.0)
    if not ra.found:
        sys.exit("A* 가 해를 못 찾았다")
    ca, acc_a, path_a = smooth_result(spec, ra, is_grid=True)
    print(f"  A*    비용 {ca:.3f}  전환 {acc_a.n_switches}  점 {len(path_a)}")

    seeds = []
    for s in range(a.seeds):
        r = rrt_star(spec, seed=s, max_samples=10 ** 7, timeout=RRT_BUDGET)
        if not r.found:
            continue
        c, acc, pth = smooth_result(spec, r, is_grid=False)
        seeds.append((c, acc, pth))
    if not seeds:
        sys.exit("RRT* 가 해를 못 찾았다")
    seeds.sort(key=lambda t: t[0])
    cr, acc_r, path_r = seeds[0]
    print(f"  RRT*  비용 {cr:.3f}  전환 {acc_r.n_switches}  "
          f"({len(seeds)}/{a.seeds} 시드 성공, 최선값)")

    rclpy.init()
    node = Node("drobot_path_publisher")
    now = node.get_clock().now().to_msg()

    pubs = {
        "map": node.create_publisher(OccupancyGrid, "/viz/map", LATCHED),
        "astar": node.create_publisher(PathMsg, "/viz/path_astar", LATCHED),
        "rrt": node.create_publisher(PathMsg, "/viz/path_rrt", LATCHED),
        "markers": node.create_publisher(MarkerArray, "/viz/markers", LATCHED),
    }
    seed_pubs = [node.create_publisher(PathMsg, f"/viz/path_rrt_seed{i}", LATCHED)
                 for i in range(len(seeds))]

    pubs["map"].publish(to_occupancy(hm, now))
    pubs["astar"].publish(to_path_msg(path_a, now))
    pubs["rrt"].publish(to_path_msg(path_r, now))
    pubs["markers"].publish(make_markers(hm, now))
    for p, (_, _, pth) in zip(seed_pubs, seeds):
        p.publish(to_path_msg(pth, now))

    print("\n발행 완료 — RViz 에서 아래 토픽을 추가하라 (전부 transient_local)")
    print("  /viz/map            Map")
    print("  /viz/path_astar     Path   (A*)")
    print("  /viz/path_rrt       Path   (RRT* 최선)")
    for i in range(len(seeds)):
        print(f"  /viz/path_rrt_seed{i}  Path   (시드 {i})")
    print("  /viz/markers        MarkerArray")
    print("\nCtrl+C 로 종료. 그 전까지 계속 latch 된다.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
