"""시뮬레이션 주행 궤적 기록기 — RViz 없이 실제 실행 데이터를 받아온다.

왜 이게 필요한가
    포스터에 쓸 그림은 Python 벤치마크 그래프가 아니라
    '실제로 Gazebo 에서 주행한 궤적'이다. 그런데 RViz 를 띄우면
    GPU·X 서버를 점유해 원격 데스크톱이 먹통이 된 전례가 두 번 있다.

    그래서 화면 없이 토픽만 받아 JSON 으로 떨군다.
    그림은 macOS 쪽에서 benchmark/plot_sim_run.py 로 그린다.

받는 것
    /plan                 플래너가 낸 계획 경로 (nav_msgs/Path)
    /mode_switch_points   이·착륙 지점 (drobot_msgs/ModeSwitchPlan)  ← 우리 기여
    /odom                 실제 주행 궤적 (nav_msgs/Odometry)

이 파일은 colcon 설치 없이 python3 로 바로 실행된다 — 빌드가 필요 없다.

사용법 (컨테이너 안에서)
    python3 record_run.py --world easy_open --out /app/sim_easy_open.json
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose

try:
    from drobot_msgs.msg import ModeSwitchPlan
except ImportError:            # 메시지 패키지가 없으면 그 토픽만 건너뛴다
    ModeSwitchPlan = None

# benchmark/envs/heightmap.py 와 config/spawn_positions.yaml 의 값.
# 여기 하드코딩하는 이유: 컨테이너 안에서 benchmark/ 를 import 하지 않으려고.
GOALS = {
    "easy_open":       (18.5, 7.0),
    "easy_corridor":   (18.5, 2.0),
    "medium_open":     (20.5, 7.0),
    "medium_corridor": (20.5, 2.0),
    "hard_open":       (22.5, 8.0),
    "hard_corridor":   (22.5, 8.0),
    # 독립연구 통제 맵 — 전부 같은 시작(2,1)/목표(2,10), 장애물 높이만 다르다
    "base_map_h0.05":  (2.0, 10.0),
    "base_map_h0.3":   (2.0, 10.0),
    "base_map_h0.5":   (2.0, 10.0),
    "base_map_h1":     (2.0, 10.0),
    "base_map_h1.8":   (2.0, 10.0),
    "base_map_h2.5":   (2.0, 10.0),
}

LATCHED = QoSProfile(depth=1,
                     reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class Recorder(Node):
    def __init__(self, world: str, goal_xy, timeout: float):
        super().__init__("drobot_run_recorder")
        self.world = world
        self.goal_xy = goal_xy
        self.timeout = timeout

        self.plan = []            # 플래너가 낸 마지막 계획
        self.plan_history = 0     # 재계획 횟수 — 몇 번 다시 짰는지
        self.odom = []            # 실제 주행 궤적 [(t, x, y, yaw)]
        self.switches = []        # 이·착륙 지점
        self.plan_msg = None
        self.t0 = time.time()
        self.result = None

        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        if ModeSwitchPlan is not None:
            self.create_subscription(ModeSwitchPlan, "/mode_switch_points",
                                     self._on_switch, LATCHED)

        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")

    # ---- 콜백 ----
    def _on_plan(self, msg: Path):
        self.plan = [(p.pose.position.x, p.pose.position.y, p.pose.position.z)
                     for p in msg.poses]
        self.plan_msg = msg          # --hold 로 재발행할 원본
        self.plan_history += 1

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        self.odom.append((round(time.time() - self.t0, 3),
                          round(p.x, 4), round(p.y, 4), round(yaw, 4)))

    def _on_switch(self, msg):
        self.switches = [{
            "x": s.position.x, "y": s.position.y,
            "altitude": s.flight_altitude,
            "type": int(s.switch_type),
            "energy_wh": s.estimated_energy_cost,
            "pair_id": int(s.pair_id),
        } for s in msg.switch_points]

    # ---- 실행 ----
    def send_goal(self) -> bool:
        self.get_logger().info("navigate_to_pose 액션 서버 대기 중...")
        if not self.ac.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("액션 서버가 안 뜬다 — Nav2 가 활성화됐는지 확인할 것")
            return False

        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(self.goal_xy[0])
        g.pose.pose.position.y = float(self.goal_xy[1])
        g.pose.pose.orientation.w = 1.0

        self.get_logger().info(f"목표 전송 {self.goal_xy}")
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("목표가 거부됐다")
            return False

        res_fut = handle.get_result_async()
        deadline = time.time() + self.timeout
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if res_fut.done():
                # 완료 != 성공. 예전에 이걸 구분 안 해서 bt_navigator 가
                # "Goal failed" 를 냈는데도 JSON 에 succeeded 로 적혔다.
                # 실패한 실행을 성공으로 기록하면 결과 해석이 통째로 틀어진다.
                st = res_fut.result().status
                ok = st == GoalStatus.STATUS_SUCCEEDED
                self.result = {
                    GoalStatus.STATUS_SUCCEEDED: "succeeded",
                    GoalStatus.STATUS_ABORTED: "aborted",
                    GoalStatus.STATUS_CANCELED: "canceled",
                }.get(st, f"status_{st}")
                if ok:
                    self.get_logger().info("목표 도달")
                else:
                    self.get_logger().error(f"목표 실패 — {self.result}")
                return True
        self.result = "timeout"
        self.get_logger().warn(f"{self.timeout}초 안에 도달하지 못했다 "
                               f"(그래도 기록한 궤적은 저장한다)")
        return True

    def dump(self, path: str):
        data = {
            "world": self.world,
            "goal": list(self.goal_xy),
            "result": self.result,
            "n_replans": self.plan_history,
            "plan": self.plan,
            "odom": self.odom,
            "mode_switches": self.switches,
            "duration_s": round(time.time() - self.t0, 2),
        }
        with open(path, "w") as f:
            json.dump(data, f)
        print(f"\n저장 {path}")
        print(f"  계획 경로 {len(self.plan)}점 · 재계획 {self.plan_history}회")
        print(f"  주행 궤적 {len(self.odom)}점 · {data['duration_s']}초")
        print(f"  모드 전환 {len(self.switches)}회  <- 0 이면 비행을 안 썼다는 뜻")
        print(f"  결과 {self.result}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", required=True, choices=sorted(GOALS))
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=180.0)
    ap.add_argument("--hold", action="store_true",
                    help="기록 후 계획 경로를 latch 토픽으로 계속 발행 (RViz 캡처용)")
    a = ap.parse_args()

    rclpy.init()
    node = Recorder(a.world, GOALS[a.world], a.timeout)

    # Nav2 가 뜨고 초기 위치가 잡힐 때까지 잠깐 받아둔다
    node.get_logger().info("토픽 수신 대기 (5초)")
    t = time.time()
    while time.time() - t < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    ok = node.send_goal()
    node.dump(a.out)

    if a.hold:
        # Nav2 의 /plan 은 latch 가 아니라, 목표가 끝나면 발행이 멈춘다.
        # 그러면 나중에 RViz 를 켰을 때 아무것도 안 보인다.
        # 마지막으로 받은 계획을 transient_local 로 다시 내보내 붙잡아둔다.
        if node.plan_msg is None:
            node.get_logger().error("붙잡아둘 계획이 없다 (/plan 을 한 번도 못 받음)")
        else:
            pub = node.create_publisher(Path, "/viz/plan_latched", LATCHED)
            pub.publish(node.plan_msg)
            print(f"\n/viz/plan_latched 로 {len(node.plan_msg.poses)}점 붙잡아뒀다.")
            print("RViz 에서 Path 디스플레이로 이 토픽을 추가하면 언제 켜도 보인다.")
            print("(Durability Policy 를 Transient Local 로 둘 것)")
            print("Ctrl+C 로 종료.")
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
