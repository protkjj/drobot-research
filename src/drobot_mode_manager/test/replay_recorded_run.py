#!/usr/bin/env python3
"""기록된 주행을 전환 판정기에 흘려보낸다 — ROS 없이.

왜 이게 필요한가
    전환이 안 일어났을 때 원인이 둘이다.
        ① 판정기가 지점을 못 집었다
        ② 로봇이 거기까지 못 갔다
    시뮬을 띄워서 보면 둘이 섞여 구분이 안 된다. record_run.py 가 떠온
    JSON 에는 실제 궤적(/odom)과 전환점(/mode_switch_points)이 둘 다
    들어 있으므로, 그걸 판정기에 그대로 먹이면 ①을 따로 확인할 수 있다.

사용법
    python3 src/drobot_mode_manager/test/replay_recorded_run.py \
        benchmark/results/sim_base_map_h0.5_derived.json
    (인자 없으면 benchmark/results 의 sim_*.json 전부)
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[1]))          # src/drobot_mode_manager

from drobot_mode_manager.switch_tracker import (   # noqa: E402
    SwitchPoint, SwitchTracker)

REPO = HERE.parents[3]


def replay(path: Path) -> dict:
    d = json.load(open(path))
    pts = [SwitchPoint(x=s["x"], y=s["y"], switch_type=s["type"],
                       flight_altitude=s["altitude"], energy_wh=s["energy_wh"],
                       pair_id=s["pair_id"])
           for s in d["mode_switches"]]

    tr = SwitchTracker()
    tr.set_plan(pts)

    print(f"\n{'=' * 78}\n{path.name}\n{'=' * 78}")
    print(f"  기록 결과 {d['result']} · {d['duration_s']}초 · "
          f"재계획 {d['n_replans']}회 · 전환점 {len(pts)}개")

    for level, text in tr.validate():
        mark = {"error": "❌", "warn": "⚠ ", "info": "  "}[level]
        print(f"  {mark} {text}")

    # 기록된 궤적을 시간 순서대로 먹인다.
    # odom 은 [t, x, y, yaw] 이고 속도는 안 들어 있으므로
    # 연속한 두 점의 차분으로 추정한다.
    odom = d["odom"]
    events = []
    for i, (t, x, y, _) in enumerate(odom):
        if i == 0:
            speed = 0.0
        else:
            pt, px, py = odom[i - 1][0], odom[i - 1][1], odom[i - 1][2]
            dt = t - pt
            speed = math.dist((x, y), (px, py)) / dt if dt > 1e-6 else 0.0
        ev = tr.update(t, x, y, speed)
        if ev is None:
            continue
        events.append(ev)
        if ev.skipped:
            print(f"  ⚠  t={t:6.2f}s  {ev.point.label}점 — 상태 불일치로 건너뜀")
        else:
            print(f"  ✅ t={t:6.2f}s  {ev.point.label}점 도달 "
                  f"({ev.point.x:.2f}, {ev.point.y:.2f})  "
                  f"오차 {ev.error_m:.2f} m · 속도 {ev.speed:.3f} m/s · "
                  f"{'정지' if ev.stopped else '주행 중'}")
            tr.complete(t)

    # 못 잡은 전환점이 있으면 왜인지 숫자로 남긴다
    if not tr.done:
        nxt = tr.points[tr.idx]
        nearest = min(math.dist((o[1], o[2]), (nxt.x, nxt.y)) for o in odom)
        moved = sum(math.dist((odom[i][1], odom[i][2]), (odom[i + 1][1], odom[i + 1][2]))
                    for i in range(len(odom) - 1))
        print(f"  ❌ 미처리 {tr.remaining}개 — 다음 {nxt.label}점까지 "
              f"최소 접근 {nearest:.2f} m (도달반경 {tr.arrival_radius} m)")
        print(f"     로봇 총 이동거리 {moved:.2f} m "
              f"— {'판정기가 아니라 주행이 문제다' if nearest > tr.arrival_radius else ''}")

    return {"file": path.name, "detected": len([e for e in events if not e.skipped]),
            "expected": len(pts), "done": tr.done}


def main():
    args = sys.argv[1:]
    files = ([Path(a) for a in args] if args
             else sorted((REPO / "benchmark/results").glob("sim_*.json")))
    if not files:
        sys.exit("기록 파일이 없다")

    rows = [replay(f) for f in files]
    print(f"\n{'=' * 78}\n요약\n{'=' * 78}")
    for r in rows:
        print(f"  {r['file']:<40} 감지 {r['detected']}/{r['expected']}  "
              f"{'완주' if r['done'] else '미완'}")


if __name__ == "__main__":
    main()
