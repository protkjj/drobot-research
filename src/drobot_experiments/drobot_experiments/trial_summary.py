#!/usr/bin/env python3
"""실험 한 번을 한 줄로 요약한다 — trial_summary.csv.

왜 필요한가
    record_run 이 떠오는 JSON 은 궤적 수천 점짜리 원자료다. 그걸 그대로
    비교하려면 매번 파싱해야 한다. 빌드 매니페스트가 experiments 의 완료
    기준으로 "trial_summary 1줄" 을 든 이유다 — 한 실험 = 한 행이면
    표 하나로 전체 비교가 끝난다.

    열 이름과 순서는 config/logging_config.yaml 의 csv_schemas.trial_summary
    를 그대로 따른다. 거기가 진실의 출처다.

왜 ROS 를 import 하지 않나
    여기 들어가는 계산(이동거리, 전환 횟수, 비행 에너지)은 전부 JSON 만
    있으면 된다. 떼어두면 이미 받아둔 기록으로 바로 검증할 수 있고,
    나중에 열을 추가할 때 시뮬을 띄우지 않아도 된다.

사용법
    # 기록 하나 요약해서 CSV 에 덧붙이기
    python3 trial_summary.py sim_base_map_h0.5_derived.json

    # 여러 개 한 번에 (이미 쌓인 기록 소급 정리)
    python3 trial_summary.py benchmark/results/sim_*.json --out data/trial_summary.csv
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

# config/logging_config.yaml 의 csv_schemas.trial_summary 와 같아야 한다.
FIELDS = [
    "trial_id",
    "world",
    "planner",
    "total_energy_wh",
    "total_time_s",
    "total_distance_m",
    "num_switches",
    "success",
    "flight_energy_wh",
    "ground_energy_wh",
    "planner_time_s",
]

# 측정값이 없을 때 쓰는 표시.
# 0 을 넣지 않는 이유: "0 Wh 썼다" 와 "안 쟀다" 는 전혀 다른 정보고,
# 0 으로 두면 나중에 평균을 낼 때 조용히 결과를 끌어내린다.
UNKNOWN = ""


def path_length(points) -> float:
    """[[t, x, y, yaw], ...] 또는 [[x, y, z], ...] 의 누적 이동거리."""
    if not points or len(points) < 2:
        return 0.0
    # odom 은 (t, x, y, yaw), plan 은 (x, y, z) 라 좌표 위치가 다르다
    xy = ([(p[1], p[2]) for p in points] if len(points[0]) == 4
          else [(p[0], p[1]) for p in points])
    return sum(math.dist(xy[i], xy[i + 1]) for i in range(len(xy) - 1))


def summarize(run: dict, planner: str, trial_id: int) -> dict:
    """record_run 의 JSON 한 건을 한 행으로."""
    switches = run.get("mode_switches", [])
    # 비행 에너지는 플래너의 '예상값' 이다. 실측(INA226)이 아니다.
    flight_wh = sum(s.get("energy_wh", 0.0) for s in switches)

    return {
        "trial_id": trial_id,
        "world": run.get("world", ""),
        "planner": planner,
        # 아래 둘은 INA226 실측이 있어야 채워진다 (Phase 2)
        "total_energy_wh": UNKNOWN,
        "total_time_s": round(float(run.get("duration_s", 0.0)), 2),
        "total_distance_m": round(path_length(run.get("odom", [])), 3),
        "num_switches": len(switches),
        # 완료 != 성공. record_run 이 GoalStatus 로 판정한 값만 믿는다.
        "success": run.get("result") == "succeeded",
        "flight_energy_wh": round(flight_wh, 3) if switches else 0.0,
        "ground_energy_wh": UNKNOWN,
        "planner_time_s": (round(float(run["planner_time_s"]), 3)
                           if run.get("planner_time_s") is not None else UNKNOWN),
    }


def next_trial_id(csv_path: Path) -> int:
    """이어붙일 다음 번호. 파일이 없으면 1."""
    if not csv_path.exists():
        return 1
    with open(csv_path, newline="") as f:
        ids = [int(r["trial_id"]) for r in csv.DictReader(f)
               if r.get("trial_id", "").isdigit()]
    return max(ids) + 1 if ids else 1


def append_row(csv_path: Path, row: dict) -> None:
    """헤더가 없으면 먼저 쓰고 한 행 덧붙인다."""
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    new = not csv_path.exists()
    with open(csv_path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)


def record(run: dict, csv_path: Path, planner: str = "proposed") -> dict:
    """요약 한 행을 만들어 CSV 에 덧붙이고 그 행을 돌려준다."""
    row = summarize(run, planner, next_trial_id(csv_path))
    append_row(csv_path, row)
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("json", nargs="+", help="record_run 이 만든 JSON")
    ap.add_argument("--out", default="data/trial_summary.csv")
    ap.add_argument("--planner", default="proposed",
                    choices=["smac2d", "drone_only", "shortest_hybrid", "proposed"])
    a = ap.parse_args()

    out = Path(a.out)
    for p in a.json:
        row = record(json.load(open(p)), out, a.planner)
        print(f"  trial {row['trial_id']:>3}  {row['world']:<20} "
              f"{str(row['success']):<5}  "
              f"{row['total_distance_m']:>7.2f} m  "
              f"{row['total_time_s']:>6.1f} s  전환 {row['num_switches']}")
    print(f"\n저장 {out}")


if __name__ == "__main__":
    sys.exit(main())
