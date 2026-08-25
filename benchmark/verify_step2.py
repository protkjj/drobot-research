"""2단계 검증 — 맵 6종이 설계 요구사항을 실제로 만족하는가.

요구사항 (1단계 검증에서 도출)
  R1. 각 맵에 서로 다른 높이의 장애물이 인접해 존재      (V4 근거)
  R2. 0.95~1.55m 높이 장애물 포함 (3D 불가 / 4D 가능)    (V5/V6 근거)
  R3. hard_corridor는 지상 전용으로 도달 불가             (baseline 비교용)

추가 확인
  C1. start/goal이 유효한 위치인가 (벽 속에 박혀있지 않은가)
  C2. 4D로는 도달 가능한가 (아예 못 푸는 맵이면 실험 불가)
  C3. 맵 크기가 A* 탐색에 현실적인가
"""
import sys
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from envs.heightmap import (  # noqa: E402
    ALL_MAPS, ROVER_MAX, H_3D_LIMIT, H_4D_LIMIT, build_all,
)

SEP = "=" * 78


def reachable_ground(hm) -> np.ndarray:
    """지상 주행만으로 start에서 도달 가능한 셀 집합 (BFS, 4-이웃).

    로버는 높이 <= ROVER_MAX 인 셀만 지날 수 있다.
    """
    passable = hm.grid <= ROVER_MAX
    seen = np.zeros_like(passable, dtype=bool)
    sx, sy = hm.to_idx(*hm.start)
    if not (hm.in_bounds(sx, sy) and passable[sy, sx]):
        return seen
    seen[sy, sx] = True
    q = deque([(sx, sy)])
    while q:
        x, y = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx_, ny_ = x + dx, y + dy
            if hm.in_bounds(nx_, ny_) and passable[ny_, nx_] and not seen[ny_, nx_]:
                seen[ny_, nx_] = True
                q.append((nx_, ny_))
    return seen


def reachable_hybrid(hm, h_limit: float) -> np.ndarray:
    """지상 주행 + 비행(높이 <= h_limit 인 장애물은 넘을 수 있음)으로 도달 가능한 셀.

    h_limit 초과 셀은 벽으로 간주.
    실제 플래너는 이륙/착륙 지점 제약이 있지만, 여기서는
    '원리적으로 도달 가능한가'만 보므로 단순화한다.
    """
    passable = hm.grid <= h_limit
    seen = np.zeros_like(passable, dtype=bool)
    sx, sy = hm.to_idx(*hm.start)
    if not (hm.in_bounds(sx, sy) and passable[sy, sx]):
        return seen
    seen[sy, sx] = True
    q = deque([(sx, sy)])
    while q:
        x, y = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx_, ny_ = x + dx, y + dy
            if hm.in_bounds(nx_, ny_) and passable[ny_, nx_] and not seen[ny_, nx_]:
                seen[ny_, nx_] = True
                q.append((nx_, ny_))
    return seen


def flight_segment_height_variation(hm, seg_len: float = 5.0,
                                    min_delta: float = 0.3) -> tuple[int, float]:
    """한 비행 구간 안에 높이가 다른 장애물이 몇 쌍이나 있는가 (R1 확인).

    R1의 취지는 '셀이 딱 붙어 있는가'가 아니라
    '한 번의 비행으로 통과하는 범위 안에서 지형 높이가 변하는가'다.
    hybrid_rrt_params.yaml 의 max_flight_segment(5.0m)를 그 범위로 삼는다.

    반환: (조건을 만족하는 장애물 쌍의 수, 그 중 최대 높이차)
    """
    g = hm.grid
    # 넘어갈 수 있는 장애물만 대상 (벽은 고도 계획과 무관)
    obst_mask = (g > ROVER_MAX) & (g <= H_4D_LIMIT)
    ys, xs = np.nonzero(obst_mask)
    if len(xs) == 0:
        return 0, 0.0

    # 셀 단위로 전수 비교하면 O(n^2)이라 비싸다.
    # 연결된 장애물 덩어리(블록)를 대표점으로 축약해서 비교한다.
    seen = np.zeros_like(obst_mask, dtype=bool)
    blocks = []  # (cx, cy, height)
    for y0, x0 in zip(ys, xs):
        if seen[y0, x0]:
            continue
        h0 = round(float(g[y0, x0]), 3)
        # 같은 높이로 연결된 영역을 BFS로 수집
        comp = []
        q = deque([(x0, y0)])
        seen[y0, x0] = True
        while q:
            x, y = q.popleft()
            comp.append((x, y))
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx_, ny_ = x + dx, y + dy
                if (hm.in_bounds(nx_, ny_) and not seen[ny_, nx_]
                        and obst_mask[ny_, nx_]
                        and abs(float(g[ny_, nx_]) - h0) < 1e-6):
                    seen[ny_, nx_] = True
                    q.append((nx_, ny_))
        cx = np.mean([c[0] for c in comp]) * hm.resolution
        cy = np.mean([c[1] for c in comp]) * hm.resolution
        blocks.append((cx, cy, h0))

    pairs, max_delta = 0, 0.0
    for i in range(len(blocks)):
        for j in range(i + 1, len(blocks)):
            x1, y1, h1 = blocks[i]
            x2, y2, h2 = blocks[j]
            dist = np.hypot(x1 - x2, y1 - y2)
            delta = abs(h1 - h2)
            if dist <= seg_len and delta >= min_delta:
                pairs += 1
                max_delta = max(max_delta, delta)
    return pairs, max_delta


print(SEP)
print("2단계 검증 — 실험 맵 6종")
print(SEP)

maps = build_all(res=0.1)

# ---------------------------------------------------------------- 통계표
print()
print("[맵 통계]")
hdr = (f"  {'맵':<17} {'크기':>10} {'셀수':>8} {'로버통과':>8} "
       f"{'3D비행':>7} {'4D전용':>7} {'벽':>6} {'최대h':>6}")
print(hdr)
print("  " + "-" * (len(hdr) - 2))
for name, hm in maps.items():
    s = hm.stats()
    print(f"  {name:<17} {s['size_m']:>10} {s['cells']:>8,} "
          f"{s['rover_ok_pct']:>7.1f}% {s['flyable_3d_pct']:>6.1f}% "
          f"{s['only_4d_pct']:>6.1f}% {s['wall_pct']:>5.1f}% {s['h_max']:>6.2f}")

# ---------------------------------------------------------------- R1
print()
print(SEP)
print("R1. 한 비행구간(5m) 안에 높이가 다른 장애물이 있는가")
print(SEP)
print("  기준: 넘어갈 수 있는 장애물 블록 쌍 중, 중심간 거리 <= 5m 이고")
print("        높이차 >= 0.3m 인 쌍이 존재하는가")
print()
r1_ok = True
for name, hm in maps.items():
    pairs, mdelta = flight_segment_height_variation(hm)
    heights = sorted(set(np.round(hm.grid[(hm.grid > ROVER_MAX) &
                                          (hm.grid <= H_4D_LIMIT)], 2).tolist()))
    ok = pairs > 0
    r1_ok &= ok
    hs = ", ".join(f"{h:.2f}" for h in heights) if heights else "(없음)"
    print(f"  {'PASS' if ok else 'FAIL'}  {name:<17} 조건만족 블록쌍 {pairs:>3}개"
          f"  최대높이차 {mdelta:.2f}m")
    print(f"        장애물 높이 종류: {hs}")
print(f"  => R1 {'충족' if r1_ok else '미충족'}")

# ---------------------------------------------------------------- R2
print()
print(SEP)
print(f"R2. 3D 불가 / 4D 가능 높이({H_3D_LIMIT}~{H_4D_LIMIT}m) 장애물 포함")
print(SEP)
r2_ok = True
for name, hm in maps.items():
    only4d = np.sum((hm.grid > H_3D_LIMIT) & (hm.grid <= H_4D_LIMIT))
    ok = only4d > 0
    # easy 맵은 의도적으로 없어도 됨 (난이도 구분)
    expected = not name.startswith("easy")
    verdict = "PASS" if ok == expected or ok else ("SKIP" if not expected else "FAIL")
    if expected:
        r2_ok &= ok
    print(f"  {verdict}  {name:<17} 4D전용 셀 {only4d:>6,}개"
          f"   {'(easy는 의도적으로 없음)' if not expected and not ok else ''}")
print(f"  => R2 {'충족 (medium/hard 전부 포함)' if r2_ok else '미충족'}")

# ---------------------------------------------------------------- R3 + C1/C2
print()
print(SEP)
print("R3/C1/C2. 도달 가능성 분석")
print(SEP)
print(f"  {'맵':<17} {'start유효':>9} {'goal유효':>9} {'지상만':>8} "
      f"{'3D하이브':>9} {'4D하이브':>9}")
print("  " + "-" * 66)

r3_ok = False
c2_ok = True
for name, hm in maps.items():
    sx, sy = hm.to_idx(*hm.start)
    gx, gy = hm.to_idx(*hm.goal)
    s_valid = hm.in_bounds(sx, sy) and hm.grid[sy, sx] <= ROVER_MAX
    g_valid = hm.in_bounds(gx, gy) and hm.grid[gy, gx] <= ROVER_MAX

    ground = reachable_ground(hm)
    hyb3 = reachable_hybrid(hm, H_3D_LIMIT)
    hyb4 = reachable_hybrid(hm, H_4D_LIMIT)

    g_ok = bool(ground[gy, gx]) if hm.in_bounds(gx, gy) else False
    h3_ok = bool(hyb3[gy, gx]) if hm.in_bounds(gx, gy) else False
    h4_ok = bool(hyb4[gy, gx]) if hm.in_bounds(gx, gy) else False

    if name == "hard_corridor":
        r3_ok = (not g_ok) and h4_ok
    c2_ok &= h4_ok

    print(f"  {name:<17} {str(s_valid):>9} {str(g_valid):>9} "
          f"{str(g_ok):>8} {str(h3_ok):>9} {str(h4_ok):>9}")

print()
print(f"  R3 (hard_corridor: 지상 실패 & 4D 성공): {'충족' if r3_ok else '미충족'}")
print(f"  C2 (모든 맵이 4D로 도달 가능)          : {'충족' if c2_ok else '미충족'}")

# ---------------------------------------------------------------- C3
print()
print(SEP)
print("C3. 탐색 규모 — A*가 감당할 수 있는 크기인가")
print(SEP)
print("  3D 상태공간 = nx * ny * 2 (ground/air)")
print("  4D 상태공간 = nx * ny * nz * 2,  nz = z해상도 단계 수")
print()
Z_MAX = 1.75
for zres in (0.05, 0.1, 0.25):
    nz = int(Z_MAX / zres) + 1
    print(f"  [z해상도 {zres}m -> nz={nz}]")
    for name, hm in maps.items():
        n3 = hm.nx * hm.ny * 2
        n4 = hm.nx * hm.ny * nz * 2
        print(f"     {name:<17} 3D {n3:>10,}   4D {n4:>13,}")
    print()

print(SEP)
print("요약")
print(SEP)
print(f"  R1 높이 변화 인접      : {'PASS' if r1_ok else 'FAIL'}")
print(f"  R2 4D전용 높이 포함    : {'PASS' if r2_ok else 'FAIL'}")
print(f"  R3 지상전용 실패 맵    : {'PASS' if r3_ok else 'FAIL'}")
print(f"  C2 4D 전부 도달가능    : {'PASS' if c2_ok else 'FAIL'}")
