"""4D z해상도 비단조 문제 진단.

관측: medium_corridor 에서
    z=0.50 -> 79.986
    z=0.25 -> 79.886
    z=0.20 -> 78.686
    z=0.10 -> 78.326
    z=0.05 -> 79.686   <- 더 촘촘한데 더 나쁘다

z 격자가 촘촘해지면 선택지가 늘어나므로 해가 나빠질 수 없다.
(z=0.10의 격자점은 z=0.05 격자점의 부분집합이다: 0.1k = 0.05*(2k))
따라서 이건 이산화 오차가 아니라 구현 버그일 가능성이 크다.

진단
  M1. z=0.05 격자가 정말 z=0.10 격자를 포함하는가 (수치 확인)
  M2. z=0.10의 최적 경로를 z=0.05 상태로 옮겨도 유효한가
      -> 유효한데 z=0.05 A*가 더 비싼 해를 냈다면 탐색 쪽 문제
  M3. A*가 timeout 없이 완주했는가 (부분해 반환 의심)
  M4. 스무딩 전/후 어느 쪽에서 역전이 생기는가
"""
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from cost.energy import EnergyModel  # noqa: E402
from envs.heightmap import build_all  # noqa: E402
from planners.state_space import ProblemSpec, GROUND, AIR  # noqa: E402
from planners.grid_search import astar  # noqa: E402
from planners.smoothing import smooth_result, grid_path_to_xyz, path_cost  # noqa: E402

SEP = "=" * 80
model = EnergyModel.from_yaml()
maps = build_all(res=0.1)
NAME = "medium_corridor"
hm = maps[NAME]

Z_LIST = [0.5, 0.25, 0.2, 0.1, 0.05]


# ---------------------------------------------------------------- M1
print(SEP)
print("M1. z 격자 포함관계 확인")
print(SEP)
spec_coarse = ProblemSpec(hm=hm, model=model, dims=4, z_res=0.1)
spec_fine = ProblemSpec(hm=hm, model=model, dims=4, z_res=0.05)
print(f"  z=0.10: nz={spec_coarse.nz}, 격자점 = "
      f"{[round(spec_coarse.z_of(i),3) for i in range(min(6, spec_coarse.nz))]} ...")
print(f"  z=0.05: nz={spec_fine.nz}, 격자점 = "
      f"{[round(spec_fine.z_of(i),3) for i in range(min(11, spec_fine.nz))]} ...")

coarse_pts = {round(spec_coarse.z_of(i), 6) for i in range(spec_coarse.nz)}
fine_pts = {round(spec_fine.z_of(i), 6) for i in range(spec_fine.nz)}
contained = coarse_pts <= fine_pts
print(f"  z=0.10 격자점이 전부 z=0.05 격자에 포함되는가: {contained}")
if not contained:
    print(f"     누락: {sorted(coarse_pts - fine_pts)[:10]}")
print(f"  z_max: coarse={spec_coarse.z_max:.4f} fine={spec_fine.z_max:.4f}")


# ---------------------------------------------------------------- M3
print()
print(SEP)
print("M3. A*가 완주했는가 / 스무딩 전후 비용")
print(SEP)
print(f"  {'z해상도':>8} {'상태수':>11} {'스무딩전':>10} {'스무딩후':>10} "
      f"{'시간':>8} {'완주':>6} {'확장':>10}")
print("  " + "-" * 70)

raw_results = {}
for zr in Z_LIST:
    spec = ProblemSpec(hm=hm, model=model, dims=4, z_res=zr)
    t0 = time.perf_counter()
    r = astar(spec, timeout=600.0)
    dt = time.perf_counter() - t0
    if not r.found:
        print(f"  {zr:>7.2f}m {spec.summary()['n_states']:>11,} "
              f"{'실패':>10} {'-':>10} {dt:>7.1f}s "
              f"{'timeout' if r.timed_out else 'no-sol':>6}")
        continue
    c_sm, acc_sm, path_sm = smooth_result(spec, r, is_grid=True)
    raw_results[zr] = {"spec": spec, "res": r, "raw_cost": r.cost,
                       "sm_cost": c_sm, "sm_path": path_sm}
    print(f"  {zr:>7.2f}m {spec.summary()['n_states']:>11,} "
          f"{r.cost:>10.4f} {c_sm:>10.4f} {dt:>7.1f}s "
          f"{'OK':>6} {r.n_expanded:>10,}")

print()
print("  스무딩 '전' 비용이 단조 감소하는지 확인:")
zs = sorted(raw_results, reverse=True)
prev = None
mono_raw = True
for zr in zs:
    c = raw_results[zr]["raw_cost"]
    flag = ""
    if prev is not None and c > prev + 1e-9:
        flag = "  <-- 역전!"
        mono_raw = False
    print(f"     z={zr:<5} raw={c:.4f}{flag}")
    prev = min(prev, c) if prev is not None else c
print(f"  => 스무딩 전 단조: {mono_raw}")

print()
print("  스무딩 '후' 비용:")
prev = None
mono_sm = True
for zr in zs:
    c = raw_results[zr]["sm_cost"]
    flag = ""
    if prev is not None and c > prev + 1e-9:
        flag = "  <-- 역전!"
        mono_sm = False
    print(f"     z={zr:<5} sm ={c:.4f}{flag}")
    prev = min(prev, c) if prev is not None else c
print(f"  => 스무딩 후 단조: {mono_sm}")


# ---------------------------------------------------------------- M2
print()
print(SEP)
print("M2. 성긴 격자의 최적 경로를 촘촘한 격자로 옮기면 유효한가")
print(SEP)
print("  z=0.10 의 A* 경로를 z=0.05 상태공간의 상태로 변환해서")
print("  (a) 모든 전이가 유효한지, (b) 비용이 같은지 확인한다.")
print("  유효하고 비용이 같은데 z=0.05 A*가 더 비싼 해를 냈다면 탐색 문제다.")
print()

if 0.1 in raw_results and 0.05 in raw_results:
    sc = raw_results[0.1]["spec"]
    sf = raw_results[0.05]["spec"]
    path_c = raw_results[0.1]["res"].path

    # z=0.10 경로의 상태를 z=0.05 인덱스로 변환
    converted = []
    ok_convert = True
    for s in path_c:
        ix, iy, iz, mode = s
        z_real = sc.z_of(iz)
        iz_f = sf.iz_of(z_real)
        if abs(sf.z_of(iz_f) - z_real) > 1e-9:
            ok_convert = False
            print(f"     변환 실패: z={z_real} 이 z=0.05 격자에 없음")
            break
        converted.append((ix, iy, iz_f, mode))

    print(f"  경로 변환 성공: {ok_convert} ({len(converted)}개 상태)")

    if ok_convert:
        # 변환된 경로가 z=0.05 상태공간에서 유효한 전이인지
        try:
            acc = sf.path_cost(converted)
            c_conv = sf.model.cost(acc)
            print(f"  변환 경로의 z=0.05 비용 : {c_conv:.4f}")
            print(f"  원래 z=0.10 비용        : {raw_results[0.1]['raw_cost']:.4f}")
            print(f"  z=0.05 A*가 찾은 비용   : {raw_results[0.05]['raw_cost']:.4f}")
            print()
            if c_conv < raw_results[0.05]["raw_cost"] - 1e-6:
                print("  !!! z=0.05 A*가 더 나쁜 해를 냈다 !!!")
                print("      같은 경로가 그 상태공간에 존재하는데 못 찾았다는 뜻.")
                print("      -> A* 탐색 또는 상태공간 정의에 버그.")
            else:
                print("  z=0.05 A*의 해가 변환 경로보다 같거나 좋다. 탐색은 정상.")
        except ValueError as e:
            print(f"  !!! 변환 경로가 z=0.05 상태공간에서 무효 !!!")
            print(f"      {e}")
            print("      -> 4D가 z해상도에 따라 '상위집합'이 아니라는 뜻.")
            print("         상태공간 정의(이웃 생성 규칙)를 봐야 한다.")


# ---------------------------------------------------------------- M4
print()
print(SEP)
print("M4. 원인 후보 점검")
print(SEP)
print("  4D 이웃 생성에서 고도 변경은 diz in (-1, 0, +1) 로 한 칸씩만 허용한다.")
print("  z해상도가 촘촘해지면 같은 고도차를 내는 데 더 많은 '스텝'이 필요하다.")
print("  비용은 dz에 비례하므로 합은 같아야 하지만,")
print("  '수평이동 없는 제자리 고도변경'이 각 스텝마다 별도 상태를 만든다.")
print()
for zr in (0.1, 0.05):
    if zr not in raw_results:
        continue
    r = raw_results[zr]["res"]
    p = r.path
    n_vert = sum(1 for a, b in zip(p, p[1:])
                 if a[0] == b[0] and a[1] == b[1] and a[3] == b[3] == AIR)
    n_switch = sum(1 for a, b in zip(p, p[1:]) if a[3] != b[3])
    zvals = sorted({raw_results[zr]["spec"].z_of(s[2]) for s in p if s[3] == AIR})
    print(f"  z={zr}: 경로 길이 {len(p)}, 제자리 고도변경 {n_vert}회, "
          f"모드전환 {n_switch}회")
    print(f"        사용한 고도: {[round(v,3) for v in zvals]}")
