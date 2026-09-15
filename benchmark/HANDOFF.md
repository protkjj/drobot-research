# A* vs RRT* 벤치마크 — 진행 상황

**목표 (2026-08-23 전환)**: "A* 채택 근거 확보".
원래는 "왜 RRT*인가"를 정당화하려 했으나, 측정 데이터가 A* 우위를 가리켜 kj가 방향 전환.

단, **결론에 맞춰 근거를 만들지 않는다.** RRT*를 오히려 최대한 강하게 구현해서
(Informed RRT*, k-nearest rewire, best-of-N seed) 공정 비교한다.
약한 RRT*를 이기면 "제대로 구현 안 한 것 아니냐"는 반박에 답할 수 없다.

**확정된 실험 설계**
- 상태공간: 3D `(x,y,m)` + 4D `(x,y,z,m)` 양쪽, 4D는 z해상도 sweep
- 실행: Python 독립 벤치마크 (ROS/Gazebo 없음)
- 파라미터 sweep: takeoff 에너지 0.1~5.0 Wh
- A*가 지는 조건도 함께 보고 (조건부 결론이 방어력이 높다)

---

## 완료된 단계

### 1단계 — 에너지 비용 모델 ✅ `cost/energy.py`
kj의 `energy_params.yaml`을 직접 읽는다. yaml에 없는 값만
`config/benchmark_assumptions.yaml`에 분리 (실측값과 내 가정을 구분하기 위함).

검증: `verify_step1.py`, `verify_step1b.py`

| 발견 | 내용 |
|---|---|
| ground_effect 죽은 파라미터 | `flight_clearance 0.8 > activation_height 0.5` → 3D에서 영구히 꺼짐 |
| 3D 고도 한계 | 3D는 h≤0.95m, 4D는 h≤1.55m만 통과 가능 (천장 2.5m 기준) |
| 4D 에너지 이득 | 0.5~0.7%로 미미 → 4D 정당화 근거로는 약함 |
| **4D의 진짜 근거** | **통과가능 높이 63% 확장 — "효율"이 아니라 "경로 존재 여부"** |

### 2단계 — 실험 맵 6종 ✅ `envs/heightmap.py`
{easy, medium, hard} × {open, corridor}, 20x14m ~ 24x16m.

**1차 설계 폐기**: 최적해가 6개 맵 전부에서 비행 0회를 선택.
원인은 `analyze_breakeven.py`로 규명 — 비행 손익분기 우회거리가 7.3~10.3m인데
장애물이 폭 1m라 우회거리가 2~3m뿐이었다. 2차는 '좁고 긴 벽(폭 0.6m)' 구조로 재설계.

### 3-4단계 — Dijkstra / A* ✅ `planners/grid_search.py`
`state_space.py`가 상태·전이·비용을 한 곳에 정의 → 세 플래너가 같은 문제를 푼다는 걸 보장.

검증 `verify_step34.py`: A*=Dijkstra 비용 일치, h=0이면 완전 동일, 비용 재계산 일치.

### 5단계 — Hybrid RRT* ✅ `planners/rrt_star.py`
Informed 샘플링 + k-nearest rewire + 공간 해싱.

### 6단계 — 동일 시간 예산 비교 ✅ `exp_time_budget.py`

**2.0초 예산(= kj config의 timeout)에서 5개 맵 전부 A* 우위**

아래는 **버그 5개를 전부 수정한 뒤의 최종 측정값**이다 (시드 20개).

| 맵 | A* | A*시간 | RRT*평균 | RRT*σ | A*우위 |
|---|---|---|---|---|---|
| easy_open | 44.276 | 0.12s | 44.278 | 0.250 | +0.00% |
| easy_corridor | 47.744 | 0.12s | 48.473 | 0.305 | +1.53% |
| medium_open | 72.088 | 0.81s | 74.158 | 0.552 | +2.87% |
| medium_corridor | 79.256 | 0.95s | 79.502 | 0.627 | +0.31% |
| hard_open | 79.407 | 1.00s | 81.671 | 0.726 | +2.85% |

**주의**: 이 우위는 8단계에서 조건부임이 밝혀진다.
현재 config(takeoff 5.0Wh)가 비행을 0회로 만들기 때문에 나온 결과다.

---

## 고친 버그 5개 (실험 결과보다 중요)

1. **A* corner-cutting** — 대각선 이동이 벽 모서리를 관통. 높이 1.10m 벽을 통과하는
   경로를 "최적해"로 내놓고 있었다. `_diag_ok_ground/_diag_ok_air`로 수정.
2. **불공정 비교** — A*만 격자 각도 제약(45° 배수)을 받고 RRT*는 연속 공간이라
   RRT*가 5.3% 앞서 보였다. `planners/smoothing.py`로 양쪽에 동일 스무딩 적용.
   **이걸 안 고쳤으면 결론이 정반대였다.**
3. **RRT* O(n²)** — rewire 반경에 `max(r, step_size)` 하한을 걸어 RRT*의 반경 축소
   성질을 파괴. k-nearest 방식(Karaman & Frazzoli 2011)으로 교체 → 40배 가속.
4. **RRT* 착륙 노드 미생성** — `_nearest()`가 같은 모드 노드만 우선 찾아,
   비행으로 목표 상공까지 가도 착륙 엣지가 안 생겼다(이륙 1, 착륙 0).
   하이브리드 문제 성공률 0%. AIR 노드 생성 시 착륙 노드를 함께 만들어 해결.
   **이걸 안 고쳤으면 "RRT*는 하이브리드를 못 푼다"는 틀린 주장을 했을 것.**
5. **상승각의 z해상도 종속** — 4D 고도 변경을 `diz ∈ {-1,0,1}`로 제한해서
   z=0.10에서 45°, z=0.05에서 26.6°가 됐다. 해상도를 높일수록 해가 나빠지는
   비단조 발생. `max_climb_angle_deg`로 물리 제약을 각도로 정의해 해결.
   RRT*/스무더에도 동일 제약 적용 (RRT*는 아예 제한이 없어 유리한 상태였음).

---

### 7단계 — 4D + z해상도 sweep ✅ `exp_4d_zres.py`, `exp_4d_fair.py`
양쪽 모두 2초 제약으로 재측정한 결과: **4D에서 A*는 easy_open 외 전부 timeout.**
가장 거친 z=0.5m에서도 3.75~4.97초. 4D가 필요하면 A*는 선택지가 아니다.

4D의 가치는 확인됨: hard_corridor는 3D로 아예 안 풀린다(통로 높이 1.10m > 3D 한계 0.95m).

### 8단계 — 파라미터 sweep ✅ `run_benchmark.py --only param`
**결론을 바꾼 실험.** takeoff을 낮춰 비행 유인을 키우면 RRT*가 역전한다.

| takeoff | A* 모드전환 | A* 우위 맵수 |
|---|---|---|
| 0.1~1.5 Wh | 1.33회 | 1~2/3 |
| 3.0 Wh | 0.67회 | 2/3 |
| 5.0 Wh (현재) | 0회 | 3/3 |

즉 "A* 5/5 승"은 현재 config가 비행을 배제한 결과였다.
원인 규명: A*의 이/착륙 지점이 격자(10cm)에 묶여서. 해상도를 5cm로 높이면
A*가 따라잡지만(medium_open -0.50%, hard_open +1.44%) 3.4~4.5초로 제약 위반.

### 9단계 — narrow passage ✅ `exp_narrow_passage.py`, `exp_mode_switch_passage.py`
**가설 기각.** 통로를 0.4m까지 좁혀도 RRT* 20/20 성공. 논거 4는 성립하지 않는다.
hard_corridor 실패는 통로 폭이 아니라 모드 전환 필요 때문이었고,
그마저 RRT* 착륙 버그(아래 5번)를 고치니 대부분 해소됐다.

### 10단계 — 연구일지 ✅
- `benchmark/RESEARCH_LOG.md` — 레포 보관용
- 웹 페이지: https://claude.ai/code/artifact/04e8cfc2-d027-478e-9476-f57e14c99c31
- 그림: `results/fig_summary.png`, `results/maps.png`

---

## 최종 결론

```
3D (현재 설계)        → A*   0.12~1.00초 최적해, 결정론적, 2초 제약 충족
비행 활발 조건        → RRT* 근소 우위(1~3%). A*는 해상도 높이면 따라잡지만 제약 위반
4D 확장               → RRT* A*는 전부 timeout
```

**A*를 택하는 진짜 이유는 해 품질(1~3%)이 아니다.** 에너지 파라미터가
72배 불확실한 상태라 3% 차이는 근거가 약하다. 실질적 이유는:
최적성 보장 / 결정론적 재현성(σ=0) / 시간 여유 2~16배 / 튜닝 부담 적음.

**실무 권고**: `hybrid_rrt_star_planner` → `hybrid_astar_planner` 이름 변경.
Nav2 SmacPlanner2D 구조 참고하되 하이브리드(모드 전환)는 직접 구현.
Phase 2에서 takeoff/landing 에너지를 최우선 실측.

---

## 다음 단계 (연구일지 이후)

1. 패키지/클래스 이름 변경 (`hybrid_astar_planner`)
2. `ElevationLayer` (2.5D costmap) C++ 구현 — 현재 헤더만 존재
3. `hybrid_astar_planner` C++ 구현 — `state_space.py`의 전이 정의를 옮기면 됨
4. Phase 2 INA226 실측 → `energy_params.yaml` 보정 → 이 벤치마크 재실행

---

## 실행 방법

```bash
cd /Users/kj/Desktop/dynamic/drobot-research
python3 benchmark/verify_step1.py      # 에너지 모델
python3 benchmark/verify_step1b.py     # 천장 제약
python3 benchmark/verify_step2.py      # 맵 요구사항
python3 benchmark/verify_step34.py     # Dijkstra/A*
python3 benchmark/verify_step5.py      # RRT* (느림 — 예산 축소 필요)
python3 benchmark/analyze_breakeven.py # 비행 손익분기
python3 benchmark/diagnose_noflight.py # 비행 미선택 원인
python3 benchmark/exp_time_budget.py   # 시간 예산 비교 (3D 핵심)
python3 benchmark/exp_4d_fair.py       # 4D 동일 제약 비교
python3 benchmark/run_benchmark.py --only param  # 파라미터 sweep -> JSON
python3 benchmark/exp_mode_switch_passage.py     # 모드 전환 통로
python3 benchmark/diagnose_4d_monotone.py        # 4D 비단조 진단
python3 benchmark/plot_maps.py         # 맵 시각화
python3 benchmark/plot_summary.py      # 종합 요약 그림
python3 benchmark/build_log_page.py    # 연구일지 HTML 생성
```

## 파일 구조

```
benchmark/
├── HANDOFF.md
├── plotstyle.py                # matplotlib 한글 폰트
├── config/benchmark_assumptions.yaml
├── cost/energy.py              # 비용 모델 (반드시 여기 한 곳에만)
├── envs/heightmap.py           # 맵 6종
├── planners/
│   ├── state_space.py          # 상태·전이·비용 정의 (3D/4D 공용)
│   ├── grid_search.py          # Dijkstra + A*
│   ├── rrt_star.py             # Hybrid Informed RRT*
│   └── smoothing.py            # 공정 비교용 shortcut 스무더
├── RESEARCH_LOG.md             # 연구일지 (레포 보관용)
├── research_log.html           # 연구일지 (발행본, base64 이미지 포함)
├── build_log_page.py           # 연구일지 HTML 생성기
├── run_benchmark.py            # 통합 러너 (결과를 JSON으로 저장)
├── plot_summary.py             # 종합 요약 그림
└── results/
    ├── maps.png
    ├── fig_summary.png
    └── benchmark_param_sweep.json
```

## 주의사항

- **비용 계산은 `cost/energy.py` 한 곳에만.** 플래너마다 다른 비용 함수를 쓰면 비교가 무의미.
- **스무딩은 반드시 양쪽에 동일 적용.** 한쪽만 하면 그게 또 불공정.
- kj의 `src/` config는 수정하지 않는다. 벤치마크 전용 값은 `benchmark/config/`에만.
- 결과가 극단적이면 보고 전에 먼저 버그를 의심할 것. 실제로 5개 나왔다.
  (RRT* 0% 성공률이 버그 4를 잡는 단서였다)
- `verify_step5.py`는 예산 4만 샘플이라 매우 느리다. 실행 전 예산을 줄일 것.
