# cost↔높이 계약은 호스트 costmap 설정에 얼마나 의존하는가

**2026-09-22** · 측정 코드 `test/test_costmap_terrain_contract.cpp` (실행하면 아래 표가 그대로 찍힌다)
· 설계는 바꾸지 않았다 — 측정과 권고까지다.

> **2026-10-02 갱신 — 권고 (b-0) 를 구현했다.**
> `LayerTerrainSource` (state_space.hpp) 가 등급은 `terrain_layer`(기본 `elevation_layer`)
> 자체 격자에서, 충돌(253·254)은 master 에서 읽는다. 레이어가 없으면 아래의 (a) 로
> 떨어지며 경고한다. 미관측(255)은 주행은 평지로 보되, 착륙은 관측된 칸에만 한다
> (`land_only_on_observed`). 회귀 테스트는 `test_state_space.cpp` 의 `LayerTerrainTest`.
> 이 문서의 표와 `test_costmap_terrain_contract.cpp` 는 (a) 를 잰 기록이라 그대로 둔다.
>
> 이 문서에서 바로잡을 것 두 가지 (코드 대조)
> - "ElevationLayer 는 '더 큰 값만' 쓴다 (max 결합)" — 실제 `elevation_layer.cpp`
>   updateCosts 는 처음(2026-08-25)부터 `setCost` 로 **덮어쓴다**. 그래서 3절 (d) 와 달리,
>   관측된 칸에서는 obstacle_layer 의 254 가 ElevationLayer 등급으로 바뀐다.
>   이 문서의 측정은 max 결합 레이어로 흉내 낸 것이다.
> - "LiDAR 평면은 지면에서 0.55 m" — URDF `base_footprint` 높이 오류(+0.25 m)가
>   든 TF 값이었다. 실제는 0.30 m (HANDOFF.md 2절, 2026-10-02 수정).

---

## 한 줄 결론

지금의 (a) cost 역추론은 드롭인 배포에 쓰기 어렵다. 플래너가 읽는 '지형'이 호스트의
inflation 두 값, 장애물 소스, `track_unknown_space` 에 따라 바뀐다. 호스트와 무관하게도
fly_over 등급이 실제 높이를 최대 0.6 m 가린다.
**(b) 로 가길 권한다. 높이·주행 등급은 ElevationLayer 에서 직접 읽고, master cost 는
충돌(253·254) 판정에만 쓴다.** ElevationLayer 와 플래너가 같은 planner_server 프로세스에
있으므로, 토픽보다 레이어를 직접 조회하는 쪽이 작고 안전하다.

---

## 1. 무엇을 어떻게 쟀나

- 실제 `nav2_costmap_2d::InflationLayer` (Jazzy 1.3.11) 를 `LayeredCostmap` 에 붙였다.
  레이어 순서는 global_costmap 과 같다: `[obstacle_layer, elevation_layer, inflation_layer]`.
- ElevationLayer 자리에는 결합 규칙('더 큰 값만 덮어쓴다', `elevation_layer.cpp`
  updateCosts)과 cost 값(0/100/200/254)이 같은 고정 격자 레이어를 뒀다.
  센서 콜백 없이 등급을 직접 넣기 위해서다.
- 읽기는 플래너가 쓰는 클래스 그대로다: `CostmapTerrainSource`, `ProblemSpec`.
- footprint 0.45 × 0.45 m (내접 0.225 m), 해상도 0.05 m — `nav2_params_hybrid.yaml` 의 global_costmap.
- 거리는 Nav2 관례대로 벽 칸 중심 ~ 칸 중심이다. 벽 표면까지는 0.025 m 짧다.
- **손계산 검증**: 네 설정 모두에서 레이어가 실제로 쓴 cost 가 `252·exp(−k·(d − r))`
  (내접 안 253, 반경 밖 0) 식과 **칸 단위로 같았다**. 테스트가 이를 단언한다.

## 2. (a)(b)(c) — 호스트 inflation 설정별

| inflation_radius / cost_scaling_factor | (a) 주행불가 띠 fly_over(0.60 m) | (a) 가짜 턱 띠 rover(0.15 m) | (a) h>0 띠 전체 | (b) 0.15 m 턱 오분류 | (c) 목표 거부 거리 | 순정 플래너 거부(253 이상) | 로버 통과 최소 통로 폭 | 순정 플래너 최소 통로 폭 |
|---|---|---|---|---|---|---|---|---|
| 0.55 / 3.0 | 0.35 m (7칸) | 0.40 ~ 0.55 m (4칸) | 0.55 m | 0.35 m 이내 주행불가 오독, 0.40 ~ 0.55 m 높이차 소실 | 0.35 m 이내 | 0.20 m 이내 | 0.75 m | 0.45 m |
| 0.70 / 3.0 (Jazzy nav2_bringup 기본) | 0.35 m (7칸) | 0.40 ~ 0.70 m (7칸) | 0.70 m | 0.35 m 이내 주행불가 오독, 0.40 ~ 0.70 m 높이차 소실 | 0.35 m 이내 | 0.20 m 이내 | 0.75 m | 0.45 m |
| 1.00 / 2.0 (이 저장소) | 0.45 m (9칸) | 0.50 ~ 1.00 m (11칸) | 1.00 m | 0.45 m 이내 주행불가 오독, 0.50 ~ 1.00 m 높이차 소실 | 0.45 m 이내 | 0.20 m 이내 | 0.95 m | 0.45 m |
| 0.55 / 10.0 (InflationLayer 코드 기본) | 0.25 m (5칸) | 0.30 ~ 0.35 m (2칸) | 0.35 m | 0.25 m 이내 주행불가 오독, 0.30 ~ 0.35 m 높이차 소실 | 0.25 m 이내 | 0.20 m 이내 | 0.55 m | 0.45 m |

마지막 행은 파라미터를 하나도 안 줬을 때 InflationLayer 가 쓰는 값이다 (테스트가 확인).
Jazzy `nav2_bringup/params/nav2_params.yaml` 은 local·global 모두 0.70 / 3.0 이다 (이 머신에서 확인).

**읽는 법.** 벽 옆 빈 바닥 중 '주행불가 띠'는 0.60 m 장애물로, 그 바깥 '가짜 턱 띠'는
0.15 m 턱으로 읽힌다. hybrid·rover_detour 에서 로버는 주행불가 띠에 들어가지 못한다.
목표점이 그 안에 있으면 `createPlan` 이 거부하고("목표점이 로버가 설 수 없는 셀이다"),
시작점도 같다. 이 저장소 설정에서는 로봇이 벽에서 0.45 m 안에 서 있으면 계획 자체가
안 된다. 로봇 반폭은 0.225 m 라 물리적으로는 그보다 훨씬 붙을 수 있다.

**호스트 의존도.** 같은 로봇·같은 맵인데 호스트의 두 숫자만으로 값이 이만큼 바뀐다.
- h>0 띠: 0.35 → 1.00 m (2.9배)
- 주행불가 띠: 0.25 → 0.45 m (1.8배)
- 로버 최소 통로: 0.55 → 0.95 m

순정 플래너(NavFn/Smac)는 네 설정 모두에서 0.20 m / 0.45 m 로 같다. 순정은 inflation 을
'높이'가 아니라 '충돌 여유'로만 쓰고, 253 이상만 막힌 칸으로 보기 때문이다.

**띠 폭 = inflation_radius.** 네 설정 중 셋에서 가짜 턱 띠가 inflation 반경 끝까지 간다.
반경 끝 칸의 cost 가 아직 50 을 넘기 때문이다 (이 저장소 1.00 m 끝 칸 = 53).
즉 호스트가 정한 반경이 곧 오염 폭이다.

**과제 손계산과 비교** (1.00 / 2.0, Δ = d − 0.225)
- 'Δ < 0.26 m 주행불가' — 맞다. 칸 단위로는 Δ ≤ 0.225, 곧 d ≤ 0.45 m 다.
- '0.26 ≤ Δ < 0.81 m 가짜 턱' — 위쪽 경계가 cost 50 이 아니라 inflation 반경에서 끊긴다
  (Δ ≤ 0.775, d ≤ 1.00 m).
- 'Δ ≥ 0.81 m 정상'은 'd > 1.00 m 정상'이 맞다.

**(b) 턱의 오분류.** rover 등급(100) 칸의 cost 는 max(100, inflation) 이 된다.
- 주행불가 띠 안에서는 0.60 m 로 읽혀 턱이 막힌다.
- 그 바깥 가짜 턱 띠에서는 턱과 바닥이 둘 다 0.15 m 로 읽혀 턱의 높이차가 사라진다.
  rover_climb 이 이 턱을 오를 때 내야 할 등반 비용이 없어진다.

여기서 '턱'은 ElevationLayer 가 rover 등급(100)으로 쓰는 지형이다. 코드상(실행 검증은
안 했다) 모서리가 날카로운 0.15 m 턱은 단차 기준(`max_step_height` 0.05 m)에 걸려
가장자리가 fly_over(200)가 된다. 따라서 rover 등급은 0.15 m 이하의 완만한 경사나 거친
바닥에서 나온다.

**팬텀 등반 비용 (과제의 0.3 Wh).** rover_climb 에서만 붙는다. hybrid·detour 에서는
`groundEdge` 가 평지 비용만 쓴다. rover_climb 에서 벽으로 다가가면 두 번 붙는다.
- 가짜 턱 띠에 들어설 때: 0.15 × 2.0 = 0.3 Wh
- 주행불가 띠에 들어설 때: 0.45 × 2.0 = 0.9 Wh 추가. rover_climb 에서는 0.60 ≤ 0.70 이라
  이 띠를 밟고 넘을 수 있다.

## 3. inflation 밖의 오염 경로

| | 원인 (호스트 쪽) | 플래너가 읽는 것 | 테스트 |
|---|---|---|---|
| (d) 장애물 소스가 등급을 가림 | obstacle_layer / static_layer 가 254 를 찍는다. ElevationLayer 는 '더 큰 값만' 쓰므로 254 를 못 낮춘다 | 날아 넘을 수 있는 상자(200)가 99 m 벽이 되고, 주변에 inflation 띠까지 생긴다. 이 저장소 LiDAR 평면은 지면에서 0.55 m 라 0.55~1.2 m 장애물이 전부 이렇게 된다 | `HostObstacleLayerMasksFlyOverGrade` |
| (e) 벽 하나로 생기는 가짜 지형 | inflation | 평지뿐인 맵에서 비행 고도 후보가 {0.80} 에서 {0.80, 0.95, 1.40} 으로 는다. rover_climb 은 내접 구간(253, footprint 가 벽과 겹치는 곳)을 0.60 m 장애물로 읽어 **주행 가능**으로 판정한다 | `WallAloneCreatesPhantomTerrain` |
| (f) 미탐색 칸의 의미 | `track_unknown_space` | false(이 저장소)면 평지 0.00 m. true(Jazzy nav2_bringup 기본)면 99 m 벽이라 주행도 비행도 안 된다. 순정 플래너는 `allow_unknown` 으로 지나가는 곳이다 | `UnknownSpaceMeaningFollowsTrackUnknownSpace` |

`nav2_params_hybrid.yaml` 주석은 "elevation_layer 는 obstacle_layer 뒤에 와야 한다 —
2.5D 높이 판정이 2D 장애물 판정을 덮어쓸 수 있어야" 라고 적혀 있다. 하지만 결합 규칙이
max 라서 덮어쓰지 못한다. 순서를 바꿔도 같다 — inflation 도 max 로 쓴다.

(e) 의 비행 쪽 부수효과(계산): 가짜 0.60 m 띠 위를 z = 0.80 m 로 날면 clearance 가 0.20 m 다.
activation_height(0.5) 이하라 ground effect ×1.15 가 붙어, 벽 옆 비행이 15% 비싸진다.

## 4. 호스트와 무관한 한계 — 대표 높이가 실제 높이를 가린다

fly_over 등급 하나가 0.15 < h ≤ 1.2 m (fly_over_max) 를 전부 덮는데, 역추론은 이를
대표값 0.60 m 하나로 읽는다. 그래서 가장 낮은 비행 고도 0.80 m 로도 '넘을 수 있다'고
판정한다 (`FlyOverGradeHidesTrueHeight`).

| 실제 높이 (m) | 0.20 | 0.60 | 1.00 | 1.20 |
|---|---:|---:|---:|---:|
| z = 0.80 에서 여유 (요구 0.20 m) | +0.60 | +0.20 | **−0.20** | **−0.40** |

0.6 m 를 넘는 fly_over 장애물 위로는 충돌 고도로 계획이 나온다.
아래 medium_open 결과가 실제로 그랬다.

## 5. 실제 맵에서 — 호스트 설정만 바꿔 같은 문제를 푼다

`BenchmarkMapDecisionsPerHostConfig`.
- 맵 칸 배치는 Python 원본과 같다 (`scripts/test_maps.py` build_base_map,
  `benchmark/envs/heightmap.py` make_medium_open).
- 탐색은 플래너와 같은 상태공간·비용·휴리스틱의 A* 다 (스무딩 없음).
- '순수' 열은 등급만이다. 나머지 열은 호스트 inflation + LiDAR (0.55 m 보다 높은 장애물은 254).
- **충돌 고도**: 비행 고도가 비행 구간 아래 실제 높이보다 낮다는 표시다.

| 맵 | 에너지 세트 | 순수(등급만) | 0.55 / 3.0 + LiDAR | 0.70 / 3.0 (Jazzy nav2_bringup 기본) + LiDAR | 1.00 / 2.0 (이 저장소) + LiDAR | 0.55 / 10.0 (InflationLayer 코드 기본) + LiDAR |
|---|---|---|---|---|---|---|
| base_map_h0.5 | default | 전환 0 · 5.33 Wh · 비행 0.0 m | 전환 0 · 5.33 Wh · 비행 0.0 m | 전환 0 · 5.33 Wh · 비행 0.0 m | 전환 0 · 5.33 Wh · 비행 0.0 m | 전환 0 · 5.33 Wh · 비행 0.0 m |
| base_map_h0.5 | derived | 전환 0 · 5.33 Wh · 비행 0.0 m | 전환 0 · 5.33 Wh · 비행 0.0 m | 전환 0 · 5.33 Wh · 비행 0.0 m | 전환 0 · 5.33 Wh · 비행 0.0 m | 전환 0 · 5.33 Wh · 비행 0.0 m |
| base_map_h1.0 | default | 전환 0 · 5.33 Wh · 비행 0.0 m | 전환 0 · 5.47 Wh · 비행 0.0 m | 전환 0 · 5.47 Wh · 비행 0.0 m | 전환 0 · 5.51 Wh · 비행 0.0 m | 전환 0 · 5.43 Wh · 비행 0.0 m |
| base_map_h1.0 | derived | 전환 0 · 5.33 Wh · 비행 0.0 m | 전환 0 · 5.47 Wh · 비행 0.0 m | 전환 0 · 5.47 Wh · 비행 0.0 m | 전환 0 · 5.51 Wh · 비행 0.0 m | 전환 0 · 5.43 Wh · 비행 0.0 m |
| medium_open | default | 전환 0 · 17.56 Wh · 비행 0.0 m | 전환 0 · 18.49 Wh · 비행 0.0 m | 전환 0 · 18.49 Wh · 비행 0.0 m | 전환 0 · 18.69 Wh · 비행 0.0 m | 전환 0 · 18.17 Wh · 비행 0.0 m |
| medium_open | derived | 전환 2 · 12.96 Wh · 비행 18.5 m · **충돌 고도** | 전환 2 · 18.34 Wh · 비행 27.6 m | 전환 2 · 18.34 Wh · 비행 27.6 m | 전환 2 · 18.39 Wh · 비행 27.6 m | 전환 2 · 18.29 Wh · 비행 27.6 m |

- **base_map** (상자 옆 통로 1.9 m): 모든 설정에서 결정이 같다 (전환 0).
  1.0 m 상자가 LiDAR 에 가려지면 우회가 길어져 에너지가 +1.9~3.4% 늘 뿐이다.
- **medium_open, default**: 모든 설정에서 전환 0 이다 (Python 벤치마크와 같다).
  호스트 레이어가 벽 B·C 를 254 로 만들고 띠를 두르면서 우회가 길어져 +3.5~6.4% 다.
- **medium_open, derived — 순수 계약**: 1.10 m 벽 B 를 0.60 m 로 읽고 z = 0.80 m 로
  넘는 **충돌 고도** 계획이 나온다 (4절).
- **medium_open, derived — 호스트 LiDAR 가림**: B·C 가 가려져 전혀 다른 경로가 된다
  (비행 27.6 m). 이번에는 가림이 우연히 충돌을 막았다. 하지만 같은 가림이 0.80 m 벽 C 도
  막아 정상적인 비행 선택지까지 없앤다.
- **이 두 맵에서는 inflation 설정 넷 사이에 결정 차이가 없었다** (통로가 모두 1.7 m 이상).
  결정이 뒤집히는 조건은 2절의 최소 통로 폭이다. 0.55~0.95 m 보다 좁은 통로는 호스트
  설정에 따라 로버가 지나가기도 하고, 못 지나가 비행·우회를 강요받기도 한다.

**참고 — derived 세트의 성질.** derived 에서는 평지 1 m 비용이 비행 0.78, 주행 0.80 으로
비행이 오히려 싸다 (시간 가중 포함, clearance > 0.5 m). 그래서 한번 떠오르면 장애물이
없는 구간까지 길게 난다. 위 표의 비행 27.6 m 가 그 예다.
`energy_model` 파라미터 전달 버그를 고쳤으니, 이제 derived 시뮬레이션은 지금까지의
기록과 질적으로 다른 경로를 낼 것이다.

**참고 — 미확인.** 시뮬레이션 기록 `sim_medium_open_derived.json` (실제로는 default 값으로
계획됨) 에는 장애물이 없는 x ≈ 2 m, y 7.8 → 12.2 m 구간을 나는 전환 2회가 있다.
같은 맵을 위 다섯 설정 어느 것으로 계획해도 default 에서는 전환이 0 이다. 따라서
시뮬레이션 costmap 에 벤치마크 맵에 없는 비주행 칸이 있었다는 뜻이다. costmap 기록이
없어 원인(지각 쪽 분류, 미관측 칸 등)은 확인하지 못했다. 다음 시뮬레이션에서
`/global_costmap/costmap` 을 함께 기록하면 가릴 수 있다.

## 6. 권고 — (a) / (b) / (c)

판단 기준은 '얼마나 부정확한가'가 아니라 '호스트 설정에 따라 얼마나 달라지는가'다.

| | 호스트 의존 | 4절의 높이 손실 | 변경 범위 |
|---|---|---|---|
| (a) 지금 그대로 | inflation 두 값, 장애물 소스, `track_unknown_space`, footprint 에 모두 의존 (2·3절) | 있음 | 0 |
| (b) 높이·등급은 별도 채널, master 는 충돌만 | 충돌 여유(253·254)만 호스트 몫으로 남는다 — 순정 플래너와 같은 의미 | 없음 (실제 높이) | 아래 표 |
| (c) grid_map | (b)와 같다 | 없음 | (b)-토픽 + 의존성 |

**권고: (b).** 원칙은 하나다.
**지형(높이·주행 등급)은 ElevationLayer 에서 직접 읽고, master cost 는 충돌(253·254) 판정에만 쓴다.**
그러면 호스트의 inflation 은 Nav2 가 의도한 '안전 여유'로만 작동하고, 높이로 새지 않는다.
255(미탐색)는 충돌로 치지 않는다. 순정 플래너의 `allow_unknown` 처럼 따로 다뤄야 (f) 가 되살아나지 않는다.

(a) 안에서 고치는 방법은 모두 다시 호스트 설정에 묶인다.
- 레이어 순서 바꾸기: max 결합이라 효과가 없다.
- inflation 을 역계산해 빼기: 호스트의 반경·기울기·footprint 를 알아야 하고,
  lethal 가림(d)은 되돌릴 수 없다.
- 호스트에 특정 설정을 요구하기: 드롭인이라는 목표와 맞지 않는다.

(b) 는 토픽보다 **같은 프로세스 안에서 레이어를 직접 조회**하는 쪽이 낫다.
ElevationLayer 는 planner_server 가 소유한 global_costmap 의 플러그인이다. 그래서 플래너가
이미 받는 `costmap_ros->getLayeredCostmap()->getPlugins()` 에서 찾을 수 있다
(Nav2 `InflationLayer::getInflationLayer()` 와 같은 방식).
- 메시지 타입, 지연, 동기화 문제가 없다.
- 플래너가 계획 중에 잡는 master 잠금이 레이어 갱신(updateMap)도 막는다.
- 레이어를 못 찾으면 지금의 `CostmapTerrainSource` 로 떨어지면서 경고하면 된다.

변경 범위 (어림). TerrainSource 인터페이스는 이미 분리돼 있다.

| 단계 | 바뀌는 곳 | 규모 | 효과 |
|---|---|---|---|
| (b-0) 등급을 레이어 자체 격자에서 읽기 | `hybrid_astar_planner.cpp` configure: ElevationLayer 를 찾아 `CostmapTerrainSource` 에 master 대신 레이어의 Costmap2D 를 넘긴다 (ElevationLayer 는 CostmapLayer 라 자체 격자에 등급을 적어 둔다). `ProblemSpec::groundOk` 에 master cost 253·254 충돌 검사를 추가한다. 비행 쪽은 master 254 를 계속 통과 불가로 둬야 한다 — 안 그러면 LiDAR 가 가려 주던 0.6~1.2 m 장애물 위로 충돌 고도 계획이 다시 나온다 (4절). package.xml/CMake 에 drobot_costmap_2_5d 의존 추가 | 50~100 줄 | 2절과 (e)(f) 오염 제거. (d)와 4절은 남는다 |
| (b-1) 실제 높이까지 | ElevationLayer 에 잠금을 거는 공개 조회 함수 (셀 높이, 관측 여부) 추가. 새 `TerrainSource` 구현: 비행은 레이어가 관측한 실제 높이로 판정하고, master 의 253·254 는 지상 충돌에만 쓴다. 레이어가 못 본 칸인데 master 가 lethal 이면 높이를 모르므로 통과 불가로 둔다. `buildAirLevels` 양자화 — 연속 높이면 칸마다 고도 후보가 생겨 상태공간이 커지고, `level` 이 uint8 이라 255 개에서 잘린다 | +150~250 줄, 테스트 포함 1~2일 | 4절과 (d)까지 제거 |
| (b-토픽) 생산자가 다른 프로세스일 때만 | drobot_msgs 에 높이맵 메시지, 레이어 퍼블리셔, 구독·격자 정렬·지연(stale) 처리 | 350~500 줄 | (b-1) 과 같지만 동기화 문제가 새로 생긴다 |
| (c) grid_map | (b-토픽) 의 메시지를 `grid_map_msgs/GridMap` 으로 바꾼다. 두 패키지에 grid_map_core/ros/msgs 의존 추가 (이 머신 Jazzy 에 2.2.2 가 설치돼 있다) | 400~600 줄 | 다층 맵·보간·RViz 플러그인. elevation_mapping 같은 외부 스택과 호환된다 |

(c) 는 외부 elevation mapping 스택으로 바꾸거나 다층 지도가 필요해질 때 `TerrainSource`
구현 하나로 추가하면 된다. 지금의 드롭인 목표에는 (b-0) → (b-1) 순서가 가장 작은 변경으로
호스트 의존을 없앤다.

## 7. 다시 재기

```bash
colcon build --packages-select drobot_msgs drobot_hybrid_planner
colcon test --packages-select drobot_hybrid_planner \
  --ctest-args -R test_costmap_terrain_contract --event-handlers console_direct+
# 표만 보려면 테스트 바이너리를 직접 실행한다
./build/drobot_hybrid_planner/test_costmap_terrain_contract
```

호스트 설정을 더 넣으려면 테스트의 `kHosts` 에 한 줄을 추가하면 된다. 두 표에 행과 열이 함께 생긴다.
