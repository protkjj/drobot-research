# drobot-research 진행 상황

**최종 갱신 2026-08-25**

---

## 지금 상태 한 줄 요약

벤치마크로 **A\* 채택 근거를 확보**하고, C++ 구현을 마쳤으며,
**Gazebo 시뮬레이션에서 2.5D 하이브리드 경로계획이 끝까지 동작함을 확인**했다.
8/24 회의 이후 연구 방향이 **3 modal 에너지 비교**로 바뀌어 그에 맞게 재구현했고,
C++ 이 장애물을 날아 넘지 못하던 결함까지 수정해 원격 빌드·테스트를 통과했다.

---

## 0. 2026-08-25 작업 (3 modal 재정의 + 비용함수 수정)

### 0.1 3 modal 확정

장애물을 만났을 때 '어떻게 넘어가느냐'로 나눈다.

| modal | 우회 | 밟고넘기 | 비행 | 로버 통과높이 |
|---|---|---|---|---|
| `rover_detour` | O | X | X | 0.15 m |
| `rover_climb` | O | O | X | **0.70 m** |
| `hybrid` | O | X | O | 0.15 m |

`rover_climb`, `hybrid` 는 각각 `rover_detour` 의 상위집합이다 —
밟거나 날 수 있는 로봇은 그냥 도는 경로도 갈 수 있다.
이 성질이 결과 검증의 불변식으로 쓰인다.

파일: `benchmark/planners/state_space.py` (`modal` 필드),
`src/drobot_hybrid_planner/include/drobot_hybrid_planner/state_space.hpp` (`enum class Modal`),
`hybrid_astar_params.yaml` 의 `modal` 파라미터로 노출.

### 0.2 비용함수 수정 — 전환 에너지가 40배 싸게 계상되던 문제

**이전 식**

```
C = wE·E_motion/E_ref + wS·E_switch/E_switch_ref + wT·T/T_ref
            E_ref=0.5 Wh          E_switch_ref=8.0 Wh
```

두 에너지 항의 참조값이 달라, 같은 1 Wh 라도

| 소모처 | 비용 기여 |
|---|---|
| 주행 | 1.000 = 0.5 × 1/0.5 |
| 모드 전환 | 0.025 = 0.2 × 1/8.0 |

로 **40배** 차이가 났다. 배터리에서 빠지는 1 Wh 는 어디서 쓰든 1 Wh 인데도 그랬다.
그 결과 medium_open 에서 총 52.9 Wh 쓰는 하이브리드 경로가
14.1 Wh 쓰는 밟고넘기 경로를 이기고 '최적'으로 뽑혔다.
에너지 효율 경로를 찾는다는 연구 목적과 정면으로 어긋난다.

**수정된 식**

```
C = wE·(E_motion + E_switch)/E_ref + wS·n_switch + wT·T/T_ref
등가형: C = α·(E_motion + E_switch) + β·n_switch + γ·T
        α = wE/E_ref = 1.0,  β = wS = 0.2,  γ = wT/T_ref = 0.09
```

에너지는 에너지끼리 같은 참조값으로 정규화하고,
전환은 에너지 외 비용(착륙 실패 위험, 자세 재수립, 제어 복잡도)이 있으므로
**횟수 페널티**로 분리했다. `n_switch` 는 이·착륙을 각각 세므로
비행 한 구간이면 `2·wS` 가 붙는다.

회귀 방지 테스트: `test_energy_model.cpp` 의
`SwitchEnergyCostsSameAsMotionEnergy` (같은 1 Wh 는 어디서 쓰든 같은 비용).

### 0.3 등반 에너지 모델 — 위치에너지로는 모델링 불가

URDF 링크 14개 질량 합계는 **2.723 kg**. h=0.7m 등반 위치에너지는
18.70 J → 효율 0.4 적용 시 **0.013 Wh** 로, 평지 주행 **2.6 cm** 에 해당한다.
사실상 공짜라 밟고넘기가 항상 이겨 비교 자체가 무의미해진다.
실제 등반은 모터 토크 급증·슬립·저속 때문에 훨씬 크다.

그래서 `climb_mode.energy_per_height_m` 를 **미지수로 두고 sweep** 한다.
INA226 실측이 나오면 곡선 위에 점 하나만 찍으면 결론이 정해진다.

비용은 **상승분(Δh)에만** 부과한다. 목적지 높이에 비례해 매기면
장애물 위를 여러 셀 지나갈 때 중복 계상되기 때문이다.
올라갈 때 한 번, 위에서는 평지와 동일, 내려올 때 공짜 —
위치에너지와 같은 경로 무관 구조다.

### 0.4 스무딩 아티팩트로 인한 불변식 위반과 수정

sweep 결과에서 `rover_climb` 이 `rover_detour` 보다 최대 **4.84%** 비싼
논리적으로 불가능한 행이 90행 중 14행 나왔다. 단계를 분리해 원인을 특정했다.

| 맵 | 계수 | A\* 격자해 | 스무딩 후 |
|---|---|---|---|
| easy_open | 5.0 | 17.148 ≤ 17.195 정상 | 17.077 > 16.289 위반 |
| medium_open | 12.0 | 28.146 ≤ 28.240 정상 | 27.472 > 26.617 위반 |
| hard_open | 8.0 | 30.937 ≤ 31.031 정상 | 30.242 > 29.479 위반 |

**A\* 격자해는 전부 정상이고 역전은 스무딩에서만 생긴다.**
우회 경로는 5.0~5.8% 개선되는데 밟고넘기 경로는 0.4~2.4% 밖에 개선되지 않는다.
밟고넘기 경로는 장애물을 가로질러 이미 짧아 shortcut 여지가 적고,
우회 경로는 길고 지그재그라 개선 폭이 크다.
shortcut smoothing 이 그리디 휴리스틱이라 최적을 보장하지 않기 때문이다.

**수정**: 모든 파라미터 지점에서 얻은 경로를 한데 모아, 각 지점마다 전부
재평가하고 최소를 취한다 (`evaluate_path`). 후보 집합이 파라미터와 무관하게
같아지므로 단조성이 보장된다. 후보 경로는 전부 실제 실행 가능한 것들이라
결과 조작이 아니다 — 최적 플래너라면 당연히 골랐을 경로다.
안전망으로 `benchmark/modal_compare.py` 의 `enforce_superset` 도 함께 쓴다.

수정 후 **단조성 위반 0건 / 상위집합 위반 0건**.

**남은 한계**: 근본 원인(스무더가 비최적)은 그대로다. 이 보정은 modal 간·
파라미터 간 비교만 바로잡을 뿐, 같은 조건 안에서 스무딩이 놓친 개선은 못 찾는다.
논문에는 '경로 후처리로 shortcut smoothing 을 쓰며 최적은 아니다'를 명시할 것.

### 0.5 등반계수 sweep 결과 — 밟고넘기의 우회 대비 비용 절감률

`benchmark/exp_climb_sweep.py` → `results/benchmark_climb_sweep.json`

| 계수 Wh/m | easy_open | easy_corridor | medium_open | medium_corridor | hard_open |
|---:|---:|---:|---:|---:|---:|
| 0.0 | 15.95% | 6.79% | 21.62% | 0.89% | 16.50% |
| 2.0 | 7.95% | 1.12% | 17.49% | 0.05% | 11.66% |
| 5.0 | 0.00% | 0.20% | 11.29% | 0.05% | 4.54% |
| 10.0 | 0.00% | 0.20% | 0.95% | 0.05% | 0.53% |
| 25.0 | 0.00% | 0.20% | 0.26% | 0.05% | 0.53% |

대략 **5 Wh/m 이하면 밟고넘기가 확실히 유리**하고, 10 Wh/m 를 넘으면 이득이 거의 사라진다.
medium_corridor 는 0.7m 이하 장애물이 거의 없어 처음부터 이득이 없다.

### 0.6 가중치 sweep 결과 — 하이브리드는 시간을 우선할 때만 이긴다

`benchmark/exp_weight_sweep.py` → `results/benchmark_weight_sweep.json`
66개 가중치 조합(wE+wS+wT=1, 0.1 단위) x 5맵 x 3modal = 990회 계획.
등반계수는 현재 추정치 2.0 Wh/m 고정.

**승자 분포 (330행)**

| modal | 승 | 비율 |
|---|---:|---:|
| 밟고넘기 | 269 | 81.5% |
| 하이브리드 | 56 | 17.0% |
| 우회 | 0 | 0.0% |
| 동률 | 5 | 1.5% |

동률 5건은 전부 `wE=0, wS=1.0, wT=0` — 에너지와 시간을 완전히 무시해
지상 경로가 모두 비용 0 이 되는 축퇴 조합이다. 승리로 집계하지 않는다.

**에너지 가중치 wE 에 따른 승자 (핵심)**

| wE | 하이브리드 | 밟고넘기 | 우회 | 동률 |
|---:|---:|---:|---:|---:|
| 0.0 | 43 | 7 | 0 | 5 |
| 0.1 | 12 | 38 | 0 | 0 |
| 0.2 | 1 | 44 | 0 | 0 |
| 0.3 | 0 | 40 | 0 | 0 |
| 0.5 (현재) | 0 | 30 | 0 | 0 |
| 1.0 | 0 | 5 | 0 | 0 |

**하이브리드는 wE ≤ 0.2 에서만 이긴다.** wE ≥ 0.3 이면 한 번도 못 이긴다.
현재 설정이 wE=0.5 이므로 지금 가중치에서는 비행이 선택되지 않는다.

이유는 명확하다. 하이브리드가 이긴 경우 에너지를 **3.0~4.6배** 더 쓴다.

| 가중치 | 맵 | 하이브리드 | 밟고넘기 | 배수 |
|---|---|---:|---:|---:|
| 0.0/0.0/1.0 | easy_open | 45.2 Wh | 10.3 Wh | 4.39x |
| 0.0/0.0/1.0 | medium_open | 51.6 Wh | 14.1 Wh | 3.66x |
| 0.0/0.0/1.0 | hard_open | 56.0 Wh | 18.7 Wh | 3.00x |

즉 비행은 **시간을 사는 대신 에너지를 3~4배 지불**하는 선택지다.
에너지를 조금이라도 중시하면(wE ≥ 0.3) 이 거래가 성립하지 않는다.

**현재 설정(0.5/0.2/0.3)에서 맵별 비교**

| 맵 | 우회 | 밟고넘기 | 하이브리드 | 승자 | 승자 에너지 |
|---|---:|---:|---:|---|---:|
| easy_open | 16.231 | **14.993** | 16.231 | 밟고넘기 | 9.8 Wh |
| easy_corridor | 17.628 | **17.296** | 17.628 | 밟고넘기 | 11.2 Wh |
| medium_open | 26.608 | **21.963** | 26.608 | 밟고넘기 | 14.1 Wh |
| medium_corridor | 28.739 | **28.681** | 28.739 | 밟고넘기 | 17.9 Wh |
| hard_open | 29.299 | **25.975** | 29.299 | 밟고넘기 | 16.7 Wh |

하이브리드 비용이 우회와 소수점까지 같다 — **비행을 한 번도 쓰지 않았다**는 뜻이다.
하이브리드는 밟고넘기를 못 하므로 우회 아니면 비행인데, 비행이 이득이 안 되니
우회와 같은 경로가 된다.

**해석 시 주의**: 이 결과는 등반계수 2.0 Wh/m 라는 **미검증 추정치**에 의존한다.
INA226 실측이 이 값보다 훨씬 크면 밟고넘기의 우위가 줄고 판도가 바뀐다.
0.5절의 표에서 실측값에 해당하는 행을 보면 된다.

---

### 0.7 C++ 비행 결함 — 재현 → 수정 → 검증 (완료)

**결함**: C++ 3D 상태공간이 지형 추종 고도(`airZ = 지형높이 + flight_clearance`)를
쓰는데, 이는 상승각 제약과 충돌한다.

```
격자 0.05m, 최대상승각 45도 -> 한 스텝 최대 고도변화 0.050 m
0.60m 장애물 경계에서 필요한 고도변화        0.600 m  -> 거부
```

**하이브리드 플래너가 어떤 장애물도 날아서 넘지 못했다.** 이륙은 되지만
장애물 경계에서 모든 공중 전이가 거부되어 실질적으로 `rover_detour` 와 같았다.

**1) 결함을 코드로 재현** — `test/test_state_space.cpp` 신설

수정 전 실행 결과 (대조군이 통과하므로 테스트 자체는 멀쩡하다):

| 테스트 | 수정 전 |
|---|---|
| `ObstacleIsFlyableButNotDrivable` | OK (설정은 의도대로) |
| `ModalGatesTakeoffAndClimb` | OK (3 modal 게이팅 정상) |
| `RoverClimbCrossesObstacleOnGround` | OK (대조군 — 밟고넘기는 건넌다) |
| `FlightStepCanEnterObstacleAirspace` | **FAILED** |
| `ReachesAcrossFlyOverObstacle` | **FAILED** |

**2) 수정** — Python 과 같은 '비행 구간 고도 고정' 방식으로 전환

- `State` 에 `level`(비행 고도 인덱스) 추가
- `ProblemSpec::buildAirLevels()` — costmap 을 훑어 후보 고도 열거
  (`지형높이 + flight_clearance`, 등급이 4개뿐이라 후보도 소수)
- 이륙 시 고도 선택, 비행 중 고정, 착륙 시 하강
- 공중 이동에서 수직 비용·상승각 검사 제거 (고도가 안 변하므로)
- `index()` 를 `셀 x (1 + 고도후보수)` 로 확장

함께 고친 것:
- `groundSegmentCost()` — 스무딩 직선이 장애물 위를 지날 때 등반 비용 반영
  (없으면 rover_climb 에서 밟고넘기가 공짜가 된다. Python 쪽과 같은 규칙)
- `buildSwitchPlan()` — 비행 구간 에너지를 `airMoveHorizontal(dist, 고도)` 로
  재계산하던 것을 실제 구간 비용 누적으로 교체.
  두 번째 인자는 '고도'가 아니라 '지형 상단으로부터의 여유'라 의미가 달랐다.

**3) 검증**

```
73 tests, 0 errors, 0 failures      (gtest 26 + 린터)
```

수정이 진짜 원인이었는지 확인하려고 **이륙 고도를 level 0 하나로 제한하는
개악을 일시 적용**했더니 `ChoosesHigherAltitudeWhenLowestIsNotEnough` 가
다시 실패했다. 테스트가 로직을 실제로 붙잡고 있다는 뜻이다. 개악은 되돌렸다.

Python↔C++ 고도 후보 규칙도 대조했다:

| 맵 | Python | C++ 테스트 단언 |
|---|---|---|
| 평지 + 0.60m | [0.80, 1.40] | [0.80, 1.40] |
| 평지 + 0.90m | [0.80, 1.70] | [0.80, 1.70] |

**작업 중 발견한 자기 실수**: 처음 쓴 BFS 헬퍼가 방문 집합 키에서 `level` 을
빠뜨려, 같은 셀의 다른 고도가 이미 방문한 것으로 처리됐다. 그래서 코드는
멀쩡한데 `ChoosesHigherAltitudeWhenLowestIsNotEnough` 가 실패했다.
0.60m 장애물은 가장 낮은 고도(0.8)로도 넘어져서 이 실수가 드러나지 않았고,
0.90m 케이스를 추가하고 나서야 잡혔다.

---

## 1. 알고리즘 선택 (완료)

`benchmark/` 에서 A\* vs RRT\* 를 실측 비교했다. 상세: `benchmark/RESEARCH_LOG.md`
발행본: https://claude.ai/code/artifact/04e8cfc2-d027-478e-9476-f57e14c99c31

**결론 (2초 제약, 시드 20개, 정규화 비용함수)**

| 조건 | 선택 | 근거 |
|---|---|---|
| 3D (현재 설계) | **A\*** | 0.12~1.00초에 최적해, 결정론적(σ=0) |
| 비행 활발 조건 | RRT\* 근소 우위 | 해 품질 1~3%. A\*는 격자 촘촘히 하면 따라잡지만 제약 위반 |
| 4D 확장 | RRT\* | A\*는 전부 timeout |

A\*를 택한 실질적 이유는 해 품질(1~3%)이 아니라
**최적성 보장 / 결정론적 재현성 / 시간 여유 2~16배 / 튜닝 부담 적음**이다.
에너지 파라미터가 72배 불확실한 상태라 3% 차이는 근거가 약하다.

---

## 2. C++ 구현 (완료, 원격 빌드·테스트 통과)

### drobot_hybrid_planner — Hybrid A\* Nav2 플러그인

```
include/drobot_hybrid_planner/
  energy_model.hpp         비용 모델 (benchmark/cost/energy.py 이식)
  state_space.hpp          상태·전이 정의 (state_space.py 이식)
  hybrid_astar_planner.hpp Nav2 GlobalPlanner 플러그인
src/
  energy_model.cpp
  state_space.cpp
  hybrid_astar_planner.cpp  A* 탐색 + 스무딩 + ModeSwitchPlan 퍼블리시
test/
  energy_fixtures.hpp       Python이 생성한 기댓값 (자동 생성)
  test_energy_model.cpp     Python↔C++ 동등성 테스트
```

**이식 시 특히 신경 쓴 것** (Python 벤치마크에서 실제로 버그였던 지점)
- `check_diagonal_corners` — 대각 이동이 벽 모서리를 관통하지 않게
- `maxDzFor()` — 상승각을 격자 칸수가 아니라 물리 각도로 정의
- priority_queue 의 tie-breaker — 동률 시 순서가 흔들리면 결정론이 깨진다
- 스무딩은 비용이 실제로 줄 때만 채택 (최악의 경우 원본 유지)

### drobot_costmap_2_5d — ElevationLayer

RGB-D 포인트클라우드로 셀별 높이를 누적하고 4단계로 분류한다.
경사(3x3 평면 피팅) / 거칠기(표준편차) / 단차(인접 셀 차이)로 traversability 판정.

**알려진 한계**: Nav2 Costmap2D는 cost(0~255)만 저장하므로 높이가 전달되지 않는다.
플래너는 cost 등급에서 대표 높이를 역추론한다(`CostmapTerrainSource`).
정밀 높이가 필요해지면 높이맵을 별도 토픽으로 넘기도록 바꾸면 된다.

### 비용함수 정규화 전환

```
이전:  C = α·E_motion + β·E_switch + γ·T          (Wh와 s가 섞임)
현재:  C = wE·E/E_ref + wS·E_switch/E_sw_ref + wT·T/T_ref   (전부 무차원)
       α = wE/E_ref 로 정확히 동등
```

전환하니 이전 설정(α=1.0, β=1.0, γ=0.5)이 **모드 전환 페널티 79%** 라는 게
드러났다. 그래서 최적해가 비행을 한 번도 선택하지 않았던 것.
현재 wE=0.5 / wS=0.2 / wT=0.3 에서는 비행이 정상적으로 선택된다.

---

## 3. 다음에 할 일

### (A) 빌드 검증 — 가장 먼저

**빌드 환경 (2026-08-24 확인/구축)**

| | 맥북 (개발) | 랩실 데스크탑 `vail-detop` (빌드/실행) |
|---|---|---|
| CPU | Apple M4 (arm64) | x86_64 16코어 |
| 메모리 | — | 62GB |
| GPU | — | RTX 3070 Ti |
| OS | macOS | **Ubuntu 20.04** |
| ROS | 없음 | **ROS1 Noetic** (ROS2 아님) |

**중요**: 랩실 데스크탑은 Ubuntu 20.04 + ROS1 Noetic 이다.
ROS2 Jazzy 는 Ubuntu 24.04 용이라 여기 직접 설치할 수 없다.
그래서 **Docker 컨테이너**로 빌드한다. 맥북은 arm64 라서
x86 ROS2 이미지가 에뮬레이션으로 매우 느려 빌드 환경으로 부적합하다.

구축한 것 (기존 ROS1 환경은 건드리지 않음):
- Docker 28.1.1 + Compose v2.35.1 (공식 저장소)
- nvidia-container-toolkit 1.20.0 (GPU 접근 확인 완료)
- SSH 키 인증 등록

**빌드 절차**

```bash
# 1) 동기화 (맥북에서)
./sync.sh

# 2) 원격 컨테이너에서 빌드
ssh vail-detop
cd ~/drobot-research/docker
sudo docker compose up -d ros2
sudo docker exec -it drobot_ros2 bash

# 컨테이너 안에서
cd /app
colcon build --symlink-install \
  --packages-select drobot_msgs drobot_hybrid_planner drobot_costmap_2_5d
source install/setup.bash
colcon test --packages-select drobot_hybrid_planner
colcon test-result --verbose
```

**`--packages-select` 를 꼭 쓸 것.** `src/px4_msgs` 와
`src/px4-ros2-interface-lib` 는 git submodule 인데 초기화되지 않아 비어 있다.
전체 빌드하면 거기서 걸린다. 필요해지면:
`git submodule update --init --recursive`

**첫 빌드는 오류가 날 가능성이 높다** — Nav2 API 가 배포판마다 조금씩 다르다.
특히 `nav2_core::GlobalPlanner::createPlan()` 의 `cancel_checker` 인자는
Jazzy 부터 추가된 것이라 확인이 필요하다.

### (B) 시뮬레이션 동작 확인 ✅ 완료 (2026-08-24)

**전체 파이프라인이 끝까지 작동함을 확인했다.**

```
RGB-D 포인트클라우드 (4.5Hz, camera 프레임)
  -> TF 변환 (camera -> map)
  -> ElevationLayer 높이 누적 (관측 1658셀, 0.36~1.20m 인식)
  -> 4단계 분류 -> costmap
  -> HybridAStarPlanner 경로계획 (0.246초)
  -> 모드 전환 2회 (이륙 + 착륙)
  -> /mode_switch_points 발행
```

ModeSwitchPlan 실제 출력:
```
switch_type 0 (GROUND_TO_AIR)  (1.03, 5.88)  고도 0.95m  6.9 Wh
switch_type 1 (AIR_TO_GROUND)  (1.18, 5.63)  고도 0.95m  4.9 Wh
pair_id 0 (동일)               total_flight_energy 12.38 Wh
```
- 이륙-착륙이 같은 pair_id 로 짝지어짐 (BT 매칭용 설계대로)
- 비행 고도 0.95m = 장애물 0.15m + flight_clearance 0.8m
- 에너지 6.9 = takeoff 5.0 + 2.0*0.95, 4.9 = landing 3.0 + 2.0*0.95

**실행 방법 (headless 기본 — 아래 주의사항 참고)**
```bash
./sync.sh sim medium_open proposed     # headless
./sync.sh sim medium_open proposed gui # 화면 필요할 때만
./sync.sh stop                         # 확인 끝나면 반드시
```

### 시뮬레이션에서 해결한 문제 5건

| # | 증상 | 원인 | 조치 |
|---|---|---|---|
| 1 | 월드 XML 파싱 실패 | Git LFS 자산이 서버에 없음(404) | 벤치마크 맵 6종을 SDF로 변환 (`benchmark/export_sdf.py`) |
| 2 | `Unable to read STL` | 메시 13개도 LFS 포인터 | 관성에서 치수 역산 -> `drobot_primitives.urdf.xacro` |
| 3 | `PointCloud 프레임이 camera인데 costmap은 map` | ElevationLayer에 TF 변환 미구현 | `Layer::tf_` 로 실제 변환 구현 |
| 4 | `목표점이 costmap 밖이다` | static_layer가 SLAM 맵 크기를 따라감 | costmap 30x20m 고정, static_layer 제거 |
| 5 | **계획에 23.9초** (timeout 2초) | 스무딩이 O(passes*n^3) | prefix sum + lookahead -> **0.288초 (83배)** |

5번은 벤치마크(격자 0.1m, 짧은 경로)에서 드러나지 않다가 실제 시뮬레이션
(격자 0.05m, 381점)에서 터졌다. timeout 초과 경고를 코드에 심어 재발을 잡는다.
수정 후 동등성 테스트 55개는 그대로 통과한다.

### ⚠️ 머신 운용 주의 (두 번 마비됨)

**시뮬레이션과 빌드를 절대 동시에 돌리지 않는다.**
RViz + Gazebo GUI 가 X 서버와 GPU 를 점유하면 RustDesk 화면 입력이 먹통이 되고
SSH 까지 끊긴다. 실제로 랩실 데스크탑이 두 번 재부팅됐다.

- `sync.sh build/test` 는 시뮬레이션을 자동 종료한 뒤 빌드한다
- `sync.sh sim` 은 기본이 headless (GUI 없음)
- 병렬 작업 제한: `--parallel-workers 2~3`, `MAKEFLAGS=-j6`
- 확인이 끝나면 반드시 `./sync.sh stop`

### Git LFS 문제 (미해결, 근본 원인)

`worlds/`, `meshes/`, `models/` 의 파일 680개가 LFS 포인터(130B)만 남아 있고
**GitHub LFS 서버에 실제 데이터가 없다** (`Object does not exist: 404`).
포인터만 커밋되고 업로드가 안 된 상태다. 로컬 3곳을 확인했으나 전부 포인터다.

대응:
- 월드: 벤치마크 맵을 SDF 로 생성해서 씀 (오히려 하이브리드 테스트에 적합)
- 메시: 단순 도형 URDF 로 대체 (`robot_model:=primitives`, 기본값)
- `sync.sh` 가 `worlds/ models/ meshes/` 를 exclude 해서 원격의 실제 파일을
  로컬 포인터로 덮어쓰지 않도록 했다
- STL 원본을 되찾으면 `robot_model:=mesh` 로 원본을 쓰면 된다

### (C) 미구현 — 남은 패키지

| 패키지 | 상태 |
|---|---|
| `drobot_mode_manager` | 스캐폴드만. `/mode_switch_points` 구독 → PX4 이착륙 명령 |
| `drobot_experiments` | 스캐폴드만. baseline 4종 자동 실행 + CSV 수집 |
| `drobot_energy_model` | `energy_logger.py` 작성됨. INA226 드라이버 노드는 미작성 |

### (D) Phase 2 실측

`energy_logger.py` 가 JMP 분석용 `segment_summary.csv` 를 바로 만든다.

```bash
ros2 run drobot_energy_model energy_logger

# idle 전력 먼저 측정 (로봇 정지 상태에서)
ros2 service call /energy_logger/measure_idle std_srvs/srv/Trigger

# 구간 측정
ros2 topic pub --once /energy/segment_control std_msgs/String \
  "data: 'start|run01|seg01|ground|speed=0.3,slope=5.0'"
ros2 topic pub --once /energy/segment_control std_msgs/String "data: 'stop'"
```

**최우선 측정 대상: `takeoff_energy` / `landing_energy`.**
현재 값(5.0/3.0 Wh)은 같은 config의 `hover_power=50W` 로 계산한
0.069 Wh 의 약 72배다. 이 값이 알고리즘 선택보다 결과에 훨씬 큰 영향을 준다.

실측 후 `energy_params.yaml` 을 고치면:
1. `python3 benchmark/gen_cpp_fixtures.py` 로 테스트 기댓값 재생성
2. `python3 benchmark/run_benchmark.py --only param` 으로 벤치마크 재실행
   (결론이 유지되는지 확인)

---

## 4. 알려진 불일치 / 주의사항

| 항목 | 내용 |
|---|---|
| `flight_clearance` vs `activation_height` | 0.8 > 0.5 이라 3D에서 ground effect가 영원히 안 켜진다. 4D로 가거나 clearance를 낮춰야 의미를 갖는다 |
| `fly_over_max` vs 천장 제약 | elevation_params의 2.0m는 천장 제약상 실제 통과 가능 높이(1.2m)를 넘는다. 레이어가 시작 시 경고를 남긴다 |
| 파라미터 이중 정의 | `flight_clearance`(플래너)와 `robot_flight_height`(costmap)가 같은 값을 가리킨다. 한쪽만 바꾸면 분류와 판단이 어긋난다 |
| `src/README.md` | 삭제된 패키지들을 설명하는 옛 문서. 정리 필요 |
| 등반계수 미검증 | `climb_mode.energy_per_height_m = 2.0` 은 추정치다. 현재 3 modal 결론 전체가 이 값에 걸려 있다. INA226 실측 후 0.5절 표에서 해당 행을 보면 결론이 정해진다 |
| 스무더 비최적 | shortcut smoothing 은 그리디라 최적을 보장하지 않는다. 경로 풀링으로 비교는 바로잡았지만 근본 해결은 아니다 (0.4절) |
| 실험 기록 축 불일치 | `drobot_experiments/config/logging_config.yaml` 의 baseline 목록이 플래너 축(`smac2d`/`drone_only`/…)이라 modal 축과 어긋난다. `trial_summary` 에 `modal` 컬럼도 없다 |

---

## 5. 파일 지도

```
benchmark/                     A* vs RRT* 벤치마크 (ROS 무관, Python)
  RESEARCH_LOG.md              연구일지
  HANDOFF.md                   벤치마크 자체의 진행 기록
  gen_cpp_fixtures.py          C++ 테스트 기댓값 생성
  cost/energy.py               비용 모델 (C++ 이식의 원본)
  planners/                    Dijkstra, A*, RRT*, 스무딩
  results/                     그림과 JSON 원자료

src/drobot_hybrid_planner/     Hybrid A* Nav2 플러그인 (C++)
src/drobot_costmap_2_5d/       ElevationLayer (C++)
src/drobot_energy_model/       energy_logger.py
src/drobot_bringup/config/navigation/
  nav2_params.yaml             baseline (SmacPlanner2D)
  nav2_params_hybrid.yaml      proposed (Hybrid A* + ElevationLayer)
```
