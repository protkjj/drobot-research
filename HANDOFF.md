# drobot-research 진행 상황

**최종 갱신 2026-10-02 — C 트랙 (실행·에너지)**

브랜치 `integration` = A(eunseo) + C(track-c) + 미푸시분. 충돌 0, SSOT 불일치 0.
브랜치 `fix/floor-offset` 을 2026-10-02 integration 에 병합했다 (바닥 높이·B 플래너·에너지 값·회전 임계값).
브랜치 `track-b` (B, 커밋 5개)를 2026-10-02 오전 integration 에 병합했다 (2-2절).

**다음에 할 일 (2026-10-02 오전 기준)**
0. ~~경로가 상자·벽에 붙는 문제~~ — **우리가 고쳤다** (`fix/path-clearance`, 3절 '경로 여유').
   수정 전 3회 중 2회 상자에 박힘(최소 거리 0) → 수정 후 3회 모두 충돌 없음(상자까지 0.65~0.72 m)
0-1. 비행 착륙 직후 aborted 1건 — 로그가 덮여 원인 미확인. 로그를 남기며 재현할 것
0-2. `xy_goal_tolerance: 0.75` (플랫폼 원본 값) — 목표 0.75 m 앞에서 succeeded 로 끝난다.
     도착 정확도를 지표로 쓰려면 줄일지 팀이 정할 것
1. **착륙 규칙 결정 (팀)** — '관측 칸에만 착륙' 때문에 카메라(0.36 m)보다 높은 장애물 뒤로는
   날아서 착륙할 수 없다. 하이브리드가 비행을 거의 안 고를 수 있다 (2-2절)
2. 주행 1회 — 회전(track-b 경로 수정 + 임계값 0.785)·B 증상(21 cm 이착륙, 잦은 재계획)·비행 여부 재측정
3. derived 결과 다시 내기 — 그동안 derived 에너지 값이 버려지고 있었다 (4절)
4. 시뮬은 `sync.sh` 로만 띄울 것 — 직접 `ros2 launch` 를 겹쳐 띄워 시간 역행·Nav2 기동 실패를 겪었다 (RUN 4-8)
5. ElevationLayer 에 `updateOrigin` (rolling window) — 그래야 local costmap 에 다시 넣을 수 있다 (A)
실행 절차는 `RUN.md`, Gazebo 진단은 `tools/diag_gz.sh`.

---

## 한 줄 요약

**C 트랙 단계 1~4 를 전부 구현하고 시뮬에서 전 사이클 완주를 확인했다.**
`GROUND → TAKING_OFF → FLYING → LANDING → GROUND`, 착륙 오차 17 cm.
남은 건 **위치추정이 비행을 못 따라오는 구조적 문제**와 B 트랙 플래너 안정화다.

빈 바닥이 0.25 m 로 보여 fly_over 로 분류되던 것은 **URDF `base_footprint` 높이 오류**가
원인이었다 (gz 실측으로 확정). `fix/floor-offset` 에서 고쳤고 시뮬 재측정이 남았다 (2절).

---

## 1. 오늘 한 것 (2026-10-01 ~ 02)

### C 트랙 — 소유 3개 패키지의 빈칸을 채웠다

| 단계 | 내용 | 검증 |
|---|---|---|
| 0 | `entry_points` 등록 (4개 노드) | 데탑 `ros2 pkg executables` |
| 1 | 전환 판정 `switch_tracker` | 단위테스트 11 + 기록 replay |
| 2 | 이착륙 백엔드 `flight_profile` + `backends` | 단위테스트 15 |
| 3 | `/battery_state` 계약 양쪽 연결 | 배터리 관문 5종 테스트 |
| 4 | `trial_summary` 한 줄 요약 | 기록 2건 + yaml 스키마 일치 |
| — | INA226 레퍼런스 전력값 | 수치 검산 |
| — | 비행 중 Nav2 속도 제한 | 실측 동작 확인 |
| — | 비행 뒤 위치추정 재설정 | **미검증** |

설계: 판정(`switch_tracker`) / 타이밍(`flight_profile`) / 실행(`backends`) /
ROS 배선(`mode_manager`) 넷으로 나눴다. 앞의 셋은 ROS 를 import 하지 않아
**시뮬 없이 맥북에서 전부 검증된다**. 전환이 실패했을 때 어디가 틀렸는지도 갈린다.

### 시뮬이 안 돌던 것 — 원인 둘을 찾아 고쳤다

**① 렌더 엔진** — 월드의 `<render_engine>ogre2</render_engine>` 가
RTX 5070 Ti(Blackwell, driver 580) + gz-sim 8.11 에서 렌더 스레드 초기화를
끝내지 못한다. Sensors 가 그걸 기다리며 메인 루프를 막아 `/clock` 부터 모든
토픽이 0 이 된다. 월드 12개와 생성기를 `ogre`(v1) 로 바꿨다.

> **2026-10-02 정정 — ①은 틀린 판단이었다.** ②를 고친 뒤(root 실행) ogre2 로 다시 띄우니
> 3/3 정상이었다 (gz 토픽 32, 라이다 0.91~6.99 m, Nav2 active). ①과 ②를 동시에 고쳐
> 원인을 섞어 판단했다. 지금은 launch 가 `render_engine:=ogre2` 를 기본으로 월드 사본에
> 덮어쓴다 (track-b). ogre(v1) 는 B 머신(RTX 4070 SUPER)에서 라이다가 모든 빔에 0.5 m 를 냈다.

> **2026-10-02 정정 — ②도 지금은 재현되지 않는다.** ogre2 기준으로 `-u` 로 띄워도 3/3 정상.
> 시뮬은 root 유지 — 붙는 도구가 전부 root 이고 DDS 는 같은 사용자끼리만 통신된다 (RUN 4-9).

**② 실행 사용자** — `docker exec -u $(id -u):$(id -g)` 로 `ros2 launch` 를
띄우면 같은 증상. root 로 띄우면 매번 성공(3/3 vs 전부 실패). `sync.sh do_sim`
에서 `-u` 를 뺐다. **메커니즘은 미규명** — gz sim 단독은 `-u` 로도 매번 된다.

**제거한 가설 (다시 밟지 말 것)**: LD_LIBRARY_PATH · GZ_SIM_RESOURCE_PATH ·
HOME · 브리지와 카메라 구독 · WorldControl unpause · 스폰 타이밍 ·
스폰 방식(-file/-topic) · UID 그룹과 /dev/dri 권한 · 컨테이너·GPU 재시작 ·
노드 기동 지연(startup_delay).

### 로봇 모델 — primitives 를 버리고 원본 메시로

내가 이전 세션에 급조한 `drobot_primitives.urdf.xacro` 가 틀려 있었다.

```
base_link        0.128 x 0.055 x 0.090 m   바퀴 지름 0.212 m 보다 작다
left_front_arm   origin z = -0.216881      팔이 링크 원점에서 22cm 아래
right_front_arm  origin z = -0.175993      좌우 비대칭
```

visual 뿐 아니라 **collision 도 같은 값**이라 물리가 어긋났다. 팔 충돌체가
땅을 긁고 있었을 가능성이 크다 — 메시로 바꾸자 주행이 눈에 띄게 좋아졌다.

```
                 primitives    mesh
회전만 비율         64%          42%
이륙점 도달 시 속도  0.008 m/s    0.402 m/s  (최고속 주행 중 도달)
재계획             60회/33초     20회/13초
```

STL 13개가 실파일로 복구돼 있어 `robot_model` 기본값을 `mesh` 로 바꿨다.
primitives 는 상단에 폐기 사유를 달아 참고용으로만 남겼다.

### 기록이 거짓이던 것 — 고쳤다

`record_run` 이 비행을 하고도 "모드 전환 0회" 로 적었다. `/mode_switch_points`
를 depth=1 로 받는데 Nav2 가 재계획하며 마지막에 전환점 0개짜리 계획을 내면
그걸 덮어썼기 때문이다. 빈 계획으로 덮지 않게 하고, `/mode_state` 를 구독해
**실제로 수행한 전이**를 따로 남긴다. `trial_summary` 의 `num_switches` 도
계획이 아니라 실행 횟수를 쓴다.

---

## 2. 빈 바닥이 0.25 m 로 측정되던 것 — 원인 확정, 수정, 시뮬 검증 완료

브랜치 **`fix/floor-offset`** (integration 에서 분기, 아직 병합 안 함).
이것이 가짜 fly_over 의 원인이다. 다만 아래 "이것으로 안 풀리는 것" 은 별개 문제라 남는다.

### 측정 — 수정 전 (tools/cloud_probe.py · costmap_probe.py, 깨끗한 실행에서)

포인트클라우드 (로봇 (2,1) 에서 +y 를 봄)

```
센서 프레임  x [0.80, 9.89]  y [-3.86, 1.99]  z [-0.39, 2.72]   <- TF 적용 전 원본
전역 프레임  x [0.03, 5.88]  y [ 1.82, 10.90] z [ 0.22, 3.33]
전역 z 분포   0.10~0.30 m : 117,120 (63%)   <- 바닥 자체가 여기 보인다
             0.30~0.55 m :  18,667          <- 장애물(0.5m) 앞면
```

elevation_layer 격자 (x=2.12 선상)

```
y=1.91~3.76   0.25 m    빈 바닥인데 0.25
y=3.97        0.40 m    장애물 앞면
y=4.18~5.00   미관측     장애물 윗면을 못 봄
```

분류: `elevation_layer.cpp:397,409` 가 셀의 절대 `max_z` 를 `rover_traversable_max 0.15`
와 비교하므로 0.25 m 바닥은 fly_over(200) 이 된다.

### 원인 — `base_footprint_joint` 의 z = 0.25

`src/drobot_description/urdf/drobot.urdf.xacro` 의 `base_footprint → base_link` 가 +0.25 m 였다.
TF 는 "base_link 가 바닥에서 25 cm 위" 라고 말하는데, 메시 모델은 base_link 원점이
이미 바닥 높이다 (바퀴 접지점이 base_link 기준 -0.002 / +0.005).

- Gazebo 물리에서는 바퀴가 땅에 닿아야 하므로 base_footprint 가 **땅 밑 -0.25 m** 에 놓인다
- EKF `two_d_mode: true` (`ekf.yaml:13`) 는 base_footprint 를 **z=0 으로 고정**한다
- 그래서 TF 를 거친 센서 점이 전부 +0.25 m 들린다. 카메라 높이: TF 0.61 m / 실제 0.36 m

```
                          예측 (URDF 계산)    실측 (gz model -m drobot -p)
base_footprint z            -0.2516 m          -0.2518 m
roll                         1.17°              1.22° (0.0213 rad)
```

같은 결론을 가리키는 독립 증거

1. **센서 프레임 원본(TF 무관)의 z 최저가 -0.39** — 센서 스스로 "바닥이 39 cm 아래" 라고
   말한다. 카메라가 0.61 m 라면 -0.61 이어야 한다. 전방·좌우 거리는 월드와 맞으므로
   깊이 스케일 문제가 아니다.
2. **장애물 윗면 미관측** — 실제 카메라 높이 0.36 < 장애물 0.5 라서다.
   0.61 이었다면 비스듬히라도 보였다.
3. (보조) 센서 프레임 극값 -0.39 / +2.72 — 좌우 바퀴 메시가 7 mm 비대칭이라 생기는
   roll 1.2° 를 넣으면 -0.394 / +2.722 로 1 cm 안에 맞는다. 시야 끝점 위치 가정에 의존.

### 기각한 가설 — 다시 밟지 말 것

- **카메라 광학 프레임** — `gz_frame_id` 는 `camera_link` 가 맞다 (회전은 정상).
  단, 광학 프레임 교체 실험은 **회전만** 검증한다. 두 링크는 위치가 같아서 높이 오류는
  그 실험으로 잡히지 않는다. 이전 판에 "카메라 좌표계는 무죄로 확인됐다" 라고 적어
  TF 높이를 안 보게 만들었다.
- **자기 가림** — 점이 0.80 m 앞부터 시작하고 0.25 m 층이 2 m 넘게 평평하다.
  가림은 가까운 바닥을 지울 뿐 들어 올리지 않는다. (바퀴 지름도 메시 기준 0.224 m)
- 이전 판의 "바닥(z=0)을 보는 점이 하나도 없다" — 63% 가 바닥 자체였고 0.25 에 보였을 뿐이다.

### 수정 (fix/floor-offset)

| 커밋 | 내용 |
|---|---|
| `2cb1927` | URDF `base_footprint_joint` z 0.25 → 0. 카메라 광학 프레임 주석 정정 |
| | `tools/urdf_ground_check.py` — URDF 만으로 바닥 높이 검사 (ROS 불필요, 맥에서 돈다). 수정 후 -0.002 OK / 수정 전 +0.248 FAIL |
| | primitives 는 폐기 대상이라 값은 두고 같은 결함(+0.205 m)만 상단에 명시 |
| `fd0b127` | `tools/costmap_probe.py` — 환산 토픽 대신 `costmap_raw` 를 읽게 (3절 B 참고) |
| `5ad328f` | `costmap_probe` elevation 모드가 `…/elevation_grid` 토픽을 찾아 쓰게 (실제 이름 `/global_costmap/elevation_layer/elevation_grid`) |
| `cf1369a` | ElevationLayer 자체 격자의 미관측 칸을 NO_INFORMATION 으로 명시 (전에는 미정의 값) |
| `4d0445c` | 플래너 (b-0): 지형은 ElevationLayer 에서, 충돌은 master 에서. 착륙은 관측 칸에만 (3절 B) |
| `7ea2ba6` | uncrustify 정렬 되돌림 (데탑 테스트에서 린터 1건) |
| `dff0643` | **nav2 params: `energy_model` 을 `planner_server` 아래로** — 값 31개가 전부 버려지고 있었다 (4절) |
| `923ff01` | `costmap_probe` 플래너 열을 지금 방식(지형 격자 / master 충돌)으로, elevation 토픽 이름 고정 |

영향 검토 (코드로 확인)

- voxel/obstacle 레이어는 `/scan` 만 쓴다. 라이다 TF 높이 0.55 → 0.30 은 `max_obstacle_height 2.0` 안이라 무관
- ElevationLayer 는 `min_obstacle_height -0.5` 라 z≈0 바닥을 그대로 받는다
- mode_manager 착륙 `set_pose z=0` 은 이제 바퀴가 바닥에 닿는 높이다 (전에는 0.25 m 공중에 놓고 떨어뜨렸다)
- 스폰 `-z 0.05` 그대로 써도 된다 (5 cm 낙하)
- 남는 오차: roll 1.2° 는 two_d_mode 가 TF 에 넣지 않아, 바닥이 옆으로 4 m 에서 ±8 cm 기울어 보인다.
  rover_traversable_max 0.15 와 경사 한계 15° 안이라 분류는 free 로 예상 — **미검증**

### 수정 후 확인할 것 (데탑 시뮬)

```bash
./sync.sh stop && ./sync.sh build        # drobot_description 재설치 — 8 packages 확인
DROBOT_ENERGY=derived ./sync.sh sim base_map_h0.5 proposed
# 40초 뒤 (RUN.md 2절 확인 먼저), 컨테이너 안에서 source 후
gz model -m drobot -p        # z ≈ 0.00            (전: -0.25)
python3 /app/tools/cloud_probe.py --frame map --min-z -0.5 --max-z 3.0
                             # 전역 z 대부분 -0.1~0.1  (전: 0.1~0.3 에 63%)
python3 /app/tools/costmap_probe.py --mode elevation --x0 2.12 --y0 1.5 --x1 2.12 --y1 5.0
                             # 빈 바닥 0.00~0.05 m   (전: 0.25)
python3 /app/tools/costmap_probe.py --x0 2.12 --y0 1.5 --x1 2.12 --y1 5.0
                             # 빈 바닥 free          (전: fly_over 로 예상)
```

넷 다 맞으면 시뮬을 끄고 플래너 테스트를 돌린다 (`test` 는 시뮬을 자동으로 끈다).

```bash
./sync.sh test        # 빌드 + colcon test. LayerTerrainTest 5개 포함, 실패 0 이어야 함
```

2026-10-02 첫 실행: `test_state_space` 13/0 (LayerTerrainTest 5 포함), 계약 9/0, 에너지 18/0, cpplint 통과.
실패 110 중 내 것은 uncrustify 1건(`7ea2ba6` 에서 고침), params 7건은 원래 있던 버그(`dff0643`).
**남는 실패는 원래 있던 린터뿐이다** — flake8 94 + pep257 2 는 전부 `scripts/test_maps.py`
(작은따옴표 Q000 90건 등), copyright 1 은 `src/energy_model.cpp`. CTest 가 이를 한 번 더 센다.

다 통과하면 integration 에 병합하고 push.

### 시뮬 검증 결과 (2026-10-02, base_map_h0.5 · derived · 시계 1개 확인 후)

```
                              수정 전              수정 후
gz model z (base_footprint)   -0.2518 m           -0.0017 m   (예측 -0.0016)
포인트클라우드 전역 z 최저       0.22 m              -0.03 m
바닥이 몰린 구간                0.10~0.30 (63%)     -0.10~0.10 (67%)
빈 바닥 높이 격자 y 1.9~3.9     0.25 m              0.00 m
빈 바닥 cost                   fly_over            free
플래너 지형 소스                master cost         레이어 elevation_layer, 착륙=관측 칸만
이륙 에너지 (derived 0.5)       5.0 (무시됨)         0.5
지형 격자의 장애물 윗면           -                   255 미관측 (master 에서는 free)
```

센서 프레임 원본은 수정 전과 같다 (z [-0.39, 2.72]) — 카메라 실물은 그대로고 TF 만
고쳤으니 그래야 맞다. `test_state_space` 13/0, `test_nav2_params_file` 7/0.

**주의 — 검증 중 시뮬이 두 개 떠 있던 구간이 있었다.** 헤드리스 시뮬 위에 GUI 런치를
하나 더 띄워 `/clock` 이 두 개가 됐고, 모든 노드가 "jump back in time" 을 9,199 번 냈다.
RViz 가 계속 리셋돼 로봇 모델이 사라졌다 — 오늘 낮의 "로봇 모델 토픽이 처음엔 잡히다
나중엔 안 잡힌다" 도 같은 원인일 가능성이 크다 (그때 로그는 없어 확인은 못 했다).
위 표는 GUI 런치를 끄고 시계 publisher 가 1 개인 것을 확인한 뒤 다시 잰 값이다.
재발 방지 절차는 RUN.md 4-8.

**아직 안 한 것**: 목표를 보내 실제로 주행·비행시켜 보는 것 (RUN.md 3절). B 증상
(21 cm 이착륙, 잦은 재계획)과 착륙 제한의 효과는 그 실행에서 다시 재야 한다.

### 이것으로 안 풀리는 것 — 별개 문제

- **장애물 윗면은 여전히 미관측이고, 그게 free 로 보인다.** 카메라는 물리적으로 0.36 m 라
  0.5 m 장애물 위를 못 본다. ElevationLayer 는 미관측 셀을 건너뛰고(`elevation_layer.cpp:469`)
  `track_unknown_space: false`(`nav2_params_hybrid.yaml:216`) 라 master 기본값 free(0) 가 남는다.
  → 장애물 윗면이 "착륙 가능한 평지" 로 보인다. 바닥 수정만으로는 "착륙점이 장애물 한가운데" 가 남는다.
  **→ 플래너에서 착륙만 관측 칸으로 제한했다** (`land_only_on_observed`, `4d0445c`). 주행은 그대로 낙관.
  `track_unknown_space` 는 false 그대로. 카메라 위치를 올리는 건 별개의 하드웨어 결정.
- **플래너가 inflation 값을 지형 등급으로 읽는다** — **수정함** (`4d0445c`, 3절 B 참고)
- 착륙 waypoint z 가 지형높이 무시 (`hybrid_astar_planner.cpp:294`) — **고치지 않음.**
  착륙 오판의 원인은 이 줄이 아니라 미관측 → free 였다. z=0 은 플래너가 믿는 지형과 일치한다

---

## 2-2. Track B 반영 (2026-10-02 오전, track-b 커밋 5개)

| 커밋 | 내용 | 판단 |
|---|---|---|
| `8c8bbb5` | launch `render_engine` 인자, 기본 ogre2 (월드 사본에 덮어씀) | 데탑(RTX 5070 Ti)에서도 ogre2 3/3 정상 — 그대로 씀. 주석의 '5070 Ti 는 ogre2 가 멈춘다' 는 정정함 |
| `3690c7b` | local costmap 에서 ElevationLayer 제외 | 회피책. 레이어가 rolling window 를 지원하지 않아 높이가 엉뚱한 칸에 쌓였다. 근본 해결은 A 의 `updateOrigin`. 그동안 라이다 평면(0.30 m)보다 낮은 장애물은 local 에서 안 보인다 |
| `2c3e457` | 컨트롤러로 나가는 경로를 0.05 m 간격 + 진행 방향 yaw 로 (`densifyPath`) | **진짜 버그 수정.** 듬성한 꺾임점·yaw 0 이라 DWB 가 점 사이를 경로로 못 봤다. 어젯밤 yaw 회전 과다의 주된 원인으로 보인다 |
| `6e698c2` | 비행 1구간 5 m 제약(상태에 비행 거리, 0.25 m 구간) + 휴리스틱 하한 min(지상, 비행) | 지배 판정 논리 맞음 (비행 비용이 비행 거리와 무관 + 휴리스틱 일관). 같은 0.25 m 구간 안의 손실은 B 주석대로. **휴리스틱 수정은 진짜 버그** — derived 는 비행 0.78 < 주행 0.80 이라 하한이 실제 비용을 넘었다 |
| `92e18da` | 출발점 0.35 m 안의 253 은 충돌로 안 침 + 목표가 막히면 0.5 m 안의 가까운 칸으로 | 출발 쪽은 내(4d0445c) 충돌 판정이 만든 회귀를 잡은 것. **테스트 없음.** 목표 대체는 결과가 조용히 'succeeded' 가 되니 목표 오차 지표에서 감안할 것 |

**회전 개선의 귀속은 나눌 수 없다.** B 의 '회전만 76% -> 7%' 는 내 임계값 수정(0.785)이 이미 들어간
브랜치에서, 경로 수정·local 높이 제외와 함께 들어갔다. 또 같은 '205 s 에 3 m' 실행을 3690c7b 는
64%, 2c3e457 은 76% 로 적었다 — B 에게 확인할 것. 논문에 쓰려면 하나씩 빼 보는 비교가 필요하다.

**비행은 코드상 가능하지만 base_map 에서는 잘 안 나올 수 있다.** derived 가 이제 실제로 적용돼
1 m 비용이 비행 0.78 < 주행 0.80 이고, 0.5 m 상자는 고도 0.8 m 로 넘고, 상자 깊이 3 m 는 5 m 제약 안이다.
그런데 '관측 칸에만 착륙'(4d0445c) 때문에, 카메라(0.36 m)가 못 보는 상자 뒤에는 착륙할 수 없다.
실제 드론이라면 0.8 m 상공에서 너머를 볼 텐데, hop 백엔드는 비행 중 관측이 없다.
선택지: (a) 지금대로 (b) 실험 때만 `land_only_on_observed: false` (c) 비행 고도에서 보일 칸이면 착륙 허용
(가시성 계산 필요). 팀 결정.

## 2-1. 함께 남은 것 — 위치추정이 비행을 못 따라온다

`set_pose` 순간이동은 IMU 에 가속도를 남기지 않아 EKF 가 걸러낸다. 그래서
TF 가 옛 자리를 가리키고 Nav2 가 로봇이 안 움직였다고 믿는다.

지금 조치 (동작 확인됨)
- 이륙 전 `/slam_toolbox/pause_new_measurements` 로 SLAM 정지
- 착륙 후 `/set_pose` 로 EKF 재설정 -> SLAM 재개 (순서 중요)
- 비행 중 `/speed_limit` 1% 로 지상 컨트롤러 묶기
- `hop` 은 끝에 한 번만 자세를 바꾼다 (중간에 공중 방치하면 Nav2 가 죽는다)

이것으로 `aborted` 가 사라지고 `timeout` 까지 버틴다. 다만 EKF 만 다시
잡는 보정이고, 근본적으로는 운동학 재생 방식의 비용이다. 실물 PX4 백엔드로
가면 사라진다 (`backends.Px4Backend`, 지금은 빈 껍데기).

## 3. 트랙별 현황

| 트랙 | 상태 |
|---|---|
| **A** 지각·지도 | Gazebo spawn, 센서 토픽, TF `map→base_footprint`, depth PointCloud2, global_costmap, elevation_layer(local·global 로드 + 분류 로직), robot_physical SSOT 는 동작. **단 TF 높이가 0.25 m 틀려 있어 분류 결과가 틀렸다** — URDF 수정(2절), 재측정 대기 |
| **B** 계획·비용 | 동작하나 불안정 — 아래 참고 |
| **C** 실행·에너지 | 단계 1~4 완료, 전 사이클 실증. INA226 실측과 위치추정 문제가 남음 |

### B 에게 (수정 중이라고 들음)

**먼저**: 아래 증상들은 바닥 오프셋(2절) 수정 **전**에 잰 것이다. 빈 바닥 전체가
fly_over 로 보이던 상태라 플래너는 어디서든 날 이유가 있었다.
**`fix/floor-offset` 반영 후 다시 재고 나서 고칠 것.**

```
21cm 구간에 이착륙을 건다        쌍 0: (0.27, 3.93) → (0.43, 4.08) 거리 0.21 m
                                 전환 1회 11.2 Wh 를 쓰면서
                                 이륙점은 왼쪽 벽 안쪽면(x=0.1)에서 0.17 m
                                 — 벽이 LETHAL 이었다면 아래 inflation 띠 안
재계획이 잦고 전환점이 매번 튄다   20회/13초, 매 계획마다 다른 자리
착륙점이 장애물 한가운데          (1.33, 4.53) 지형높이 0.50 m 인데 z=0.0
                                 지상 waypoint z 가 지형높이 무시
                                 hybrid_astar_planner.cpp:294
                                 그 자리는 장애물 윗면 = 미관측 = free(0) 로 보이는 곳 (2절 끝)
```

**경로 여유 — 경로가 상자·벽에 붙는다 (2026-10-02) → 우리가 고쳤다 (`8469e25`)**

구현: 계획마다 '이 modal 의 로버가 설 수 없는 칸'(지형 > roverHLimit, LETHAL 포함)까지의
거리 지도를 만들어 (1) `min_ground_clearance` 0.30 m 미만은 주행·착륙 불가 (출발점 0.35 m 안은 완화)
(2) `clearance_weight`·exp(-decay·(d - 0.30)) 비용을 1 m 당 더한다 (1.0, 2.0, 1.0 m 까지).
비용은 E·T 와 별도 항(`clearance_penalty`)이고 스무딩 직선에도 같은 규칙. **밟고넘기에서는 0.7 m
이하 장애물이 올라탈 곳이라 여유를 두지 않는다.** 코드 기본은 꺼짐(0), nav2 yaml 에서 켠다.
참고: 플랫폼 원본(protkjj/drobot)은 SmacPlanner2D + inflation 기울기로 가운데를 골랐고
`rules.yaml` 의 `wall_clearance: 0.3` 은 선언만 있고 쓰이지 않았다 — 0.30 은 거기서 가져왔다.

시뮬 (base_map_h0.5, derived, `sync.sh run`, 상자까지 최소 거리는 /odom 기준):

```
            결과        상자까지 최소   벽까지 최소   비행
수정 전 1   aborted     0.000 m (충돌)   —            0
수정 후 1   succeeded   0.650 m         0.894 m      0   (통로 가운데 x≈5.0 로 우회)
수정 후 2   aborted     0.719 m         0.625 m      1   (착륙 직후 중단 — 원인 미확인)
수정 후 3   succeeded   0.681 m         0.859 m      0
```
테스트: test_state_space 25/0 (경로 여유 7개 — modal 구분, 경계, 비용 모양, 꺼짐=예전,
출발 완화, 격자=직선, 여유를 지키며 비행 횡단).

같이 본 것: 목표 (2, 10) 의 표시용 원판(goal_marker)이 지형으로 읽혀(master 100) 목표가
(1.68, 10.3) 쯤으로 옮겨진다. 월드 쪽(마커에 충돌·시각 형상을 카메라가 보지 않게) 문제다.

아래는 수정 전 분석이다.

시뮬에서 직접 봤다: A* 가 로봇을 점으로 보고 경로를 내서 상자 모서리를 바짝 돈다.
base_map_h0.5 derived 3회 중 2회 로봇이 상자 오른쪽 앞 모서리에 박혀 aborted —
한 번은 뒤집혔고(gz roll -145°), 다른 한 번은 출발 칸 (3.86, 3.98) 이 상자 면에서 2 cm 였다.
그 뒤로는 자기 칸이 fly_over(200) 라 '시작점이 로버가 설 수 없는 셀' 이 반복된다.

- **상자(fly_over)에는 몸체 충돌 검사가 없다.** LayerTerrainSource 의 충돌은 master 253·254 만
  보는데, 1.2 m 이하 장애물은 ElevationLayer 가 200 으로 덮어써 inflation 이 안 붙는다.
  그래서 로봇 중심이 상자 면 바로 옆을 지나도 된다고 본다 (반폭 0.225 m 만큼 겹침).
  예전 방식(master 를 등급으로 읽기)도 같은 구멍이 있었다.
- **벽은 0.225 m 까지 붙는다.** Nav2 순정 플래너는 inflation 을 '가까울수록 비싼 비용' 으로
  더해 가운데를 선호하는데, 이 플래너에는 그 항이 없다 (예전엔 inflation 을 지형으로 오독한
  덕에 우연히 0.45 m 떨어졌다).

제안 (결정은 B): 계획마다 '로버가 못 가는 칸'(지형 > rover 또는 LETHAL)까지의 거리 지도를
만들어, (1) 반폭 미만은 주행 불가 (2) 가까울수록 비용 추가 — E·T 와 별도 항, 가중치 0 이면
지금과 같게. 스무딩(groundSegmentCost)에도 같은 규칙을 넣어야 직선화가 모서리를 다시 자르지 않는다.
임시로 `footprint_padding` 은 벽(LETHAL)에만 효과가 있고 상자에는 소용없다.

같이 볼 것: 목표 대체(goal_tolerance 0.5 m) 때문에 (2, 10) 목표를 0.72 m 떨어진 (2.61, 9.61) 에서
'succeeded' 로 끝낸 실행이 있었다.

**새로 — `CostmapTerrainSource` 가 inflation 값을 지형 등급으로 읽는다 → 수정함** (`4d0445c`, 테스트 대기)

B 에게: kj 승인으로 플래너 코드를 고쳤다. `COST_HEIGHT_CONTRACT.md` 의 권고 (b-0) 를 그대로
구현했고, 상단에 갱신 내용과 문서 정정 두 가지를 적었다. 진행 중인 작업과 겹치면 알려 줄 것.

`state_space.cpp:39-46` 이 inflation 이 섞인 master costmap 의 cost 를
`≤50 free · ≤150 rover · ≤253 fly_over` 로 읽는다. global costmap 설정
(inflation_radius 1.0 · cost_scaling 2.0 · footprint 0.45 → 내접반경 0.225)에
Nav2 inflation 공식을 넣으면:

```
LETHAL 셀로부터   0 ~ 0.45 m   cost 253~151  ->  fly_over  (주행 불가, 날아야 함)
                 0.50 ~ 1.0 m  cost 150~51   ->  rover     (0.15 m 지형)
                 (칸 중심 거리, 계약 문서가 실제 InflationLayer 로 잰 값. 공식 손계산은 0.48)
                 1.0 m ~                       free
```

즉 **벽 옆 약 0.5 m 띠를 플래너는 날아야 하는 땅으로 본다.** 단 LETHAL 이 되는 건
**카메라가 1.2 m 넘게 관측한 셀**(벽 3 m)뿐이다. ElevationLayer 는 라이다가 찍은 셀을
0.16 m 로 기록하고(`elevation_layer.cpp:306-308`) 관측 셀을 자기 등급으로 덮어쓰므로
(`:474-478`), 라이다만 본 셀과 1.2 m 이하 장애물은 200 이 되어 inflation 이 안 붙는다.

적용한 방식 (`LayerTerrainSource`, state_space.hpp)

```
지형 높이·등급   ElevationLayer 자체 격자 (inflation 전 값)    미관측은 평지 (주행 낙관)
충돌            master 가 253·254 면 어느 modal 이든 못 선다    master 254 는 비행도 막는다
착륙            관측된 칸에만 (land_only_on_observed: true)
대비책          레이어를 못 찾으면 예전 방식 + 경고
```

남은 한계: 등급 대표 높이(fly_over = 0.60 m 하나)가 실제 높이를 가리는 문제
(계약 문서 4절)는 그대로다. 그건 (b-1) — 레이어의 실제 높이를 읽는 단계다.
또 비행 고도 후보(`buildAirLevels`)는 configure 때 한 번만 만든다. 지금은 대표 높이가
0.60 하나라 0.8 m 하나로 충분하지만, (b-1) 에서 실제 높이를 쓰면 다시 봐야 한다.

**실측 도구 주의**: `/global_costmap/costmap` 토픽은 Nav2 가 0~100 으로 환산해서
rover=39, fly_over=77, LETHAL=100 으로 보인다. 지형과 inflation 을 구분할 수 없다.
`tools/costmap_probe.py` 는 이제 `costmap_raw`(원본 0~255)를 읽고, inflation 이 지형으로
읽히는 셀에 `⚠` 를 붙인다. 이전 버전 출력(100 을 rover 로 표시)으로 판단한 게 있으면 다시 볼 것.

### 공동 — 킥오프에서 정할 것

```
angular_dist_threshold: 0.1 은 5.7° 인데 주석은 45° (Nav2 기본 0.785)
  -> 2026-10-02 세 파일 모두 0.785 로 고침 (fix/floor-offset). 시뮬에서 yaw 회전 과다를 봤고,
     0.1 로는 회전이 수렴할 수 없다: 멈추는 데 필요한 각 1.5^2/(2*5.0)=0.225 rad > 허용 0.1 rad.
     효과는 재측정 필요 (record_run 의 '회전만 %', 전에는 42%)
규약은 TF 를 base_link 로 정했는데 실제는 base_footprint
  fix/floor-offset 이후 둘은 같은 자리다 (항등 변환). 문서만 맞추면 된다
track_unknown_space: false — 미관측을 free 로 봐서 장애물 윗면이 착륙 가능으로 보였다 (2절 끝)
  플래너에서 "주행은 낙관, 착륙은 보수" 로 나눴다 (land_only_on_observed). false 는 유지.
  대가: 카메라(0.36 m)가 못 보는 장애물 뒤에는 착륙할 수 없으니, 그런 곳은 우회해야 한다
월드 이름에 '.' 이 들어가 ROS 서비스 경로로 못 쓴다 (base_map_h0.5)
  mode_manager 가 gz CLI 로 떨어지는 이유. 이름을 바꾸거나 리맵이 필요
```

---

## 4. 에너지 파라미터가 서로 안 맞는다

> **⚠ 2026-10-02 — 지금까지 시뮬은 설정 파일의 에너지 값을 쓰지 않았다.**
> `nav2_params_hybrid*.yaml` 의 `energy_model:` 블록이 최상위 키라서 rcl 이
> 'energy_model' 이라는 (존재하지 않는) 노드의 설정으로 읽고 버렸다. 플래너는 C++ 기본값
> (= default 세트)으로 계획했고, **`DROBOT_ENERGY=derived` 로 돌린 기록도 실제로는 default 로
> 계획된 것이다.** `test_nav2_params_file` 7건이 이걸 잡고 있었다. `dff0643` 에서 값은 그대로 두고
> `planner_server: ros__parameters:` 아래로 옮겼다. derived 결과는 이 수정 뒤에 다시 내야 한다.

```
ground  0.5 Wh/m x 0.3 m/s =  540 W
air     2.0 Wh/m x 0.5 m/s = 3600 W      hover_power 50 W 의 72배
derived 0.6 Wh/m x 0.5 m/s = 1080 W      21배
```

레퍼런스 전력표(`drobot_energy_model/config/reference_power.yaml`)는
전력 단위로 적힌 유일한 값 `hover_power: 50 W` 만 근거로 삼았다.

```
111 Wh 팩 기준   정지 1092분 · 지상주행 282분 · 비행 108분
이착륙 9초 실소모 0.154 Wh
  vs default 전환 1회 8.0 Wh  (52배)
  vs derived 전환 1회 0.8 Wh  ( 5배)
```

비행 108분은 같은 급 쿼드로터 실제 체공(15~25분)의 5배다 — `hover_power`
과소평가가 거의 확실하다. 그래도 50 을 유지한 이유는 `energy_params.yaml` 과
출처를 하나로 두기 위해서다(SSOT). **INA226 실측 1순위.**

---

## 이전 기록 (2026-08-25 이전)

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
