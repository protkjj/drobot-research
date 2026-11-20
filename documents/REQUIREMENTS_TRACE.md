# 요구사양 추적표 — 무엇이 되어 있고 무엇이 안 되어 있나

**작성** 2026-11-20 · **기준 커밋** `b3f27bf` (eunseo, integration 병합 직후) · 같은 날 '바로 고칠 수 있는 것' 5 건 반영
**기준 문서** 「드로봇 빌드 매니페스트」(2026-09-26) · 「드로봇 인터페이스 규약」(2026-09-26)

사양은 9/26 문서를 그대로 옮겼다. 현재 코드에 맞춰 사양을 고쳐 쓰면 "하기로 했는데 안 된 것"이
표에서 사라지기 때문이다. 사양과 코드가 다르면 둘 중 하나로 정리한다.

- **의도된 변경** → 사양을 고치고 이유를 적는다
- **의도하지 않은 차이** → 코드를 고친다

정리가 안 된 항목은 전부 [5. 결정 필요](#5-결정-필요)에 모았다. 그 목록이 다음 작업 목록이다.

**범례** ✅ 구현·근거 있음 · 🟡 구현, 검증 부분적 또는 사양과 다름(사유 있음) · 🔴 문제 있음 또는 사양 위반 · ⬜ 없음

**이 표의 근거 범위** — 코드·설정은 `b3f27bf` 에서 직접 읽었다. 테스트는 2026-11-20 로컬에서
돌렸다 (기능 테스트 gtest 63 · pytest 26 전부 통과, 실패 106 건은 전부 린트). 시뮬 동작은
새로 돌리지 않았고, 커밋 메시지·HANDOFF·디버깅 기록의 실측을 근거로 적었다. 시뮬이 필요한
항목은 "미검증" 으로 표시했다.

---

## 0. 요약

| 영역 | 상태 | 한 줄 |
|---|---|---|
| 메시지 | ✅ | 규약과 필드 단위로 일치 |
| 2.5D 코스트맵 (A) | 🟡 | 동작은 하나 **패키지에 기능 테스트 0 개**, 전체 격자 검증 없음 |
| TF 프레임 이름 (A) | 🟡 | 규약은 `base_link`, 저장소 전체가 `base_footprint`. **규약을 개정하기로 결정** (D-4) |
| 플래너 (B) | ✅ | gtest 63 개 통과, nogap 에서 전환점 생성 |
| 에너지 모델 | 🟡 | 코드는 됨. 계수는 실측 전 추정값 (두 세트) |
| 컨트롤러 | 🔴 | 계획서는 **MPPI 확정**, 저장소는 처음부터 **DWB**. 사유 기록 없음. **MPPI 를 시도하기로 결정** (D-1) |
| 파라미터 관리 | 🔴 | 런타임은 nav2_params 인라인 값만 읽는다. 패키지 config 와 이미 2 건 어긋남 |
| 모드 전환 실행 (C) | 🟡 | 운동학 재생(set_pose)으로 동작. 실제 비행 없음 |
| 변신 모션 | 🟡 | 동작하나 목표 각도 미도달 |
| 연구 결론 ("비행을 선택하는가") | ⬜ | narrow 월드 실험 미실시 |

---

## 1. 착수 전 구조 항목 (매니페스트 「착수 전」 ①~⑧)

| ID | 사양 | 현재 | 상태 | 근거 / 비고 |
|---|---|---|---|---|
| S-1 | ① 인터페이스 계약 4 개 | 2 절 참조 | 🟡 | A→B·B→C·C→B 는 형식 일치. A→전원(TF) 위반 |
| S-2 | ② 토픽·QoS 표 | 2 절 참조 | ✅ | 규약 토픽 전부 존재 |
| S-3 | ③ TF 트리 `map→odom→base_link→센서` | `base_link` 대신 `base_footprint` | 🟡 | `ekf.yaml` `base_link_frame: base_footprint`, `slam_params.yaml` `base_frame: base_footprint`. **규약을 `base_footprint` 로 개정하기로 결정** — 규약 PDF 갱신 필요 ([D-4](#5-결정-필요)) |
| S-4 | ④ 파라미터 흐름 규칙 — "nav2_params 에는 플러그인 **이름만**, 세부 값은 각 패키지 config 에서 참조" | 세부 값 전부 nav2_params 에 인라인. launch 는 패키지 config 를 하나도 읽지 않는다 | 🔴 | 같은 값이 여러 곳에 손으로 복사돼 있다 — 패키지 config(벤치마크·test_maps.py 가 읽는다), nav2_params 2 개, 그리고 각 파일 안의 local·global 블록. elevation 파라미터는 5 벌이다. 이미 2 건 어긋남 — `fly_over_max` 2.0 vs 1.2, 지역 `fly_over` 200 vs 254. 결정 [D-5](#5-결정-필요) |
| S-5 | ⑤ SSOT — 공유 상수를 `robot_physical.yaml` 한 곳에 | 파일은 있다 (`drobot_description/config/robot_physical.yaml`) | 🟡 | **그 파일을 읽는 코드가 없다.** 다른 파일은 `← SSOT` 주석만 달고 값을 복사해 쓴다. 현재 값은 일치 |
| S-6 | ⑥-1 컨트롤러 DWB vs MPPI — "계획서 = MPPI 확정" | `RotationShimController` + `DWBLocalPlanner` | 🔴 | 첫 시뮬 자산 커밋(`29ee27a`)부터 DWB. MPPI 는 구현된 적이 없고 바꾼 사유도 없다. 전복·코너 교착이 전부 DWB 에서 나왔다. 설정 주석의 "MPPI 가 …" 는 2026-11-20 에 실제(DWB) 로 고쳤다. **MPPI 를 시도하기로 결정** ([D-1](#5-결정-필요)) |
| S-7 | ⑥-2 elevation_layer 를 local 에도 | 넣었다 | ✅ | `updateOrigin()` 구현 후 복귀 (`94c0e41`) |
| S-8 | ⑦ grid_map vs 자체 구현 | 자체 높이 격자 | ✅ | grid_map 의존 없음 |
| S-9 | ⑧ 런치 — sim/real, mapping/localization 분리, use_sim_time | sim + mapping 만. use_sim_time 처리됨 | 🟡 | real·localization 구성 없음. 월드별 출발점(`spawn_positions.yaml`)을 launch 가 읽지 않고 (2.0, 1.0) 고정. nogap·narrow 는 yaml 에 항목도 없다 (같은 출발점이라 지금은 문제없음) |

---

## 2. 인터페이스 규약

### 2-1. TF (규약 ①)

| ID | 사양 | 현재 | 상태 | 비고 |
|---|---|---|---|---|
| T-1 | `map → odom` — slam_toolbox | slam_toolbox | ✅ | |
| T-2 | `odom → base_link` — robot_localization | `odom → base_footprint` | 🟡 | 규약 개정으로 해소 ([D-4](#5-결정-필요)). EKF 주기도 사양 30 Hz → 현재 50 Hz 이고, 50 Hz 를 못 맞춘다 (1 회 251 ms, 2026-10-05). [D-9](#5-결정-필요) |
| T-3 | `base_link → 센서` — URDF 정적 | URDF | ✅ | |
| T-4 | "비행 중 map→odom 정지, 착륙 후 재보정" | mode_manager 가 `/slam_toolbox/pause_new_measurements` + EKF `/set_pose` 호출 | 🟡 | C 트랙 구현. 운동학 재생 기준으로만 확인 |
| T-5 | 계획·costmap·제어는 map 기준 | 전환점 거리 비교를 TF(map) 로 | ✅ | `mode_switch_executor`, `mode_manager` 둘 다 |

### 2-2. 토픽·QoS (규약 ②)

| ID | 토픽 | 상태 | 비고 |
|---|---|---|---|
| Q-1 | `/scan`, `/camera/depth/points`, `/imu`, `/odom`, `/cmd_vel` | ✅ | ros_gz_bridge 에 전부 있음. 추가로 `/camera/*`, `/joint_states`, arm `cmd_pos` 4 개 |
| Q-2 | 센서 토픽은 `sensor_data` QoS | 🟡 | ElevationLayer 구독은 `SensorDataQoS` 확인. 나머지 구독자는 미확인 |
| Q-3 | `/plan` | ✅ | Nav2 |
| Q-4 | `/mode_switch_points` reliable + transient_local | ✅ | 발행(플래너) transient_local, 구독 2 곳(mode_manager `LATCHED`, executor transient_local) |
| Q-5 | `/mode_switch_points` 구독자 = mode_manager | 🟡 | **구독자가 둘이다.** 규약에 없는 `mode_switch_executor`(변신)가 추가됐다. 도달 판정 반경도 다르다 (0.35 m vs 0.9 m). 지금은 둘 다 수동 실행이라 같이 뜬 적이 없다. [D-6](#5-결정-필요) |
| Q-6 | `/battery_state` energy_logger → mode_manager | 🟡 | 코드상 배선됨. INA226 이 없어 실제 값은 흐른 적 없음 |
| Q-7 | `/fmu/*` PX4 | ⬜ | 사용 안 함. px4_msgs 는 체크아웃돼 있으나 빌드 OOM (HANDOFF) |

### 2-3. 메시지

| ID | 사양 | 상태 |
|---|---|---|
| M-1 | `ModeSwitchPoint` — header, position, GROUND_TO_AIR/AIR_TO_GROUND, switch_type, flight_altitude, estimated_energy_cost, pair_id | ✅ 필드 단위 일치 |
| M-2 | `ModeSwitchPlan` — header, switch_points[], total_flight_energy | ✅ 필드 단위 일치 |

### 2-4. 트랙 간 계약 (규약 ③)

| ID | 계약 | 상태 | 비고 |
|---|---|---|---|
| C-1 | A→B cost 값 0 / 100 / 200 / 254 | ✅ | 6 절 표 참조 |
| C-2 | A→B 레이어 이름 `elevation_layer`, 플러그인 `drobot_costmap_2_5d::ElevationLayer` | ✅ | 플래너가 `terrain_layer: "elevation_layer"` 로 읽는다 |
| C-3 | A→B **global·local 둘 다** | ✅ | 10/5 이전에는 local 에서 빠져 있어 위반 상태였다 |
| C-4 | A→B — `NO_INFORMATION`(255) 의 의미 | 🔴 **사양에 없음** | 플래너가 "주행 가능" 으로 낙관한다. 높이 관측률 4.8 % 에서 전환점이 영원히 안 생긴 원인. 지금은 prior map 으로 우회. [D-3](#5-결정-필요) |
| C-5 | A→B — 지역 지도의 `fly_over` 값 | 🔴 **사양에 없음** | default 파일 200, derived 파일 254. 에너지 세트를 바꾸면 지역 지도까지 같이 바뀐다. [D-2](#5-결정-필요) |
| C-6 | B→C ModeSwitchPlan 발행, pair_id 짝 | ✅ | nogap 에서 이륙·착륙 1 쌍 (2026-10-05) |
| C-7 | C→B 에너지 yaml 스키마 (ground_mode, air_mode, climb_mode, mode_switch, idle_power) | ✅ | 키 일치. 값은 실측 전 (E-2) |
| C-8 | A→전원 TF·프레임 이름 | 🟡 | T-2 와 같음 — 규약 개정으로 해소 |

### 2-5. 공통 규약

| ID | 사양 | 상태 | 비고 |
|---|---|---|---|
| U-1 | 단위 m · Wh · W · s · rad (설정의 각도만 deg) | ✅ | 확인한 범위에서 일치 |
| U-2 | map ENU, z 는 지면 기준 절대 고도 | ✅ | |
| U-3 | use_sim_time 전 노드 통일 | ✅ | launch 인자 |
| U-4 | SSOT 0.8 / 0.5 / 5.0 | 🟡 | S-5 와 같음 |

---

## 3. 패키지별 (매니페스트)

### drobot_msgs (B) — ✅
메시지 2 개, rosidl 등록. M-1·M-2.

### drobot_costmap_2_5d (A)

| ID | 사양 | 상태 | 비고 |
|---|---|---|---|
| P-A1 | `ElevationLayer` 플러그인, `costmap_plugins.xml` | ✅ | |
| P-A2 | 높이 4 단계 분류 + 경사·거칠기·단차 + cost 매핑 | ✅ | `classify()` |
| P-A3 | `fly_over_max: 2.0` | 🟡 | 런타임은 **1.2**. 천장 2.5 − 비행고도 0.8 − 여유 0.5 에서 유도한 의도된 변경 — 사양(매니페스트)을 고칠 것. `elevation_params.yaml` 은 2026-11-20 에 1.2 로 맞췄다 |
| P-A4 | 센서 입력 (pointcloud + scan) | ✅ | LiDAR 는 높이를 넣지 않고 `hit_by_scan` 플래그만 남긴다 (`94c0e41`) |
| P-A5 | (추가) rolling window | ✅ | `updateOrigin()`. 실측 기록은 없음 |
| P-A6 | (추가) prior map | ✅ | `use_prior_map:=true`. `clear_on_reset: false` 와 함께 써야 복구 동작에 안 지워진다 |
| P-A7 | (추가) 병합 규칙 — 올리기 자유, 내리기는 카메라 확인 시 200 까지만 | ✅ | |
| P-A8 | 테스트 | 🔴 | **기능 테스트 0 개.** 플래너 쪽 `test_costmap_terrain_contract`(9 개)가 간접 검증할 뿐 |
| P-A9 | 알려진 구조 문제 | 🔴 | `max_z` 만 누적 (나쁜 프레임 영구 잔류), 바닥 노이즈 0.04~0.08 m (여유 0.15 의 1/3), `fly_over: 253` 이면 벽 254 를 253 으로 끌어내림 |

### drobot_hybrid_planner (B)

| ID | 사양 | 상태 | 비고 |
|---|---|---|---|
| P-B1 | energy_model / state_space / planner | ✅ | |
| P-B2 | 테스트 test_energy_model, test_state_space | ✅ | 18 · 25 개 통과 (경로 여유 테스트 7 개 추가). 그 외 contract 9 · nav2_params 7 · path_msg 4 |
| P-B3 | `scripts/test_maps.py` | ✅ | |
| P-B4 | 탐색 파라미터 (timeout 2.0, modal hybrid, flight_clearance 0.8, max_flight_segment 5.0, …) | ✅ | 사양값과 일치. `modal`·`max_climb_angle` 은 nav2_params 에 없어 코드 기본값(hybrid, 45°) 으로 동작 |
| P-B5 | (추가) 경로 여유 `min_ground_clearance 0.30` | ✅ | 팀원 10/2, 충돌 2/3 → 0/3 |
| P-B6 | 계획 시간 ≤ timeout 2 s | 🔴 | medium_open 에서 미로 우회를 2 s 안에 못 찾는다 (팀원 10/2) |
| P-B7 | 로그 `경로 생성: C=… E=… T=…` | 🟡 | C 는 스무딩 후, E·T 는 스무딩 전 경로 값이었다 (예: C 5.4224 인데 E·T 로 계산하면 5.852). 2026-11-20 에 E·T 도 실제로 내보내는 경로 값을 찍게 고쳤다. 시뮬 로그로 재확인 필요 |
| P-B9 | `scripts/test_maps.py` 가 `elevation_params.yaml` 을 읽는다 | 🔴 | **지금 실행이 안 된다.** yaml 최상위 키를 `elevation_costmap_layer` 로 찾는데 실제는 `elevation_layer` 다 (`KeyError`). 월드를 다시 생성하려면 먼저 고쳐야 한다. 고치면 `fly_over_max` 1.2 로 분류가 바뀐다 |
| P-B8 | 착륙 지점 | 🟡 | 장애물 끝(y 7.0) 보다 1.8 m 뒤 (y 8.775). 원인 미확인 |

### 에너지 모델 값 (energy_params)

| ID | 사양 | 상태 | 비고 |
|---|---|---|---|
| E-1 | 가중치 0.5 / 0.2 / 0.3 | ✅ | 990 회 sweep 완료 (RESEARCH_LOG3) |
| E-2 | 모드별 계수 `← 실측` | 🟡 | 실측 없음. default 의 이착륙 5.0 Wh 는 hover_power 로 계산한 값의 72 배. derived 세트를 함께 유지 |
| E-3 | 결론 민감도 | — | 손익분기 우회거리: default 17.1 m, derived 3.3 m (폭 1 m·높이 0.5 m 장애물). 결론은 가중치보다 이착륙 에너지가 정한다 |

### drobot_mode_manager (C)

| ID | 사양 | 상태 | 비고 |
|---|---|---|---|
| P-C1 | 구독 → 정지 확인 → **PX4 이착륙** → SLAM 재개 | 🟡 | PX4 대신 `backend: sim` (Gazebo set_pose 운동학 재생). "비행 시뮬레이션" 이 아니다 (backends.py 경고) |
| P-C2 | mode_switch_params 값 | ✅ | 사양 6 개 값 일치 |
| P-C3 | 테스트 | ✅ | pytest 26 개 통과 |
| P-C4 | 전 사이클 | 🟡 | GROUND→TAKING_OFF→FLYING→LANDING→GROUND, 착륙 오차 17 cm (C 트랙 기록) |

### drobot_energy_model (C)

| ID | 사양 | 상태 | 비고 |
|---|---|---|---|
| P-E1 | `energy_logger.py` INA226 100 Hz 로깅 | 🟡 | 코드 있음. 하드웨어가 없어 실제 측정은 없다 |

### drobot_description (A)

| ID | 사양 | 상태 | 비고 |
|---|---|---|---|
| P-D1 | URDF·meshes·worlds·rviz | ✅ | nogap·narrow 월드 추가 (10/5) |
| P-D2 | CMake install | 🟡 | `maps/` 가 설치 목록에 없었다 (수정이 stash 에만 있었다). 2026-11-20 에 커밋했다. 깨끗한 클론에서 재확인 필요 (AC-10) |
| P-D3 | (비행에 필요) 프로펠러 링크·추력 플러그인 | ⬜ | 링크 16 개에 rotor/prop 없음 |

### drobot_bringup (공동·B)

| ID | 사양 | 상태 | 비고 |
|---|---|---|---|
| P-G1 | `navigation.launch.py`, `start_goal_markers.py` | ✅ | |
| P-G2 | 컨트롤러 MPPI, `vx_max 0.4` | 🔴 | DWB. S-6 |
| P-G3 | local `["obstacle_layer","elevation_layer","inflation_layer"]` | 🟡 | `obstacle_layer` 대신 `voxel_layer` |
| P-G4 | global `["static_layer","elevation_layer","inflation_layer"]` | 🟡 | `static_layer` 제거 — 미탐색 영역이 costmap 밖이 돼 목표를 못 찍는 문제 (주석에 사유). 의도된 변경 |
| P-G5 | EKF frequency 30, base_link_frame base_link | 🔴 | 50 Hz, base_footprint. T-2 |
| P-G6 | SLAM max_laser_range 8.0, loop_closure true | 🟡 | 12.0, `do_loop_closing: false`. 사유 미확인 — [D-8](#5-결정-필요) |
| P-G7 | ros_gz_bridge | ✅ | Q-1 |
| P-G8 | spawn_positions.yaml | 🟡 | 파일은 있으나 launch 가 안 읽는다. S-9 |
| P-G9 | BT 복구에서 global costmap 통째 삭제 금지 | ✅ | (사양 외) 10/5 수정, 병합 후 유지 확인 |

### drobot_experiments (C)

| ID | 사양 | 상태 | 비고 |
|---|---|---|---|
| P-X1 | `record_run.py`, `publish_paths.py`, `logging_config.yaml` | ✅ | `trial_summary.py` 추가. `record_run --goal` (10/2) |

### drobot_controller (공동·유틸)

| ID | 사양 | 상태 | 비고 |
|---|---|---|---|
| P-U1 | `teleop_keyboard.py` | ✅ | |
| P-U2 | (사양 외) `transform_manager` | 🟡 | arm 4 개 동시 구동. 목표 1.5708 rad 미도달 (잔차 0.15~0.30 rad, 마찰과 P 게인의 평형점과 일치). 테스트 0 개 |
| P-U3 | (사양 외) `mode_switch_executor` | 🟡 | nogap 에서 이륙점 0.18 m 앞 호출 확인. 기능 테스트 0 개. 린트는 2026-11-20 에 정리 (controller 패키지 린트 테스트 전부 통과) |

### px4_msgs · px4-ros2-interface-lib (C) — ⬜
체크아웃돼 있으나 쓰지 않는다. 빌드 OOM.

---

## 4. 동작 기준 (수용 기준) — 초안

매니페스트는 "어떤 파일에 어떤 값" 까지만 정하고 "제대로 동작한다" 의 기준이 없다. 아래는 그 빈칸을
채우는 초안이다. **`__` 는 팀이 정할 값이다.**

| ID | 기준 | 방법 | 현재 |
|---|---|---|---|
| AC-1 | prior map 에서 코스트맵 지형 등급이 정답 높이맵 등급과 칸 단위 일치율 ≥ `__` % | `/global_costmap/elevation_layer_raw` 덤프 ↔ 정답 높이맵에 같은 `classify()` 규칙 적용 | ⬜ 미측정 |
| AC-2 | base_map_h0.5 에서 박스에 올라타지 않음 (전복 0 회 / `__` 회) | 주행 + bag | 🔴 default(지역 200): 전복 실측. derived(지역 254): 통과 (10/5) |
| AC-3 | nogap 에서 전환점 1 쌍 생성, 이륙점 반경 안에서 변신 호출 | 런치 로그 + `/mode_switch_points` | ✅ derived + prior map (10/5) |
| AC-4 | narrow 에서 에너지 세트별 비행 선택 여부가 오프라인 벤치마크와 같다 | 계획만 (주행 완주 불필요) | ⬜ |
| AC-5 | 도착 오차 ≤ 0.30 m | record_run | 🟡 팀원 10/2 측정. 병합 후 재측정 안 함 |
| AC-6 | 계획 시간 ≤ 2.0 s (모든 벤치마크 월드) | 런치 로그 | 🔴 medium_open 초과 |
| AC-7 | 코너 통과 — 교착 없이 `__` s 안에 | bag | 🔴 미해결 (`No valid trajectories out of 819`) |
| AC-8 | 변신 각도 오차 ≤ `__` rad | `/joint_states` | 🔴 잔차 0.15~0.30 rad |
| AC-9 | 로그·기록의 에너지 = 실제 주행 경로의 에너지 | C 를 E·T 로 재계산 | 🟡 코드 수정함, 시뮬 로그로 재확인 필요 |
| AC-10 | 새로 클론해 빌드해도 prior map 이 로드된다 | 깨끗한 클론 | 🟡 설치 규칙 수정함, 깨끗한 클론에서 재확인 필요 |
| AC-11 | 기능 테스트 전부 통과 | `colcon test` | ✅ 2026-11-20 |

---

## 5. 결정 필요

상태: **결정됨** · **보류** (논의 중) · **미결**

| ID | 무엇을 | 상태 | 내용 |
|---|---|---|---|
| D-1 | 컨트롤러 | **결정됨 (2026-11-20)** | **MPPI 를 시도한다.** `nav2_mppi_controller` 는 ROS Jazzy 에 이미 설치돼 있다(로컬 확인, 데탑 컨테이너는 미확인) — 새로 짜는 것이 아니라 설정·critic 튜닝 작업이다. DWB 와 같은 월드에서 전복(AC-2)·코너(AC-7)를 비교한다 |
| D-2 | 지역 `fly_over` 값 | **보류** | **필수는 두 파일을 같게 만드는 것이다** — 지금은 default 200, derived 254 라 에너지 세트를 바꾸면 지역 지도까지 바뀐다. 값은 논의 중: (a) 254 — 지역 지도는 지상 컨트롤러만 읽으므로 지금 잃는 정보가 없다 (b) 200 유지 — 의미를 지도에 남기고 컨트롤러가 모드에 따라 해석. 단 DWB 는 200 을 '조금 비싼 바닥' 으로만 읽어 밟는다 (전복 실측). D-1(MPPI) 결과를 보고 정해도 된다 |
| D-3 | `NO_INFORMATION` 해석 | 미결 | 권장: 지상 통과는 낙관, 비행 선택("날아 넘을 수 있다") 은 비관. prior map 은 우회일 뿐이다 |
| D-4 | 프레임 이름 | **결정됨 (2026-11-20)** | **규약을 `base_footprint` 로 개정한다.** 모든 노드가 이미 `base_footprint` 를 쓰고 REP-120 에도 맞다. 할 일: 인터페이스 규약 PDF ①·③ 갱신, 매니페스트 ekf·slam 예시 갱신, `mode_switch_params.yaml` 의 "규약 정렬은 A 트랙 몫" 주석 정리 |
| D-5 | 파라미터 원천 | 미결 | 아래 설명 |
| D-6 | 전환점 도달 판정 주체 | 미결 | 아래 설명 |
| D-7 | 수용 기준의 `__` 값 | 미결 | 팀 |
| D-8 | SLAM loop closure 끔, 레이저 범위 12 m | 미결 | 사유 확인 |
| D-9 | EKF 50 Hz (사양 30, 실측 미달) | 미결 | 30 시험. TF 지연이 의심된 적 있으나 원인은 아니었다 (반증된 가설) |

### D-5 파라미터 원천 — 무엇을 정해야 하나

지금 같은 값이 여러 벌 있다. 런타임(Gazebo)은 nav2_params 만 읽고, 오프라인 벤치마크는 패키지
config(`energy_params*.yaml`)를 읽는다. **이 연구는 오프라인 결과와 Gazebo 결과를 비교하므로(AC-4),
두 쪽이 같은 숫자를 읽는다는 보장이 필요하다.** 지금은 손으로 맞춰서 우연히 같을 뿐이다.

| 안 | 내용 | 장점 | 단점 |
|---|---|---|---|
| (a) 패키지 config 가 원천 | launch 가 `elevation_params.yaml`·`energy_params.yaml` 을 읽어 노드에 넘긴다 | 매니페스트 원칙 그대로. 벤치마크와 런타임이 같은 파일을 읽는다 | launch 작업 필요. 노드 이름 아래로 키를 옮겨야 한다 (`global_costmap.elevation_layer.*` 등). local·global 두 곳에 같은 블록을 넘겨야 한다 |
| (b) nav2_params 가 원천 | 패키지 config 는 지우거나 설명만 남긴다. derived 는 에너지 블록만 담은 덮어쓰기 파일로 | Nav2 의 보통 방식이라 단순하다. launch 를 거의 안 바꾼다 | 벤치마크가 nav2_params 를 읽도록 고쳐야 한다. 안 고치면 다시 어긋난다 |
| (c) 지금 구조 + 일치 테스트 | 파일은 그대로 두고, 값이 다르면 실패하는 테스트를 추가 | 가장 적은 작업. 어긋나면 바로 안다 | 복사본이 계속 남는다 |

권장: **(c) 를 먼저** 넣어 더 어긋나지 않게 막고, (a)/(b) 는 MPPI 작업(D-1) 이 끝난 뒤 정한다.
MPPI 로 바꾸면 컨트롤러 블록이 통째로 바뀌므로 그때 함께 정리하는 편이 덜 번거롭다.

### D-6 전환점 도달 판정 — 두 노드의 차이

둘 다 `/mode_switch_points` 를 받아 "로봇이 전환점에 왔나" 를 판정한다. 판정 방식과 판정 뒤에 하는 일이 다르다.

| | `mode_manager` (C, `switch_tracker.py`) | `mode_switch_executor` (A) |
|---|---|---|
| 도달 반경 | 0.35 m (주행 해상도의 3~4 배) | 0.9 m (footprint 길이의 약 2 배) |
| 순서 | 전환점을 **순서대로** 하나씩. 상태(GROUND/FLYING)가 안 맞으면 건너뛰고 기록 | 순서 없음. 반경 안에 들어온 점은 아무거나. `(pair_id, switch_type)` 로 중복만 막음 |
| 정지 확인 | 속도 ≤ 0.05 m/s 인지 **기록만** 한다 (막지는 않음) | 없음 |
| 배터리 | 이륙 전에 잔량 검사 (착륙에는 안 건다) | 없음 |
| 착륙 후 쿨다운 | 3 s | 없음 |
| 도달 뒤 하는 일 | 이륙 → 비행 구간 운동학 재생(set_pose) → 착륙, SLAM 정지·재개, EKF 재설정, 속도 제한 | `/transform_to_drone` · `/transform_to_rover` 호출 (암 회전) |

정리하면 **mode_manager 는 비행 상태기계, executor 는 변신 모션 트리거** 다. 둘을 같이 띄우면 같은
이륙점에서 0.9 m 에 변신이, 0.35 m 에 이륙이 따로 걸린다. 순서도 보장되지 않는다
(변신이 끝나기 전에 이륙할 수 있다).

권장: 판정은 **mode_manager 하나로** 한다 — 순서·상태·배터리 검사가 이미 있다. executor 의 변신 호출은
mode_manager 의 이륙 직전·착륙 직후 단계로 옮기고, executor 는 비행이 없는 테스트용으로만 남긴다.
HANDOFF 의 호출 순서(도달 판정 → `/transform_to_drone` → 비행 → `/transform_to_rover`)와도 맞는다.

### 2026-11-20 반영한 것 (결정 불필요 5 건)

| 항목 | 한 일 |
|---|---|
| P-D2 | stash 에만 있던 `CMakeLists.txt` 한 줄(`maps` 설치)과 `gazebo.xacro` PID 주석 커밋 |
| P-B7 | 로그의 E·T 가 실제로 내보내는 경로(스무딩 채택 시 스무딩 경로)의 값을 찍게 |
| P-A3 | `elevation_params.yaml` 의 `fly_over_max` 2.0 → 1.2 |
| S-6 | nav2_params 두 파일의 "MPPI 가 …" 주석을 실제(DWB) 로, elevation_params 가 런타임에 안 읽힌다는 점 명시 |
| P-U3 | `mode_switch_executor.py` import 순서, 두 파일 docstring 형식, `teleop_keyboard.py` 줄 끝 공백 — controller 린트 테스트 전부 통과 |

### 새로 나온 바로 고칠 수 있는 것

| 항목 | 할 일 |
|---|---|
| P-B9 | `test_maps.py` 의 yaml 키 `elevation_costmap_layer` → `elevation_layer` (지금 실행 불가) |

---

## 6. 참고 — 높이 비용 값

**ElevationLayer 가 내는 값** (`classify()`)

| 값 | 의미 | 언제 |
|---|---|---|
| 0 | free | 높이 ≤ 0.15 m, 경사·거칠기·단차가 한계의 30 % 미만 |
| 100 | rover_traversable | 높이 ≤ 0.15 m, 한계 안이지만 평탄하지 않음 |
| 200 | fly_over | 0.15 m < 높이 ≤ 1.2 m, 또는 낮지만 경사·거칠기·단차 초과 |
| 254 | impassable | 높이 > 1.2 m(천장 제약), 또는 LiDAR 반사가 있는데 카메라 높이 > 0.6 m (가까운 큰 물체 과소추정 방어) |
| 255 | 말하지 않음 | 카메라가 높이를 재지 않은 칸. master 를 건드리지 않는다 |

**master 에 다른 레이어가 쓰는 값**

| 값 | 누가 |
|---|---|
| 254 | obstacle_layer(전역) / voxel_layer(지역) |
| 253 | inflation_layer — 장애물 바로 옆(내접 반경) |
| 1~252 | inflation_layer — 거리에 따라 감소. 그래서 master 에서는 100·200 이 지형인지 inflation 인지 구분할 수 없다 |

**병합 규칙** — ElevationLayer 는 master 비용을 언제든 올릴 수 있다. 내리는 것은 카메라가 높이를 잰
칸에서만, 그리고 200 아래로는 절대 안 된다. 2.5D 의 핵심 기능(넘을 수 있는 장애물을 254 → 200 으로
내려 비행 경로를 여는 것)은 이 경로로 동작한다.

**플래너가 읽는 방식** (`LayerTerrainSource`)

| 무엇 | 어느 격자 | 판정 |
|---|---|---|
| 지형 등급 | ElevationLayer 자체 격자 | ≤ 150 주행 가능 · ≤ 253 비행 필요 · 254 통과 불가 · 255 주행 가능(낙관), 착륙 불가 |
| 충돌 | master | 254, 그리고 253 (출발점 주변은 완화) |

**`/…/costmap` 토픽을 볼 때 주의** — OccupancyGrid 라 0~100 으로 압축된다. 100 → 39, 200 → 77,
253 → 99, 254 → 100. 등급을 보려면 `/…/costmap_raw` 나 `/…/elevation_layer_raw` 를 읽을 것
(`tools/costmap_probe.py`).
