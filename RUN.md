# 시뮬레이션 실행 안내

브랜치 `integration` 기준. 2026-10-02 갱신.

처음 돌리는 사람은 **4. 함정**을 먼저 읽을 것. 오늘 하루를 거기에 썼다.

---

## 0. 준비

```bash
cd ~/Desktop/drobot-research
git fetch origin
git checkout integration
git pull origin integration
```

컨테이너는 `sync.sh` 가 알아서 띄운다. 직접 확인하려면:

```bash
docker ps --filter name=drobot_ros2
```

---

## 1. 빌드

```bash
./sync.sh stop      # 시뮬이 떠 있으면 먼저 끈다 (빌드와 동시 실행 금지)
./sync.sh build
```

**`Summary: 8 packages finished` 를 확인할 것.** 8개가 아니면 반영 안 된 패키지가 있다.

---

## 2. 시뮬레이션 기동

```bash
DROBOT_ENERGY=derived ./sync.sh sim base_map_h0.5 proposed
```

인자는 **위치 기반**이다. 플래그로 주면 안 된다.

```
./sync.sh sim <world> <planner> [gui]
  world    base_map_h0.05 | h0.3 | h0.5 | h1 | h1.8 | h2.5
           easy_open | easy_corridor | medium_open | medium_corridor | hard_open | hard_corridor
  planner  proposed (하이브리드 A*) | smac2d (지상 전용 베이스라인)
  gui      붙이면 Gazebo GUI 도 켠다 — 원격 작업 중이면 쓰지 말 것 (4-5 참고)

에너지 세트는 환경변수다:  DROBOT_ENERGY=default | derived
Gazebo 디버그 로그:        DROBOT_GZ_VERBOSE=1
```

**40초 기다린 뒤** 반드시 확인한다. 이걸 건너뛰면 "안 고쳐졌다" 는 잘못된 결론으로 샌다.

```bash
docker exec drobot_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash && source /app/install/setup.bash
  echo -n "gz 토픽 "; timeout 6 gz topic -l 2>/dev/null | wc -l
  timeout 8 ros2 topic echo /clock --once >/dev/null 2>&1; echo "clock: $?"
  timeout 8 ros2 topic echo /scan  --once --field header >/dev/null 2>&1; echo "scan:  $?"
  ros2 lifecycle get /bt_navigator
  ros2 topic info /clock | grep Publisher
'
```

기대값:

```
gz 토픽 32
clock: 0
scan:  0
active [3]
Publisher count: 1
```

`ros2 action list` 로 `/navigate_to_pose` 가 보이는지만 보면 안 된다 — bt_navigator 가
inactive 여도 액션 이름은 보인다. 그 상태면 RViz 에서 목표를 찍어도
`Action server is inactive. Rejecting the goal.` 로 조용히 거부된다.
`inactive [2]` 면 Nav2 기동이 중간에 실패한 것이다. 고치려 하지 말고 `./sync.sh stop` 후 다시 띄운다.

`gz 토픽 0` 이면 Gazebo 가 멈춘 것이다. 4-1 참고.
`Publisher count` 가 2 이상이면 시뮬이 두 개 떠 있는 것이다. 4-8 참고.

---

## 3. 실험 실행 (C 트랙)

`DROBOT_ENERGY=derived ./sync.sh run base_map_h0.5` 가 아래 셋(시뮬 + mode_manager + record_run)을
한 번에 한다. Nav2 가 active 가 될 때까지 기다리고, 끝나면 결과를 `benchmark/results/` 로 가져오고 끈다.
화면으로 보면서 하려면 아래처럼 터미널을 나눠 직접 띄운다.

### 터미널 2 — 모드 전환 관리자

```bash
docker exec -it drobot_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash && source /app/install/setup.bash
  ros2 run drobot_mode_manager mode_manager --ros-args \
    --params-file /app/src/drobot_mode_manager/config/mode_switch_params.yaml \
    -p use_sim_time:=true -p world:=base_map_h0.5
'
```

`-p world:=` 를 **월드 이름과 맞출 것**. Gazebo set_pose 서비스 경로에 들어간다.

정상 기동:

```
모드 관리자 시작 — 백엔드 sim · 도달반경 0.35 m · 정지판정 0.05 m/s
서비스 이름 /world/base_map_h0.5/set_pose 를 쓸 수 없다 — gz CLI 로 간다
백엔드 gazebo:hop — 월드 base_map_h0.5 · 모델 drobot
```

CLI 로 떨어지는 건 **정상**이다. 월드 이름에 `.` 이 있으면 ROS 서비스 이름 규칙에
어긋나서 CLI 를 쓴다.

### 터미널 3 — 목표 전송 + 기록

```bash
docker exec -it drobot_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash && source /app/install/setup.bash
  ros2 run drobot_experiments record_run --world base_map_h0.5 \
    --out /app/benchmark/results/sim_run.json
'
```

최대 180초. 끝나면 요약이 나온다.

```
계획 경로 N점 · 재계획 N회
주행 궤적 N점 · N초
속도 명령 N개 · 최대 전진 X m/s · 회전만 Y% · 정지 Z%
전환 계획 N점 (계획 N건 중 N건에 전환점 있음)
실제 수행 이륙 N회 · 상태 전이 GROUND -> TAKING_OFF -> ...
결과 succeeded / aborted / timeout
요약 추가 -> /app/data/trial_summary.csv
```

**계획과 실행은 다르다.** `전환 계획` 은 플래너가 낸 것, `실제 수행` 은
mode_manager 가 한 것이다. 둘이 어긋나면 그 자체가 결과다.

### 종료

```bash
./sync.sh stop
```

---

## 4. 함정 (전부 실제로 겪은 것)

### 4-1. Gazebo 가 조용히 멈춘다

증상: `gz 토픽 0`, `/clock` 안 나옴, Nav2 는 정상인데 아무것도 안 움직임.
로그 마지막 줄이 `Loading plugin [gz-rendering-ogre]`.

원인 두 가지를 찾아 고쳤다.

- **렌더 엔진 — 판단이 틀렸었다 (2026-10-02 정정)**: 처음엔 `ogre2` 가 RTX 5070 Ti 에서
  멈춘다고 보고 월드 12개를 `ogre`(v1) 로 바꿨다. 그런데 아래 `-u` 문제를 고친 뒤 다시 재니
  `ogre2` 가 3/3 정상이었다. 두 원인을 동시에 고쳐서 섞어 판단한 것이다.
  지금은 launch 가 `render_engine:=ogre2` 를 기본으로 월드 사본에 덮어쓴다 (track-b).
  `ogre`(v1) 는 RTX 4070 SUPER 에서 라이다가 모든 빔에 0.5 m 를 내 쓸 수 없었다.
- **실행 사용자 — 이것도 지금은 재현되지 않는다 (2026-10-02 정정)**: 처음엔 `-u` 로 띄우면
  멈추고 root 면 된다고 봤다. ogre2 기준으로 `-u` 도 3/3 정상이었다.
  그래도 시뮬은 root 로 띄운다 — 붙는 도구(mode_manager·record_run·확인 명령)가 전부
  root 라서 같은 사용자여야 하기 때문이다 (4-9). **빌드는 `-u` 를 유지한다**
  (산출물이 root 소유가 되면 호스트에서 못 지운다).

재발하면 진단 스크립트를 쓴다.

```bash
docker exec drobot_ros2 bash /app/tools/diag_gz.sh world   # 월드만
docker exec drobot_ros2 bash /app/tools/diag_gz.sh file    # + 로봇(-file)
docker exec drobot_ros2 bash /app/tools/diag_gz.sh topic   # + 로봇(-topic)
```

**이미 제거한 가설 — 다시 밟지 말 것**: LD_LIBRARY_PATH, GZ_SIM_RESOURCE_PATH,
HOME, 브리지와 카메라 구독, WorldControl unpause, 스폰 타이밍, 스폰 방식,
UID 그룹과 /dev/dri 권한, 컨테이너·GPU 재시작, 노드 기동 지연.

### 4-2. 고친 게 반영 안 된 채로 시험하게 된다

오늘 세 번 당했다. 매번 "고쳤는데 안 된다" 로 한참을 돌았다.

- `PKGS` 에 패키지가 빠져 있어 설치본이 안 바뀜 → `drobot_bringup`·`description` 추가함
- 데탑이 옛 브랜치에 있었음 → **`git pull` 먼저**
- `worlds/` 가 rsync 제외였음 → 제외를 풀고 LFS 포인터 검사로 대체함

**바꾼 걸 시험하기 전에 설치본에 들어갔는지 확인하는 습관을 들일 것.**

```bash
grep -m1 render_engine ~/Desktop/drobot-research/sim.log   # [INFO] render_engine=ogre2 -> ...
```

월드 파일에는 아직 `ogre` 가 적혀 있지만 launch 가 사본을 만들어 덮어쓴다.
그래서 설치본 월드를 grep 하면 안 되고, 실제로 쓴 값은 `sim.log` 에서 본다.

### 4-3. 파이프에 물리면 출력이 안 보인다

`ros2 topic hz`, `ros2 topic echo`, `gz sim` 을 `| head` 나 `| grep` 에 물리면
블록 버퍼링 때문에 `timeout` 으로 죽일 때 출력이 통째로 사라진다.
**파일로 받고 나서 읽을 것.**

```bash
timeout 6 ros2 topic hz /clock > /tmp/hz.txt 2>&1; cat /tmp/hz.txt
```

`--once` 는 정상 종료하므로 파이프에 물려도 된다.

### 4-4. pkill 이 자기 자신을 죽인다

`pkill -f "gz sim"` 은 명령줄에 "gz sim" 이 들어 있는 **자기 셸까지** 죽인다.
대괄호를 쓴다: `pkill -f "gz[ ]sim"`.

### 4-5. GUI 는 원격에서 쓰지 말 것

Gazebo GUI + RViz 가 X 서버와 GPU 를 점유해 원격 데스크톱 입력이 먹통이 되고
SSH 까지 끊긴 전례가 두 번 있다. 데탑 앞에 직접 앉아 있을 때만 쓴다.

이미 떠 있는 headless 서버에 GUI 만 붙이려면 (서버는 그대로 산다):

```bash
docker exec -it -e DISPLAY=:0 drobot_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash
  ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim -g --force-version 8
'
```

---

## 4-6. 화면으로 보기 (Gazebo / RViz)

둘 다 **이미 떠 있는 시뮬에 붙는** 방식이다. 시뮬을 다시 띄울 필요 없고,
창을 닫아도 서버는 계속 돈다. 실패하면 Ctrl+C 로 빠지면 된다.

⚠ **원격 작업 중이면 쓰지 말 것** (4-5 참고). 데탑 앞에 앉아 있을 때만.

### RViz — 로봇·맵·경로·TF (이쪽을 먼저 권함)

```bash
docker exec -it -e DISPLAY=:0 drobot_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash && source /app/install/setup.bash
  rviz2 -d /app/src/drobot_bringup/config/navigation/display.rviz --ros-args -p use_sim_time:=true
'
```

보이는 것: RobotModel · LaserScan · Map · Path · TF · MarkerArray(시작/목표)

`use_sim_time:=true` 를 빼면 안 된다. TF 는 시뮬 시각으로 찍히는데 RViz 가 벽시계로
찾아서 로봇 모델이 '변환 없음' 으로 나온다 (런치는 이 값을 넘기지만 따로 붙일 때는 직접 줘야 한다).

경로 비교만 볼 때는 전용 설정이 있다.

```bash
  rviz2 -d /app/src/drobot_bringup/config/navigation/paths_view.rviz --ros-args -p use_sim_time:=true
```

### Gazebo GUI — 3D 월드

```bash
docker exec -it -e DISPLAY=:0 drobot_ros2 bash -lc '
  source /opt/ros/jazzy/setup.bash
  ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim -g --force-version 8
'
```

`-g` 는 **GUI 만** 띄워서 돌고 있는 서버에 붙는다.

창이 안 뜨거나 멈추면 GUI 쪽 렌더 엔진이 `ogre2` 라서일 수 있다
(센서용은 월드에서 `ogre` 로 고쳤지만 GUI 는 별도 설정을 쓴다).

```bash
  ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim -g --render-engine ogre --force-version 8
```

### 화면이 안 뜰 때

```bash
echo $DISPLAY                       # 호스트의 디스플레이 번호 확인
xhost +local:docker                 # 컨테이너에 X 접근 허용 (호스트에서)
```

`DISPLAY` 가 `:0` 이 아니면 위 명령의 `-e DISPLAY=:0` 을 그 값으로 바꾼다.

### 멈췄을 때 복구

```bash
./sync.sh stop        # SSH 로 들어가서
```

---

## 4-7. 높이나 cost 가 이상할 때

2026-10-02 에 빈 바닥이 0.25 m 로 보여 하루를 썼다. 원인은 URDF 의
`base_footprint` 높이였다 (HANDOFF 2절).

**URDF 를 바꿨으면** 맥에서 바로 확인한다 (ROS 불필요). `OK` 가 나와야 한다.

```bash
python3 tools/urdf_ground_check.py
```

**센서 점이 전역 어디에 떨어지는지** — 빈 바닥은 z ≈ 0 이어야 한다.

```bash
python3 /app/tools/cloud_probe.py --frame map --min-z -0.5 --max-z 3.0
```

**costmap 값을 읽을 때** — `/global_costmap/costmap` 토픽은 쓰지 말 것.
Nav2 가 0~255 를 0~100 으로 환산해서 rover(100)=39, fly_over(200)=77, LETHAL=100 으로
보이고 inflation 과 구분이 안 된다. 프로브는 원본(`costmap_raw`)을 읽는다.

```bash
python3 /app/tools/costmap_probe.py --x0 2.12 --y0 1.5 --x1 2.12 --y1 5.0              # master — 플래너는 충돌만 본다
python3 /app/tools/costmap_probe.py --topic /global_costmap/elevation_layer_raw \
  --x0 2.12 --y0 1.5 --x1 2.12 --y1 5.0                                                # 플래너가 읽는 지형 등급
python3 /app/tools/costmap_probe.py --mode elevation --x0 2.12 --y0 1.5 --x1 2.12 --y1 5.0   # 높이 (m)
```

플래너는 지형을 `elevation_layer_raw` 에서, 충돌(253·254)만 master 에서 읽는다.
같은 칸이 master 에서는 `free` 인데 지형 격자에서는 `미관측` 일 수 있다 —
장애물 윗면이 그렇다. 플래너는 그런 칸으로 주행은 하지만 착륙은 하지 않는다.

`elevation_grid` 는 cost 가 아니라 **높이**다 (`--mode elevation`).

---

## 4-8. 시뮬을 두 개 띄우면 시간이 거꾸로 간다

2026-10-02 에 겪었다. 헤드리스 시뮬(`sync.sh sim`)이 떠 있는 채로 GUI 런치를 하나 더
띄웠더니 Gazebo 서버와 브리지가 둘씩 생겨 `/clock` 이 두 값을 오갔다.

증상: 모든 노드가 `Detected jump back in time. Clearing TF buffer.` 를 초당 수백 번
낸다. RViz 는 그때마다 리셋돼 **로봇 모델이 처음엔 보이다가 사라진다.**
EKF·SLAM·Nav2 도 두 벌이라 무엇을 재든 믿을 수 없다.
기동 중에 겹치면 Nav2 기동이 실패한다 (`Failed to bring up all requested nodes.
Aborting bringup.`). bt_navigator 가 inactive 로 남아 **RViz 의 Goal Pose 가 먹지 않는다.**
나중에 두 번째 런치를 꺼도 이 상태는 돌아오지 않는다 — 다시 띄워야 한다.

`sync.sh sim` 끼리는 안 겹친다 (`stop_sim` 이 먼저 다 죽인다). 겹친 건 `sync.sh` 를 거치지
않고 `ros2 launch` 를 직접 띄웠을 때다. 런치는 `sync.sh` 로만 띄운다.

확인:

```bash
docker exec drobot_ros2 bash -lc 'source /opt/ros/jazzy/setup.bash; ros2 topic info /clock | grep Publisher'
# Publisher count: 1 이어야 한다
docker exec drobot_ros2 bash -c 'ps -eo pid,lstart,cmd | grep "[g]z sim"'   # 한 줄이어야 한다
```

화면이 필요하면 **새로 런치하지 말고 떠 있는 시뮬에 붙는다** (4-6). 처음부터 GUI 로
띄우려면 `./sync.sh stop` 으로 끈 다음 `./sync.sh sim <world> <planner> gui`.

---

## 4-9. ROS 노드는 같은 사용자로 띄운다

2026-10-02 에 겪었다. 시뮬(root) 에 record_run 을 `-u`(사용자)로 붙였더니 액션 서버는
'발견' 했지만 토픽을 하나도 못 받았고, 보낸 목표는 bt_navigator 에 닿지도 않았다
(bt_navigator 로그에 수신 기록 없음). 반대로 사용자로 띄운 시뮬에 root 쪽 `ros2 lifecycle get`
은 빈 응답이었다. DDS(공유메모리) 가 다른 사용자끼리 통신하지 못한 것으로 본다.

규칙: 시뮬·mode_manager·record_run·확인 명령은 전부 `docker exec drobot_ros2 ...`
(= root) 로. `-u` 는 빌드에만 쓴다. 결과 파일은 끝나고 `chown` 한다 (`sync.sh run` 이 그렇게 한다).

---

## 5. 지금 알려진 문제

| 트랙 | 문제 | 상태 |
|---|---|---|
| A | 빈 바닥이 0.25 m 로 측정돼 fly_over 로 분류됨 — URDF `base_footprint_joint` z=0.25 가 원인 (gz 실측 z=-0.2518) | 수정, 시뮬 검증 완료 |
| 공동 | 카메라(실제 0.36 m)가 0.5 m 장애물 윗면·뒤를 못 봄 → 착륙만 관측 칸으로 제한했다 | **대가: 카메라보다 높은 장애물 뒤로는 날아서 착륙할 수 없다.** 비행이 거의 안 나올 수 있음 — 팀 결정 필요 (HANDOFF 2-2) |
| B | 플래너가 inflation 값을 지형 등급으로 읽음 — 벽에서 0.45 m 이내가 fly_over 로 읽힘 | 수정, 테스트·시뮬 확인 |
| B | 출발 칸이 벽 옆(253)이면 계획 실패 (위 수정의 부작용) | track-b 에서 출발점 0.35 m 안 253 완화. 테스트 없음 |
| B | 컨트롤러에 듬성한 꺾임점 + yaw 0 경로를 넘겨 로봇이 좌우로 흔들림 | track-b 에서 0.05 m 간격·진행 방향 yaw 로 수정 (회전만 76% -> 7%, B 측정) |
| A | ElevationLayer 가 rolling window 를 지원하지 않아 local costmap 에 높이가 엉뚱하게 쌓임 | track-b 에서 local costmap 에서 제외 (회피). `updateOrigin` 구현이 근본 해결 |
| B | derived 에서 출발→목표 19.4 m 를 한 번에 날았다 / 휴리스틱이 admissible 하지 않았다 | track-b 에서 비행 1구간 5 m 제약 + 휴리스틱 하한 수정 |
| B | 경로가 상자·벽에 붙어 로봇이 상자 모서리에 박혔다 (3회 중 2회) | **수정** (`fix/path-clearance`) — 3회 모두 충돌 없음, 상자까지 0.65 m 이상 |
| C | 비행 착륙 직후 aborted (1회) | 원인 미확인 — 로그를 남기며 재현 필요 |
| 공동 | `xy_goal_tolerance: 0.75` 라 목표 0.75 m 앞에서 succeeded | 플랫폼 원본 값. 지표로 쓰려면 조정 결정 |
| A | 목표 표시 원판(goal_marker)이 지형으로 읽혀 목표 칸이 막힌다 → 목표가 0.3~0.4 m 옮겨짐 | 월드 쪽 수정 필요 |
| B | 플래너가 21 cm 구간에 이착륙을 건다 (전환 1회 11.2 Wh) | 바닥 수정 후 재측정 필요 |
| B | 재계획이 잦고 전환점이 매 계획마다 튄다 | 바닥 수정 후 재측정 필요 |
| B | 착륙점이 장애물 한가운데로 잡힌다 | 원인은 미관측 → free. 위 착륙 제한으로 대응, 재측정 필요 |
| 공동 | 설정 파일의 에너지 값 31개가 전부 버려져 `DROBOT_ENERGY=derived` 가 적용되지 않았다 (이전 derived 기록은 default 로 계획된 것) | 수정 (`fix/floor-offset`) — derived 결과는 다시 낼 것 |
| B | `scripts/test_maps.py` 린터 실패 (flake8 94 · pep257 2, 대부분 작은따옴표 규칙) | 동작 무관, 미수정 |
| A | 렌더 엔진 — RTX 5070 Ti 에서 ogre2 가 멈춘다던 판단 | 틀렸음. ogre2 3/3 정상 (2026-10-02). 기본값 ogre2 |
| 공동 | `angular_dist_threshold: 0.1`(5.7°)이라 제자리 회전이 수렴하지 못하고 yaw 회전이 과다했다 | 0.785(45°)로 수정 (`fix/floor-offset`), 재측정 필요 |
| 공동 | 규약은 TF 를 `base_link` 로 정했는데 실제는 `base_footprint` (바닥 수정 후 둘은 같은 자리) | 문서만 맞추면 됨 |
| C | `sync.sh stop` 이 mode_manager·record_run 을 안 꺼서 관리자가 중복됐다 / `sync.sh run` 이 record_run 을 `-u` 로 띄워 통신 불가 / mode_manager 없이 돌아 비행이 실행되지 않았다 | 수정 (2026-10-02) |
| C | INA226 실측 — 지금은 `reference_power.yaml` 추정값 | Phase 2 |

상세와 근거는 `HANDOFF.md` 2절·3절.
