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
  ros2 action list | grep navigate
'
```

기대값:

```
gz 토픽 32
clock: 0
scan:  0
/navigate_to_pose
```

`gz 토픽 0` 이면 Gazebo 가 멈춘 것이다. 4-1 참고.

---

## 3. 실험 실행 (C 트랙)

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

- **렌더 엔진**: 월드의 `<render_engine>` 이 `ogre2` 면 RTX 5070 Ti + gz-sim 8.11
  에서 렌더 스레드 초기화가 안 끝난다. 월드 12개를 `ogre` (v1) 로 바꿨다.
  **월드를 새로 만들 때 `ogre2` 를 쓰지 말 것.**
- **실행 사용자**: `docker exec -u $(id -u):$(id -g)` 로 `ros2 launch` 를 띄우면
  같은 증상이 난다. root 로 띄우면 매번 성공한다 (3/3 vs 전부 실패).
  `sync.sh` 의 `do_sim` 에서 `-u` 를 뺐다. **빌드는 `-u` 를 유지해야 한다**
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
docker exec drobot_ros2 bash -lc 'grep render_engine /app/install/drobot_description/share/drobot_description/worlds/base_map_h0.5.sdf'
```

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
  rviz2 -d /app/src/drobot_bringup/config/navigation/display.rviz
'
```

보이는 것: RobotModel · LaserScan · Map · Path · TF · MarkerArray(시작/목표)

경로 비교만 볼 때는 전용 설정이 있다.

```bash
  rviz2 -d /app/src/drobot_bringup/config/navigation/paths_view.rviz
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
python3 /app/tools/costmap_probe.py --x0 2.12 --y0 1.5 --x1 2.12 --y1 5.0              # cost 등급
python3 /app/tools/costmap_probe.py --mode elevation --x0 2.12 --y0 1.5 --x1 2.12 --y1 5.0   # 높이
```

`elevation_grid` 는 cost 가 아니라 **높이**다 (`--mode elevation`).

---

## 5. 지금 알려진 문제

| 트랙 | 문제 | 상태 |
|---|---|---|
| A | 빈 바닥이 0.25 m 로 측정돼 fly_over 로 분류됨 — URDF `base_footprint_joint` z=0.25 가 원인 (gz 실측 z=-0.2518) | `fix/floor-offset` 에서 수정, 시뮬 재측정 대기 |
| 공동 | 카메라(실제 0.36 m)가 0.5 m 장애물 윗면을 못 봄 → 미관측 → `track_unknown_space: false` 라 free → 착륙 가능으로 보임 | 플래너에서 착륙만 관측 칸으로 제한 (`fix/floor-offset`), 테스트 대기 |
| B | 플래너가 inflation 값을 지형 등급으로 읽음 — 벽에서 0.45 m 이내가 fly_over 로 읽힘 | 수정 — 지형은 ElevationLayer 에서 읽음 (`fix/floor-offset`), 테스트 대기 |
| B | 플래너가 21 cm 구간에 이착륙을 건다 (전환 1회 11.2 Wh) | 바닥 수정 후 재측정 필요 |
| B | 재계획이 잦고 전환점이 매 계획마다 튄다 | 바닥 수정 후 재측정 필요 |
| B | 착륙점이 장애물 한가운데로 잡힌다 | 원인은 미관측 → free. 위 착륙 제한으로 대응, 재측정 필요 |
| 공동 | 설정 파일의 에너지 값 31개가 전부 버려져 `DROBOT_ENERGY=derived` 가 적용되지 않았다 (이전 derived 기록은 default 로 계획된 것) | 수정 (`fix/floor-offset`) — derived 결과는 다시 낼 것 |
| B | `scripts/test_maps.py` 린터 실패 (flake8 94 · pep257 2, 대부분 작은따옴표 규칙) | 동작 무관, 미수정 |
| A | 런치로 띄울 때만 ogre 초기화가 불안정한 근본 원인 (회피책은 적용됨) | 미규명 |
| 공동 | `angular_dist_threshold: 0.1` 은 5.7° 인데 주석은 45° (Nav2 기본 0.785) | 미결정 |
| 공동 | 규약은 TF 를 `base_link` 로 정했는데 실제는 `base_footprint` (바닥 수정 후 둘은 같은 자리) | 문서만 맞추면 됨 |
| C | INA226 실측 — 지금은 `reference_power.yaml` 추정값 | Phase 2 |

상세와 근거는 `HANDOFF.md` 2절·3절.
