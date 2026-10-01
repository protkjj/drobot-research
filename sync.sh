#!/usr/bin/env bash
# 랩실 데스크탑으로 코드 동기화
#
# macOS에서 작성한 코드를 ROS2가 있는 원격 머신으로 보낸다.
# 빌드 산출물(build/install/log)은 원격 것을 그대로 두므로
# 매번 전체 재빌드가 일어나지 않는다.
#
# 사용법
#   ./sync.sh              동기화만
#   ./sync.sh build        동기화 + 빌드
#   ./sync.sh test         동기화 + 빌드 + 테스트
#   ./sync.sh watch        파일이 바뀔 때마다 자동 동기화 (Ctrl+C로 중지)
#   ./sync.sh sim [world] [planner]   시뮬레이션 실행 (기본: medium_open proposed)
#   ./sync.sh run <world> [planner]   시뮬레이션 + 목표 전송 + 궤적 기록 (포스터용)
#       DROBOT_ENERGY=derived ./sync.sh run medium_open   <- 비행이 선택되는 세트
#   ./sync.sh stop         시뮬레이션 종료
#
# 주의: 빌드와 시뮬레이션을 동시에 돌리지 않는다.
#       Gazebo + RViz + GPU 렌더링에 colcon 병렬 컴파일이 겹치면
#       머신이 응답하지 않는다 (build/test 는 자동으로 시뮬레이션을 끈다).

set -euo pipefail

# 머신이 두 대다 — 환경변수로 바꿀 수 있게 둔다.
#   detop       집 데스크탑, RTX 5070 Ti, 저장소가 ~/Desktop/drobot-research
#   vail-detop  랩실 데스크탑, RTX 3070 Ti, 저장소가 ~/drobot-research
# 예:  DROBOT_REMOTE=vail-detop DROBOT_REMOTE_DIR='~/drobot-research' ./sync.sh build
REMOTE="${DROBOT_REMOTE:-detop}"
REMOTE_DIR="${DROBOT_REMOTE_DIR:-~/Desktop/drobot-research}"
LOCAL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/"

# --delete 를 기본에서 뺐다.
#   원래는 파일 이름을 바꿨을 때 원격에 옛 파일이 남는 걸 막으려고 썼다.
#   그런데 머신마다 고유 파일이 생기면서 그것들을 지워버렸다 —
#   데탑의 documents/, base_map_h*.npz, start_goal_markers.py 가 실제로 삭제됐다
#   (git 에서 복구했다). 두 머신을 오가며 쓰는 이상 --delete 는 위험하다.
#
#   이름 변경 뒤 옛 파일을 치워야 하면 명시적으로:
#       DROBOT_RSYNC_DELETE=1 ./sync.sh
#   이 경우 원격 고유 파일이 지워지므로 git 상태를 먼저 확인할 것.
# macOS 기본 rsync 는 2.6.9(2006년)라 최신 옵션을 모른다.
# --info=stats1 대신 --stats 를 쓴다 (구버전에도 있는 옵션).
RSYNC_DELETE=""
[ -n "${DROBOT_RSYNC_DELETE:-}" ] && RSYNC_DELETE="--delete"

RSYNC_OPTS=(
  -az $RSYNC_DELETE --stats
  --exclude='.git'
  --exclude='build' --exclude='install' --exclude='log'
  --exclude='__pycache__' --exclude='*.pyc'
  --exclude='.DS_Store' --exclude='.vscode'
  --exclude='benchmark/results/*.json'   # 용량 큰 원자료는 로컬에만
  # Git LFS 자산은 원격에서만 받는다 (git lfs pull).
  # 로컬이 포인터(130B) 상태면 동기화할 때 원격의 실제 파일을 덮어써 버린다.
  # 실제로 이것 때문에 Gazebo 가 "Error parsing XML" 로 죽었다.
  --exclude='src/drobot_description/models/'
  --exclude='src/drobot_description/meshes/'
  #
  # worlds/ 는 제외하지 않는다 (2026-10-01).
  #   월드는 git 에 실파일로 들어 있고(5~14KB) 자주 고친다. 제외해 두면
  #   맥북에서 월드를 고쳐 푸시해도 원격에 반영되지 않아, 고친 걸 시험한 게
  #   아닌데 "안 고쳐졌다" 고 판단하게 된다. 실제로 ogre2 수정이 그렇게 묻혔다.
  #   대신 아래 check_lfs_pointers() 가 포인터 상태면 동기화를 막는다.
)

# 빌드 대상. C++ 는 컴파일이 필요하고, Python 패키지도 한 번은 빌드해야
# setup.py 의 entry_points 가 등록돼 ros2 run 으로 불린다
# (--symlink-install 이라 이후 코드 수정은 재빌드 없이 반영된다).
#
# bringup·description 이 빠져 있어 런치/URDF 를 고쳐도 설치본에 반영되지
# 않은 채로 시뮬을 돌린 적이 있다 (2026-10-01). 그러면 고친 걸 시험한 게
# 아닌데 '안 고쳐졌다' 고 판단하게 된다. 그래서 둘 다 목록에 넣는다.
PKGS="drobot_msgs drobot_hybrid_planner drobot_costmap_2_5d \
drobot_mode_manager drobot_experiments drobot_energy_model \
drobot_bringup drobot_description"

check_lfs_pointers() {
  # LFS 포인터를 원격으로 밀면 원격의 실제 파일이 날아간다.
  # 동기화 대상 중 포인터가 섞여 있으면 멈춘다.
  local found=0
  while IFS= read -r f; do
    if head -c 40 "$f" 2>/dev/null | grep -q "version https://git-lfs"; then
      echo "오류: LFS 포인터 상태의 파일이 있다 — $f" >&2
      found=1
    fi
  done < <(find "$LOCAL_DIR/src/drobot_description/worlds" -name '*.sdf' 2>/dev/null)

  if [ "$found" != 0 ]; then
    echo "" >&2
    echo "동기화하면 원격의 실제 월드 파일을 포인터로 덮어쓴다. 먼저 받아올 것:" >&2
    echo "  git lfs pull" >&2
    return 1
  fi
  return 0
}

do_sync() {
  check_lfs_pointers || return 1
  echo "==> 동기화: $LOCAL_DIR -> $REMOTE:$REMOTE_DIR"
  rsync "${RSYNC_OPTS[@]}" "$LOCAL_DIR" "$REMOTE:$REMOTE_DIR"
}

ensure_container() {
  # 컨테이너가 안 떠 있으면 띄운다.
  #
  # 호스트를 재부팅하면 drobot_ros2 가 Exited 로 남고 자동 시작되지 않는다.
  # 그 상태로 sync.sh sim 을 돌리면 docker exec 가
  #   Error response from daemon: container ... is not running
  # 만 뱉고 끝나서 원인을 알기 어렵다. 미리 확인해 띄운다.
  #
  # compose 가 UID/GID/DISPLAY 를 참조하므로 같이 넘긴다.
  # (UID 는 bash 에서 readonly 라 export 하지 않고 compose 에 직접 준다)
  if ssh "$REMOTE" "docker ps --format '{{.Names}}' | grep -qx drobot_ros2"; then
    return 0
  fi
  echo "==> 컨테이너가 꺼져 있다 — 기동한다"
  ssh "$REMOTE" "cd $REMOTE_DIR/docker && \
    UID=\$(id -u) GID=\$(id -g) DISPLAY=\${DISPLAY:-:0} \
    docker compose up -d ros2 2>&1 | tail -2"
  sleep 3
  ssh "$REMOTE" "docker ps --format '{{.Names}}\t{{.Status}}' | grep drobot_ros2" \
    || { echo "!! 컨테이너 기동 실패" >&2; return 1; }
}

stop_sim() {
  # 빌드 전에 시뮬레이션을 반드시 끈다.
  #
  # 프로세스 목록이 긴 이유: 예전엔 'nav2' 로 잡으려 했는데 실제 프로세스
  # 이름은 controller_server, planner_server, bridge_node 라서 하나도 안 죽었다.
  # 그 결과 실행할 때마다 잔재가 쌓였고, ros_gz_bridge 가 2개 공존하면서
  # /clock 을 이중 발행해 시간이 앞뒤로 튀었다.
  #   Detected jump back in time. Clearing TF buffer.
  # 전 노드에서 이게 반복되면 TF 조회가 계속 실패하고, lifecycle manager 가
  # bt_navigator / planner_server 를 inactive 로 떨어뜨려 목표가 거부된다.
  # Gazebo(server+gui) + RViz + GPU 렌더링에 colcon 병렬 컴파일이 겹치면
  # 머신이 응답하지 않게 된다 (실제로 SSH 가 끊긴 적이 있다).
  echo "==> 시뮬레이션 종료 (빌드와 동시 실행 금지)"
  ssh "$REMOTE" "docker exec drobot_ros2 bash -c \
    \"pkill -9 -f 'gz sim|ruby.*gz|rviz2|bridge_node|ros_gz|slam_toolbox|ekf_node|ros2 launch|robot_state_pub|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|lifecycle_manager|waypoint_follower|smoother_server|map_server|amcl|component_container' 2>/dev/null\" || true"
  sleep 2
}

do_build() {
  ensure_container || return 1
  stop_sim
  echo "==> 원격 빌드 ($PKGS)"
  # 병렬 작업 수를 제한한다. 랩실 데스크탑은 공용이라
  # 16코어를 전부 쓰면 다른 작업이 멈춘다.
  ssh -t "$REMOTE" "cd $REMOTE_DIR && \
    docker exec -u \$(id -u):\$(id -g) drobot_ros2 bash -lc '
      cd /app && source /opt/ros/jazzy/setup.bash && source install/setup.bash 2>/dev/null
      MAKEFLAGS=-j6 colcon build --symlink-install --parallel-workers 3 \
        --packages-select $PKGS --event-handlers console_direct+
    '"
}

do_test() {
  ensure_container || return 1
  echo "==> 원격 테스트 (Python<->C++ 동등성)"
  ssh -t "$REMOTE" "docker exec -u \$(id -u):\$(id -g) drobot_ros2 bash -lc '
      cd /app && source /opt/ros/jazzy/setup.bash && source install/setup.bash
      colcon test --parallel-workers 2 --packages-select drobot_hybrid_planner \
        --event-handlers console_direct+
      colcon test-result --verbose
    '"
}

check_sim_args() {
  local world="$1" planner="$2" mode="$3"
  local ok=0

  case "$world" in
    -*) echo "오류: 월드 이름에 플래그가 들어왔다 ('$world')." >&2; ok=1 ;;
  esac
  case "$planner" in
    smac2d|proposed) ;;
    -*|"") echo "오류: 플래너 이름이 이상하다 ('$planner')." >&2; ok=1 ;;
    *) echo "경고: 모르는 플래너 '$planner' (아는 것: smac2d proposed)" >&2 ;;
  esac
  case "$mode" in
    headless|gui) ;;
    *) echo "오류: mode 는 headless 또는 gui 여야 한다 ('$mode')." >&2; ok=1 ;;
  esac

  # 월드 파일이 로컬에 있는지 본다 (rsync 제외 대상이라 없을 수도 있다)
  local wd="$LOCAL_DIR/src/drobot_description/worlds"
  # ls 에 여러 경로를 주면 하나라도 없을 때 실패한다 — 있는지 하나씩 본다
  local found=0
  if [ -d "$wd" ]; then
    for cand in "$wd/$world.sdf" "$wd"/*/"$world.sdf"; do
      [ -f "$cand" ] && found=1 && break
    done
  fi
  if [ "$ok" = 0 ] && [ -d "$wd" ] && [ "$found" = 0 ]; then
    echo "경고: 로컬에 $world.sdf 가 없다 (원격에는 있을 수 있음)" >&2
    echo "      있는 월드: $(ls "$wd"/*.sdf 2>/dev/null | xargs -n1 basename 2>/dev/null | sed 's/.sdf$//' | tr '\n' ' ')" >&2
  fi

  if [ "$ok" != 0 ]; then
    echo "" >&2
    echo "사용법: ./sync.sh sim <world> <planner> [gui]" >&2
    echo "  에너지 세트는 환경변수다:  DROBOT_ENERGY=derived ./sync.sh sim base_map_h0.5 proposed" >&2
    return 1
  fi
  return 0
}

do_sim() {
  # 시뮬레이션은 빌드가 끝난 뒤에만 띄운다.
  #
  # 기본은 headless 다. RViz + Gazebo GUI 가 X 서버와 GPU 를 점유하면
  # 원격 데스크톱(RustDesk)의 화면 입력이 먹통이 되고 SSH 도 끊긴다.
  # 실제로 두 번 그렇게 머신이 마비됐다.
  # 화면으로 봐야 할 때만 gui 를 붙인다: ./sync.sh sim <world> <planner> gui
  local world="${2:-medium_open}"
  local planner="${3:-proposed}"
  local mode="${4:-headless}"

  # 인자는 위치 기반이다. 플래그를 주면 그게 월드 이름으로 들어가
  # Gazebo 가 없는 월드를 열려다 조용히 실패한다 — 실제로 한 번 겪었다
  # (world=--world 로 들어가 30분을 날렸다). 여기서 막는다.
  check_sim_args "$world" "$planner" "$mode" || return 1
  ensure_container || return 1
  local energy="${DROBOT_ENERGY:-default}"   # default | derived (§energy 인자)
  # Gazebo 가 멈출 때 원인을 보려면 켠다. 로그가 길어져 기본은 꺼둔다.
  #   DROBOT_GZ_VERBOSE=1 ./sync.sh sim base_map_h0.5 proposed
  local gzv="${DROBOT_GZ_VERBOSE:+gz_verbose:=true}"

  local gui_args=""
  local disp="-e DISPLAY=:0"
  if [ "$mode" != "gui" ]; then
    # Gazebo 서버만 띄우고 GUI 와 RViz 는 끈다
    gui_args="use_rviz:=false gz_gui:=false"
    disp=""
  fi

  echo "==> 시뮬레이션: world=$world planner=$planner energy=$energy mode=$mode"
  ssh "$REMOTE" "docker exec -u \$(id -u):\$(id -g) $disp drobot_ros2 bash -lc '
      cd /app && source /opt/ros/jazzy/setup.bash && source install/setup.bash
      nohup ros2 launch drobot_bringup navigation.launch.py \
        world:=$world planner:=$planner energy:=$energy robot_model:=mesh $gui_args $gzv \
        > /app/sim.log 2>&1 &
      echo \"launch 시작 — 로그: $REMOTE_DIR/sim.log\"
    '"
  echo "    확인이 끝나면 반드시: ./sync.sh stop"
}

do_run() {
  # 시뮬레이션을 headless 로 띄우고, 목표를 보내고, 궤적을 받아온다.
  #
  # RViz 를 안 쓰는 이유: GPU·X 서버를 점유해 원격 데스크톱이 먹통이 된다.
  # 대신 /plan · /odom · /mode_switch_points 를 JSON 으로 떠서 가져오고
  # 그림은 macOS 에서 benchmark/plot_sim_run.py 로 그린다.
  local world="${2:-medium_open}"
  local planner="${3:-proposed}"
  local energy="${DROBOT_ENERGY:-default}"
  local json="sim_${world}_${energy}.json"

  ensure_container || return 1
  stop_sim
  do_sim "" "$world" "$planner" headless

  echo "==> Nav2 기동 대기 (25초)"
  sleep 25

  echo "==> 목표 전송 + 궤적 기록: $world"
  ssh -t "$REMOTE" "docker exec -u \$(id -u):\$(id -g) drobot_ros2 bash -lc '
      cd /app && source /opt/ros/jazzy/setup.bash && source install/setup.bash
      python3 src/drobot_experiments/drobot_experiments/record_run.py \
        --world $world --out /app/$json
    '"

  echo "==> 결과 회수"
  rsync -az "$REMOTE:$REMOTE_DIR/$json" "$LOCAL_DIR/benchmark/results/$json"
  echo "    benchmark/results/$json"

  stop_sim
  echo "==> 그림: python3 benchmark/plot_sim_run.py benchmark/results/$json"
}

case "${1:-sync}" in
  sync)
    do_sync
    ;;
  build)
    do_sync && do_build
    ;;
  test)
    do_sync && do_build && do_test
    ;;
  sim)
    # 빌드 없이 시뮬레이션만 (이미 빌드돼 있을 때)
    ensure_container && stop_sim && do_sim "$@"
    ;;
  run)
    do_sync && do_run "$@"
    ;;
  stop)
    stop_sim
    ;;
  watch)
    # 파일이 바뀔 때마다 자동 동기화.
    # fswatch 가 필요하다:  brew install fswatch
    if ! command -v fswatch >/dev/null 2>&1; then
      echo "fswatch 가 없다. 설치: brew install fswatch" >&2
      exit 1
    fi
    do_sync
    echo "==> 감시 시작 (Ctrl+C로 중지)"
    fswatch -o -r \
      --exclude='\.git' --exclude='build' --exclude='install' \
      --exclude='log' --exclude='__pycache__' \
      "$LOCAL_DIR" | while read -r _; do
      do_sync
    done
    ;;
  *)
    echo "사용법: $0 [sync|build|test|watch|sim <world> <planner>|run <world>|stop]" >&2
    exit 1
    ;;
esac
