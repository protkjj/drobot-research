#!/usr/bin/env bash
# Gazebo 기동 진단 — 어느 조합에서 시뮬 루프가 멈추는지 좁힌다.
#
# 왜 스크립트인가
#   docker exec 안에 여러 줄 셸을 넣으면 따옴표가 깨져 호스트에서
#   실행되거나 pkill 이 자기 자신을 죽이는 일이 반복됐다. 파일로 두면
#   그 문제가 사라지고, 같은 조건을 그대로 재현할 수 있다.
#
# 사용 (컨테이너 안에서)
#   bash /app/tools/diag_gz.sh file     월드 + create -file  로 스폰
#   bash /app/tools/diag_gz.sh topic    월드 + robot_state_publisher + create -topic
#   bash /app/tools/diag_gz.sh world    월드만 (로봇 없음)
# set -u 는 쓰지 않는다 — ROS 의 setup.bash 가 선언되지 않은 변수를
# 참조해서 'AMENT_TRACE_SETUP_FILES: unbound variable' 로 죽는다.

MODE="${1:-file}"
GZ=/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz
WORLD=/app/src/drobot_description/worlds/base_map_h0.5.sdf
URDF_SRC=/app/install/drobot_description/share/drobot_description/urdf/drobot.urdf.xacro
TMP=/tmp/diag_gz

mkdir -p "$TMP"
source /opt/ros/jazzy/setup.bash
source /app/install/setup.bash 2>/dev/null

cleanup() {
  # 'gz[ ]sim' 은 "gz 공백 sim" 에만 매칭된다. 이 스크립트 자신의
  # 명령줄에는 대괄호가 그대로 들어 있어 자기 자신을 죽이지 않는다.
  pkill -f "gz[ ]sim" 2>/dev/null
  pkill -f "robot_state_publisher" 2>/dev/null
  sleep 1
}
trap cleanup EXIT

cleanup
echo "=== 모드: $MODE ==="

ruby "$GZ" sim -r -s --headless-rendering "$WORLD" --force-version 8 -v 4 \
  > "$TMP/gz.log" 2>&1 &
sleep 8

case "$MODE" in
  world)
    ;;
  file)
    xacro "$URDF_SRC" > "$TMP/robot.urdf" 2>"$TMP/xacro.log" || { echo "xacro 실패"; tail -3 "$TMP/xacro.log"; }
    ros2 run ros_gz_sim create -file "$TMP/robot.urdf" -name drobot -x 2 -y 1 -z 0.05 \
      > "$TMP/create.log" 2>&1
    tail -1 "$TMP/create.log"
    ;;
  topic)
    xacro "$URDF_SRC" > "$TMP/robot.urdf" 2>"$TMP/xacro.log" || { echo "xacro 실패"; tail -3 "$TMP/xacro.log"; }
    ros2 run robot_state_publisher robot_state_publisher "$TMP/robot.urdf" \
      > "$TMP/rsp.log" 2>&1 &
    sleep 4
    ros2 run ros_gz_sim create -topic robot_description -name drobot -x 2 -y 1 -z 0.05 \
      > "$TMP/create.log" 2>&1
    tail -1 "$TMP/create.log"
    ;;
  *)
    echo "모드는 world | file | topic"; exit 1 ;;
esac

sleep 6
N=$(timeout 6 "$GZ" topic -l 2>/dev/null | wc -l)
timeout 6 "$GZ" topic -e -t /clock -n 1 >/dev/null 2>&1
C=$?
echo "  gz 토픽 $N · clock 종료코드 $C"
if [ "$C" != 0 ]; then
  echo "  --- Gazebo 마지막 4줄"
  tail -4 "$TMP/gz.log" | sed 's/^/    /'
fi
