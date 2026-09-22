# Drobot Research

**GPS-Denied 실내 환경에서 드론-로버 하이브리드 로봇을 위한 에너지 인식 2.5D 경로 계획**

ROS 2 Jazzy + Gazebo Harmonic 기반 연구 프로젝트

## 연구 목표

GPS를 사용할 수 없는 실내 환경에서, 지상 주행과 비행을 모두 수행하는 하이브리드 로봇이
에너지 효율을 고려한 최적 경로를 계획할 수 있도록 하는 것.

핵심 요소:
- **2.5D Costmap**: RGB-D + LiDAR 센서 융합으로 높이 정보를 포함한 환경 표현
- **에너지 인식 비용 함수**: 주행/비행 모드별 에너지 소비를 반영한 경로 비용
- **Hybrid RRT***: 지상/공중 경로를 통합 탐색하는 Nav2 플래너 플러그인

## 패키지 구조

```
src/
├── drobot_description/      # [유지] URDF, meshes, worlds
├── drobot_bringup/          # [유지] launch, Nav2 config
├── drobot_controller/       # [유지] teleop keyboard
├── drobot_costmap_2_5d/     # [신규] RGB-D + LiDAR → 2.5D costmap
├── drobot_energy_model/     # [신규] INA226 에너지 로깅 + 비용 함수
├── drobot_hybrid_planner/   # [신규] Hybrid RRT* Nav2 플래너 (C++)
├── drobot_experiments/      # [신규] 실험 자동화 + 결과 수집
├── px4_msgs/                # [submodule] PX4 메시지 타입 정의
└── px4-ros2-interface-lib/  # [submodule] PX4-ROS2 인터페이스 라이브러리
```

| 패키지 | 빌드 | 역할 |
|--------|------|------|
| `drobot_description` | ament_cmake | URDF, 메시, Gazebo 월드 |
| `drobot_bringup` | ament_python | 런치 파일, Nav2/SLAM/EKF 설정 |
| `drobot_controller` | ament_python | 키보드 텔레오퍼레이션 |
| `drobot_costmap_2_5d` | ament_python | RGB-D 깊이 + LiDAR 센서 융합 → 2.5D costmap 생성 |
| `drobot_energy_model` | ament_python | INA226 에너지 로깅, 주행/비행 에너지 비용 함수 모델링 |
| `drobot_hybrid_planner` | ament_cmake | Hybrid RRT* Nav2 글로벌 플래너 플러그인 (C++) |
| `drobot_experiments` | ament_python | 실험 자동화, 결과 수집, 베이스라인 비교 |

## 사전 요구사항

- Docker + Docker Compose
- NVIDIA GPU + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

## Docker 환경 설정

이 프로젝트는 Docker Compose로 2개 컨테이너를 사용합니다:

| 컨테이너 | 역할 |
|----------|------|
| `drobot_px4_sim` | PX4 빌드/SITL + Gazebo Harmonic + Micro-XRCE-DDS Agent |
| `drobot_ros2` | ROS2 Jazzy + Nav2 + SLAM + 커스텀 패키지 빌드 |

두 컨테이너는 `network_mode: host`로 DDS 토픽을 공유합니다.

### 최초 설정

```bash
# 1. 레포 클론 (submodule 포함)
git clone --recursive https://github.com/<owner>/drobot-research.git
cd drobot-research

# 2. Git LFS 파일 받기 (mesh STL/DAE, world SDF, model texture 등 680여 개)
git lfs install   # (시스템에 최초 1회만)
git -c lfs.url=https://github.com/protkjj/drobot.git/info/lfs lfs pull

# 3. Docker 이미지 빌드
cd docker
docker compose build
```

> ⚠️ **LFS 주의**: 현재 `drobot-research` 자체에는 LFS 오브젝트가 push되지 않은 상태라
> 위 2번 명령에서 upstream `protkjj/drobot`의 LFS 서버를 일시적으로 참조합니다
> (`-c lfs.url=...`는 config를 영구 변경하지 않음).
> LFS 파일을 받지 않으면 Gazebo가 world 파일을 파싱하지 못해 launch가 실패합니다
> (`Error parsing XML ... ErrorID=8 Line number=1` — pointer 파일을 그대로 파싱한 결과).

### 컨테이너 실행

```bash
# 컨테이너 시작 (백그라운드)
cd docker
docker compose up -d

# ROS2 컨테이너 접속
./exec_ros2.sh

# PX4 컨테이너 접속 (다른 터미널)
./exec_px4_sim.sh
```

### ROS2 컨테이너 안에서

```bash
# 패키지 빌드
cd /app
colcon build --symlink-install
source install/setup.bash

# 또는 alias 사용
cb    # colcon build --symlink-install
ws    # source /app/install/setup.bash
```

### PX4 컨테이너 안에서

```bash
# PX4-Autopilot 클론/빌드 (최초 1회)
cd /app
git clone --recursive https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_sitl gz_x500

# XRCE-DDS Agent 시작
dds    # alias: MicroXRCEAgent udp4 -p 8888
```

### 컨테이너 종료

```bash
cd docker
docker compose down
```

## 빌드 (Docker 없이)

```bash
# 의존성 설치
sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-common \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard

# 빌드
colcon build --symlink-install
source install/setup.bash
```

## 테스트 맵

연구 검증용 통제 환경. `test_maps.py` 한 스크립트가 RRT\* 알고리즘 입력(`.npz`)과 Gazebo 월드(`.sdf`)를 **동일 기하학으로 동시 생성**한다.

**설계**: 6 × 10 m 맵에 직육면체 장애물 1개를 **왼쪽으로 치우치게** 배치 (중심 (2, 5), 4 × 3 m, 높이 가변). start = (2, 0), goal = (2, 10) 직선이 박스를 정면 관통하도록 설계 — 어떤 플래너든 "비행 or 우회" 결정에 직면. 비대칭 배치로 우회 거리가 늘어나 비행과의 에너지 비교가 의미있는 영역으로 들어옴.

장애물 높이만 CLI로 변경 (`--height`) → Rover (≤0.15 m) / Flyover (0.15–2 m) / Impass (>2 m) regime sweep.

### 생성

```bash
# 기본 (높이 0.5 m → base_map_h0.5)
python3 src/drobot_hybrid_planner/scripts/test_maps.py

# 높이 sweep
python3 src/drobot_hybrid_planner/scripts/test_maps.py --height 0.3
python3 src/drobot_hybrid_planner/scripts/test_maps.py --height 1.0

# 옵션
python3 src/drobot_hybrid_planner/scripts/test_maps.py --help
```

| 출력 | 위치 | 용도 |
|------|------|------|
| `<name>.npz` | `src/drobot_hybrid_planner/scripts/maps/` | height / class / cost map + start/goal — RRT\* 입력 |
| `<name>.png` | `src/drobot_hybrid_planner/scripts/maps/` | 디버깅용 2-panel 시각화 |
| `<name>.sdf` | `src/drobot_description/worlds/` | Gazebo 월드 — launch가 자동 탐지 |

높이 임계값과 셀 해상도는 `src/drobot_costmap_2_5d/config/elevation_params.yaml` 에서 로드 (단일 진실 출처).

## 실행

```bash
# 1) 테스트 맵 생성 (위 '테스트 맵' 참고)
python3 src/drobot_hybrid_planner/scripts/test_maps.py --height 0.5

# 2) 빌드
colcon build --packages-select drobot_description drobot_bringup
source install/setup.bash

# 3) 시뮬레이션 + SLAM + Nav2 — 로봇은 맵 start (2, 0)에 spawn
ros2 launch drobot_bringup navigation.launch.py world:=base_map_h0.5

# 키보드 조종
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 센서 구성

| 센서 | ROS 토픽 | 용도 |
|------|----------|------|
| LiDAR | `/scan` | SLAM, 2.5D costmap |
| RGB-D 카메라 | `/camera/image_raw`, `/camera/depth` | 2.5D costmap |
| IMU | `/imu` | EKF 위치 추정 |
| 오도메트리 | `/odom` | EKF 위치 추정 |
| INA226 | (TBD) | 에너지 소비 측정 |
