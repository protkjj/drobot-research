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
# 레포 클론 (submodule 포함)
git clone --recursive https://github.com/<owner>/drobot-research.git
cd drobot-research

# Docker 이미지 빌드
cd docker
docker compose build
```

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

## 실행

```bash
# 시뮬레이션 + SLAM + Nav2
ros2 launch drobot_bringup navigation.launch.py

# 특정 월드 지정
ros2 launch drobot_bringup navigation.launch.py world:=hospital_original

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
