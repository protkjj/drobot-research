# Drobot

**Drobot** = **D**rone + **Robot** 하이브리드 로봇 시뮬레이션

ROS 2 Jazzy + Gazebo Harmonic 기반, 4바퀴 스키드 스티어 주행 + 쿼드콥터 비행이 가능한 하이브리드 로봇 프로젝트

## 패키지 구조

```
ros2_ws/src/
├── drobot_description/   # 로봇 모델 (URDF, meshes, worlds, 99개 3D 모델)
├── drobot_scan_2d/       # 2D 자율주행 (goal_navigator, rule engine)
├── drobot_scan_3d/       # 3D 스캔/비행 (예정)
├── drobot_bringup/       # 런치 파일 + 설정 + World UI
├── drobot_controller/    # 로봇 컨트롤러 (예정)
├── drobot_simulator/     # 시뮬레이션 (예정)
├── drobot_strategy/      # 전략/의사결정 (예정)
└── perception/           # YOLOv8 객체 인식 (카메라 기반)
```

| 패키지 | 빌드 | 상태 | 역할 |
|--------|------|------|------|
| `drobot_description` | ament_cmake | **구현 완료** | URDF, 13개 STL 메시, Gazebo 플러그인 (DiffDrive, LiDAR, IMU, Camera), 16개 월드 파일, 99개 3D 모델 |
| `drobot_scan_2d` | ament_cmake | **구현 완료** | Nav2 기반 자율주행 + YAML 규칙 엔진 (금지구역, 속도제한, 충돌방지) |
| `drobot_bringup` | ament_python | **구현 완료** | 런치 파일 (navigation, bringup), Nav2/SLAM/EKF 설정, World UI (Tkinter) |
| `perception` | ament_python | **구현 완료** | YOLOv8 객체 인식, 거리 추정, 음성 명령 퍼블리시 |
| `drobot_controller` | ament_python | 예정 | - |
| `drobot_scan_3d` | ament_python | 예정 | - |
| `drobot_simulator` | ament_cmake | 예정 | - |
| `drobot_strategy` | ament_python | 예정 | - |

## 외부 의존성

프로젝트 루트(`drobot/`)에 다음 외부 패키지가 필요합니다 (`.gitignore`로 관리):

- **PX4-Autopilot** - PX4 비행 컨트롤러
- **Micro-XRCE-DDS-Agent** - PX4 ↔ ROS 2 통신 브릿지

## 빠른 시작

### 1. 클론

```bash
git clone https://github.com/protkjj/drobot.git
cd drobot
```

### 2. 의존성 설치

```bash
# ROS 2 패키지
sudo apt update && sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-common \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro \
  ros-jazzy-cv-bridge \
  ros-jazzy-teleop-twist-keyboard

# Python (perception용)
pip install ultralytics
```

### 3. 빌드

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. 실행

```bash
# 자율주행 (Navigation)
ros2 launch drobot_bringup navigation.launch.py
ros2 launch drobot_bringup navigation.launch.py world:=hospital_original

# World UI (월드 생성 + 런치)
ros2 run drobot_bringup world_ui

# RViz 시각화만
ros2 launch drobot_description display.launch.p
y

# Gazebo 시뮬레이션만
ros2 launch drobot_description gazebo.launch.py
```

## 실행 흐름 (navigation.launch.py)

1. **시뮬레이션** - Gazebo 시작 (paused) → 로봇 스폰 → ROS-Gazebo 브릿지
2. **물리 시작** - 5초 후 Gazebo unpause
3. **위치 추정** - EKF 노드 + SLAM Toolbox (async)
4. **내비게이션** - Nav2 스택 (planner, controller, behavior, BT navigator, velocity smoother)
5. **시각화** - RViz2
6. **자율주행** - goal_navigator 노드

## 센서 구성

| 센서 | ROS 토픽 | 용도 |
|------|----------|------|
| LiDAR | `/scan` | SLAM, 충돌 감지 |
| RGB 카메라 | `/camera/image_raw` | 객체 인식 (YOLO) |
| IMU | `/imu` | EKF 위치 추정 |
| 오도메트리 | `/odom` | EKF 위치 추정 |

## 목표 설정

RViz에서 "2D Goal Pose" 버튼 클릭 후 맵에서 위치 지정

또는 터미널에서:
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}
}"
```

## 프로젝트 구조 상세

```
drobot_description/
├── urdf/
│   ├── drobot.urdf.xacro       # 메인 URDF (하이브리드 드론-로버)
│   ├── gazebo.xacro             # Gazebo 플러그인 (DiffDrive, LiDAR, IMU, Camera)
│   └── ros2_control.xacro       # ros2_control 인터페이스
├── meshes/                      # 13개 STL 메시 (base, arms, wheels, frames, camera, lidar)
├── launch/
│   ├── display.launch.py        # RViz 시각화
│   └── gazebo.launch.py         # Gazebo 시뮬레이션
├── config/display.rviz
├── object/obstacle_library/     # 장애물 YAML 라이브러리
├── models/                      # 99개 Gazebo 3D 모델 (병원, 가구 등)
└── worlds/original/             # 16개 사전 정의 월드 (hospital, warehouse, cafe 등)

drobot_scan_2d/
├── drobot_scan_2d/
│   ├── goal_navigator.py        # Nav2 액션 클라이언트, 충돌 감지 (LiDAR ±60°)
│   ├── config.py                # 안전거리(0.25m), 타임아웃(180s), 속도 상수
│   └── rules/engine.py          # YAML 규칙 엔진 (금지구역, 속도제한, 정지규칙)
└── test/test_rule_engine.py     # 규칙 엔진 유닛 테스트

drobot_bringup/
├── launch/
│   ├── navigation.launch.py    # Gazebo + SLAM + Nav2 + GoalNavigator 올인원
│   └── bringup.launch.py       # World UI용 런치
├── config/
│   ├── common/                  # EKF, SLAM, Gazebo 브릿지
│   ├── navigation/              # Nav2, BT, rules, RViz
│   └── spawn_positions.yaml     # 월드별 스폰 위치
└── drobot_bringup/
    ├── world_ui.py              # Tkinter 월드 런처 GUI
    └── worldgen.py              # SDF 월드 자동 생성기

perception/
├── perception/perception.py     # YOLOv8 노드 (카메라 → 객체 인식 → 거리 추정)
├── launch/perception.launch.py
├── models/                      # best.pt, food_model.pt
└── dataset/                     # YOLOv8 학습 데이터 (Roboflow)
```

## 규칙 엔진

`config/navigation/rules.yaml`에서 설정:

| 규칙 | 설명 |
|------|------|
| 장애물 감지 | 0.5m 이내 → 3초 대기 후 재시도 |
| 긴급 정지 | 0.3m 이내 → 즉시 정지 |
| 넓은 경로 선호 | 최소 경로 폭 0.5m, 벽 이격 0.3m |
| 최대 속도 | 직진 0.22 m/s, 회전 0.8 rad/s |

## 명령어 요약

| 상황 | 명령어 |
|------|--------|
| 터미널 열었을 때 | `source install/setup.bash` |
| 코드 수정 후 | `colcon build --symlink-install && source install/setup.bash` |
| 자율주행 실행 | `ros2 launch drobot_bringup navigation.launch.py` |
| 월드 변경 | `... world:=<월드이름>` |
| World UI | `ros2 run drobot_bringup world_ui` |
| 키보드 조종 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard` |
| 프로세스 종료 | `pkill -9 -f "gz\|rviz\|nav2\|slam\|ekf\|goal"` |