# Drobot

**Drobot** = **D**rone + **Robot** 하이브리드 로봇 시뮬레이션

ROS 2 Jazzy + Gazebo Harmonic 기반, 4바퀴 스키드 스티어 주행 + 변형 arm을 가진 하이브리드 로봇 프로젝트

## 패키지 구조

```
ros2_ws/src/
├── drobot_description/   # 로봇 모델 (URDF, meshes, worlds, 3D 모델)
├── drobot_scan_2d/       # 2D 자율주행 (goal_navigator, rule engine)
├── drobot_scan_3d/       # 3D 스캔 처리
├── drobot_bringup/       # 런치 파일 + Nav2/SLAM/EKF 설정 + World UI
├── drobot_controller/    # teleop_keyboard (수동 조작 + arm 제어)
├── drobot_simulator/     # 시뮬레이션 유틸
├── drobot_strategy/      # 전략/의사결정
└── perception/           # YOLOv8 콘 인식 (카메라 + depth)
```

| 패키지 | 빌드 | 역할 |
|--------|------|------|
| `drobot_description` | ament_cmake | URDF, STL 메시, Gazebo 플러그인 (DiffDrive, LiDAR, IMU, Camera, Arm PID), 월드, 3D 모델 |
| `drobot_scan_2d` | ament_python | Nav2 기반 자율주행 + YAML 규칙 엔진 (금지구역, 속도제한, 충돌방지) |
| `drobot_bringup` | ament_python | 런치 파일 (navigation, perception, ui), Nav2/SLAM/EKF 설정, World UI (Tkinter) |
| `perception` | ament_python | YOLOv8 콘 인식, depth 기반 거리 추정, 감지 결과 퍼블리시 |
| `drobot_controller` | ament_python | teleop_keyboard (키보드 주행 + arm fold/unfold 제어) |
| `drobot_scan_3d` | ament_python | 3D 스캔 처리 |
| `drobot_simulator` | ament_cmake | 시뮬레이션 유틸 |
| `drobot_strategy` | ament_python | 전략/의사결정 로직 |

## 빠른 시작

### 1. 클론 및 세팅

```bash
cd ~/Desktop
git clone https://github.com/protkjj/drobot.git
cd drobot
bash setup.sh
```

### 2. 의존성 (setup.sh에 포함, 수동 설치 시)

```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-common \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-xacro \
  ros-jazzy-cv-bridge

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
# UI 런처 (권장) - 월드/목표/옵션 선택 후 자동 실행
ros2 launch drobot_bringup ui.launch.py

# 또는 수동 실행
ros2 launch drobot_bringup navigation.launch.py world:=hospital_original
```

## 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 로봇 속도 명령 |
| `/scan` | sensor_msgs/LaserScan | 2D LiDAR 스캔 |
| `/odom` | nav_msgs/Odometry | 오도메트리 |
| `/camera/image_raw` | sensor_msgs/Image | RGB 카메라 |
| `/camera/depth_image_raw` | sensor_msgs/Image | Depth 카메라 |
| `/goal_pose` | geometry_msgs/PoseStamped | 네비게이션 목표 |
| `/detections/labels` | std_msgs/String | 감지된 객체 라벨 |
| `/detections/distance` | std_msgs/Float32 | 감지된 객체 거리 |
| `/robot_dog/speech` | std_msgs/String | 감지 상태 (found/None) |
| `/joint_states` | sensor_msgs/JointState | 관절 상태 |
| `/model/drobot/joint/*/cmd_pos` | std_msgs/Float64 | Arm 관절 위치 명령 |

## 명령어 요약

| 상황 | 명령어 |
|------|--------|
| 터미널 열었을 때 | `cd ros2_ws && source install/setup.bash` |
| 코드 수정 후 | `colcon build --symlink-install && source install/setup.bash` |
| UI 런처 | `ros2 launch drobot_bringup ui.launch.py` |
| 자율주행 실행 | `ros2 launch drobot_bringup navigation.launch.py world:=<월드이름>` |
| Perception | `ros2 launch drobot_bringup perception.launch.py` |
| 키보드 조종 | `ros2 run drobot_controller teleop_keyboard` |
| 목표 지정 | RViz에서 "2D Goal Pose" 클릭 |
| 프로세스 전체 종료 | `pkill -9 -f "gz\|rviz\|nav2\|slam\|ekf\|goal_navigator\|teleop"` |
