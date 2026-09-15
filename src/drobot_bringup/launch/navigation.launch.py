#!/usr/bin/env python3
"""
Drobot Research Navigation Launch File
Gazebo + SLAM + Nav2 한번에 실행

실행 방법:
  ros2 launch drobot_bringup navigation.launch.py
  ros2 launch drobot_bringup navigation.launch.py world:=warehouse
  ros2 launch drobot_bringup navigation.launch.py world:=f1_circuit
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from ros_gz_bridge.actions import RosGzBridge


def launch_setup(context):
    bringup_pkg = get_package_share_directory('drobot_bringup')
    desc_pkg = get_package_share_directory('drobot_description')
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = context.launch_configurations.get('world', 'empty')
    # 테스트 맵 start 좌표와 일치 (test_maps.py: start=(2.0, 1.0))
    # yaw=π/2: 로봇 정면이 +y_world (목표 방향). LiDAR 마운트 정상화 후 재시도.
    # (이전엔 lidar rpy 180°가 SLAM 통해 yaw 강제 보정시켰을 가능성)
    spawn_x, spawn_y, spawn_yaw = '2.0', '1.0', '1.5708'
    goal_x, goal_y = 2.0, 10.0

    # URDF (from drobot_description)
    # 원본 drobot.urdf.xacro 는 STL 메시를 참조하는데, 그 파일들이
    # Git LFS 포인터만 남고 실제 데이터가 없어 Gazebo 가 로봇을 못 만든다.
    # robot_model:=primitives 로 단순 도형 버전을 쓴다 (기본값).
    # STL 을 되찾으면 robot_model:=mesh 로 원본을 쓰면 된다.
    gz_gui = context.launch_configurations.get('gz_gui', 'true').lower() == 'true'
    use_rviz = context.launch_configurations.get('use_rviz', 'true').lower() == 'true'
    robot_model = context.launch_configurations.get('robot_model', 'primitives')
    urdf_name = ('drobot.urdf.xacro' if robot_model == 'mesh'
                 else 'drobot_primitives.urdf.xacro')
    urdf_file = os.path.join(desc_pkg, 'urdf', urdf_name)
    print(f"[INFO] robot_model={robot_model} -> {urdf_name}")
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # World file (from drobot_description)
    # Search order:
    # 1) install-space share
    # 2) source-space generated/original (for newly generated worlds not yet installed)
    ws_env = os.environ.get('DROBOT_WORKSPACE_DIR', '')
    ws_candidates = []
    if ws_env:
        ws_candidates.append(ws_env)
    ws_candidates.append(os.getcwd())

    source_generated_candidates = []
    source_original_candidates = []
    for ws in ws_candidates:
        ws_src = os.path.join(ws, 'src', 'drobot_description', 'worlds')
        source_generated_candidates.extend([
            os.path.join(ws_src, 'benchmark', f'{world}.sdf'),
            os.path.join(ws_src, 'generated', f'{world}.sdf'),
            os.path.join(ws_src, 'generated', f'{world}.world'),
        ])
        source_original_candidates.extend([
            os.path.join(ws_src, 'original', f'{world}.sdf'),
            os.path.join(ws_src, 'original', f'{world}.world'),
            os.path.join(ws_src, f'{world}.sdf'),
            os.path.join(ws_src, f'{world}.world'),
        ])

    world_candidates = [
        # 벤치마크 맵을 먼저 찾는다 (benchmark/export_sdf.py 로 생성).
        # 원본 월드들은 Git LFS 자산이 서버에 없어 복구 불가 상태다.
        os.path.join(desc_pkg, 'worlds', 'benchmark', f'{world}.sdf'),
        os.path.join(desc_pkg, 'worlds', f'{world}.sdf'),
        os.path.join(desc_pkg, 'worlds', f'{world}.world'),
        os.path.join(desc_pkg, 'worlds', 'original', f'{world}.sdf'),
        os.path.join(desc_pkg, 'worlds', 'original', f'{world}.world'),
        os.path.join(desc_pkg, 'worlds', 'generated', f'{world}.sdf'),
        os.path.join(desc_pkg, 'worlds', 'generated', f'{world}.world'),
        *source_generated_candidates,
        *source_original_candidates,
    ]

    world_file = next((p for p in world_candidates if os.path.exists(p)), None)
    if world_file is None:
        print(f"[WARNING] World file not found: {world}")
        print("  Searched install/share and source-space worlds directories.")
        fallback_candidates = [
            os.path.join(desc_pkg, 'worlds', 'original', 'empty.sdf'),
            os.path.join(desc_pkg, 'worlds', 'original', 'empty.world'),
            *[p for p in source_original_candidates if os.path.basename(p) in ('empty.sdf', 'empty.world')],
        ]
        world_file = next((p for p in fallback_candidates if os.path.exists(p)), fallback_candidates[0])

    # Config files (all from bringup)
    # planner 인자로 baseline과 제안 방법을 바꿔가며 실험할 수 있다.
    #   smac2d   : Nav2 SMAC 2D (지상 전용) — logging_config.yaml 의 Baseline 1
    #   proposed : 에너지 인식 2.5D 하이브리드 A* + ElevationLayer
    planner = context.launch_configurations.get('planner', 'smac2d')
    if planner == 'proposed':
        nav2_params = os.path.join(
            bringup_pkg, 'config', 'navigation', 'nav2_params_hybrid.yaml')
    else:
        nav2_params = os.path.join(
            bringup_pkg, 'config', 'navigation', 'nav2_params.yaml')
    print(f"[INFO] planner={planner} -> {os.path.basename(nav2_params)}")
    bt_xml = os.path.join(bringup_pkg, 'config', 'navigation', 'navigate_with_replanning.xml')
    slam_params = os.path.join(bringup_pkg, 'config', 'common', 'slam_params.yaml')
    ekf_params = os.path.join(bringup_pkg, 'config', 'common', 'ekf.yaml')
    bridge_config = os.path.join(bringup_pkg, 'config', 'common', 'ros_gz_bridge.yaml')
    rviz_config = os.path.join(bringup_pkg, 'config', 'navigation', 'display.rviz')

    # ========== GZ Resource Path (for package:// mesh resolution) ==========
    # Gazebo needs the parent of 'drobot_description' on GZ_SIM_RESOURCE_PATH
    # so that package://drobot_description/meshes/... resolves correctly
    gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join([
            os.path.join(desc_pkg, '..'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )

    # ========== Simulation Nodes ==========

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Extract world name from SDF for unpause service
    import xml.etree.ElementTree as ET
    try:
        _tree = ET.parse(world_file)
        _world_el = _tree.find('.//world')
        gz_world_name = _world_el.get('name', 'default') if _world_el is not None else 'default'
    except Exception:
        gz_world_name = 'default'

    # Start Gazebo PAUSED (no -r) so arms can be initialized before physics runs
    gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')
    ]),
    launch_arguments={
        # '-r' 은 센서 시스템을 로드하고 바로 시작하게 한다.
        # '-s' 는 서버만 띄운다 (GUI 없음).
        #   GUI + RViz 가 X 서버와 GPU 를 점유하면 원격 데스크톱 입력이
        #   먹통이 되고 SSH 까지 끊긴 적이 있어, 기본을 headless 로 둔다.
        'gz_args': (f'-r {world_file}' if gz_gui else f'-r -s {world_file}'),
        'on_exit_shutdown': 'true'
    }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'drobot',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', '0.05',
            '-Y', spawn_yaw,
        ],
        output='screen'
    )

    ros_gz_bridge = RosGzBridge(
        bridge_name='ros_gz_bridge',
        config_file=bridge_config,
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # Gazebo OdometryPublisher가 odom을 world 절대좌표로 발행 → slam_toolbox map = world.
    # 따라서 마커도 world 절대좌표 그대로 사용.
    start_goal_markers = Node(
        package='drobot_bringup',
        executable='start_goal_markers',
        name='start_goal_markers',
        output='screen',
        parameters=[{
            'frame_id': 'map',
            'start_x': float(spawn_x),
            'start_y': float(spawn_y),
            'goal_x': goal_x,
            'goal_y': goal_y,
            'use_sim_time': use_sim_time,
        }],
    )

    # ========== Localization Nodes ==========

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}],
    )

    # ========== SLAM ==========

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    slam_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['slam_toolbox']
        }],
    )

    # ========== Navigation Nodes ==========

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params],
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params, {
            'default_nav_to_pose_bt_xml': bt_xml,
        }],
    )

    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params],
    )

    nav_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'velocity_smoother',
            ]
        }],
    )

    # After 5s: unpause Gazebo (Gazebo는 -r로 이미 실행 중이라 사실상 no-op이지만 안전망)
    unpause = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'service', '-s', f'/world/{gz_world_name}/control',
                     '--reqtype', 'gz.msgs.WorldControl',
                     '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '2000',
                     '--req', 'pause: false'],
                output='screen',
            ),
        ],
    )

    # 물리/EKF 안정화 후 SLAM + Nav2를 일괄 기동.
    # 초기 사선 인식 / drift로 인한 길찾기 실패 방지 (5초간 EKF가 정지 odom + IMU bias 수렴).
    delayed_navigation = TimerAction(
        period=5.0,
        actions=[
            slam_node,
            slam_lifecycle,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            velocity_smoother,
            nav_lifecycle,
        ],
    )

    return [
        # Environment
        gz_resource_path,
        # Simulation
        robot_state_publisher,
        gazebo,
        spawn_robot,
        ros_gz_bridge,
        unpause,
        rviz2,
        start_goal_markers,
        # Localization (즉시 — odom/IMU 융합은 일찍 시작해야 SLAM 시작 시점에 안정)
        ekf_node,
        # SLAM + Navigation (5초 지연)
        delayed_navigation,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='World name (empty, warehouse, f1_circuit, office_maze, param_test)'
        ),
        DeclareLaunchArgument(
            'gz_gui',
            default_value='true',
            description='Gazebo GUI 표시 (false면 서버만 — 원격 작업 시 권장)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='RViz 실행 (false면 미실행 — 원격 작업 시 권장)'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='primitives',
            choices=['primitives', 'mesh'],
            description=(
                'primitives(단순 도형, STL 불필요) 또는 mesh(원본 STL 필요)'
            )
        ),
        DeclareLaunchArgument(
            'planner',
            default_value='smac2d',
            choices=['smac2d', 'proposed'],
            description=(
                'Global planner: smac2d(baseline, 지상 전용) 또는 '
                'proposed(하이브리드 A* + 2.5D elevation)'
            )
        ),
        OpaqueFunction(function=launch_setup),
    ])
