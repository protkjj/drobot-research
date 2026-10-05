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
import re
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from ros_gz_bridge.actions import RosGzBridge
from nav2_common.launch import RewrittenYaml


def with_render_engine(world_file, engine):
    """월드 SDF 의 Sensors render_engine 을 engine 으로 바꾼 사본 경로를 돌려준다.

    월드 파일(12개)은 그대로 두고, 바꿀 게 있을 때만 임시 사본을 만든다.
    월드는 상대 경로 참조가 없어 사본 위치가 달라도 깨지지 않는다.
    """
    with open(world_file) as f:
        sdf = f.read()
    new = re.sub(r'<render_engine>[^<]*</render_engine>',
                 f'<render_engine>{engine}</render_engine>', sdf)
    if new == sdf:
        return world_file
    name = os.path.splitext(os.path.basename(world_file))[0]
    fd, path = tempfile.mkstemp(prefix=f'drobot_{name}_{engine}_', suffix='.sdf')
    with os.fdopen(fd, 'w') as f:
        f.write(new)
    return path


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
    # 기본은 mesh — 원본 STL 13개가 실파일로 복구됐다 (2026-10-01 확인).
    #
    # 한때 STL 이 Git LFS 포인터만 남아 Gazebo 가 로봇을 못 만들어
    # primitives(단순 도형)를 기본으로 뒀는데, 그 대체본은 치수와 원점이
    # 잘못돼 있었다 — 본체가 바퀴보다 작고(0.128 m vs 바퀴 지름 0.212 m)
    # 팔 충돌체가 링크 원점에서 20 cm 아래에 있었다. 보기만 이상한 게
    # 아니라 물리도 틀렸다. primitives 는 폐기 대상이다.
    gz_gui = context.launch_configurations.get('gz_gui', 'true').lower() == 'true'
    headless_rendering = context.launch_configurations.get(
        'headless_rendering', 'true').lower() == 'true'
    hr_flag = '--headless-rendering ' if (headless_rendering and not gz_gui) else ''
    # Gazebo 자체 로그를 sim.log 로 끌어온다. 시뮬이 멈출 때 어느 시스템에서
    # 막혔는지는 이것 없이는 안 보인다 (기본 출력은 Msg 수준뿐).
    gz_verbose = context.launch_configurations.get('gz_verbose', 'false').lower() == 'true'
    v_flag = '-v 4 ' if gz_verbose else ''
    # Gazebo 가 렌더 컨텍스트를 세울 동안 다른 노드를 띄우지 않는다.
    #
    # 왜: 로봇이 스폰되면 Sensors 시스템이 ogre 렌더 컨텍스트를 만든다.
    # 런치는 노드 13개를 동시에 올리는데, 그 부하 중에 초기화가 끝나지
    # 못하고 멈추는 일이 잦았다 (/clock 부터 모든 토픽이 0).
    # 같은 조건을 수동으로(프로세스 1~2개) 돌리면 매번 성공한다.
    # 2026-10-01 진단.
    try:
        startup_delay = float(context.launch_configurations.get('startup_delay', '8.0'))
    except ValueError:
        startup_delay = 8.0
    use_rviz = context.launch_configurations.get('use_rviz', 'true').lower() == 'true'
    robot_model = context.launch_configurations.get('robot_model', 'mesh')
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

    # 센서 렌더 엔진. 월드 파일에는 ogre(v1) 가 박혀 있다 (RTX 5070 Ti 에서
    # ogre2 가 멈춰서 바꾼 것). 그런데 ogre v1 에서는 gpu_lidar 가 모든 빔에
    # range_min(0.5 m) 을 낸다 — LiDAR 를 1 m 올려도 같고, ogre2 로 바꾸면
    # 0.9~4.4 m 실제 거리가 나온다 (2026-10-02 실측, RTX 4070 SUPER).
    # LiDAR 가 죽으면 SLAM·obstacle_layer·elevation_layer 가 로봇 둘레 0.5 m
    # 에 가짜 벽을 그린다. 그래서 기본을 ogre2 로 두고, ogre2 가 멈추는
    # 머신에서만 render_engine:=ogre 로 내린다.
    render_engine = context.launch_configurations.get('render_engine', 'ogre2')
    if os.path.exists(world_file):
        world_file = with_render_engine(world_file, render_engine)
    print(f"[INFO] render_engine={render_engine} -> {world_file}")
    if render_engine == 'ogre':
        print("[WARNING] render_engine=ogre: gpu_lidar 가 모든 빔에 range_min 을 낸다. "
              "/scan 은 발행되지만 값이 쓸모없다 — SLAM·장애물 감지가 동작하지 않는다.")

    # Config files (all from bringup)
    # planner 인자로 baseline과 제안 방법을 바꿔가며 실험할 수 있다.
    #   smac2d   : Nav2 SMAC 2D (지상 전용) — logging_config.yaml 의 Baseline 1
    #   proposed : 에너지 인식 2.5D 하이브리드 A* + ElevationLayer
    #
    # energy 인자는 proposed 일 때만 의미가 있다.
    #   default : 이륙 5.0 + 착륙 3.0 Wh — 전환 고정비가 평지 16 m 상당
    #   derived : 이륙 0.5 + 착륙 0.3 Wh — 평지 1.6 m 상당
    # 이착륙 에너지는 INA226 실측 전이라 확정값이 없는데, 이 값이 비행 선택
    # 여부를 통째로 좌우한다. default 로는 벤치마크 5개 맵 전부에서 모드 전환이
    # 0회였다. 실측 전까지 한쪽을 고르지 않고 둘 다 돌려 비교한다.
    planner = context.launch_configurations.get('planner', 'smac2d')
    energy = context.launch_configurations.get('energy', 'default')
    if planner == 'proposed':
        fname = ('nav2_params_hybrid_derived.yaml' if energy == 'derived'
                 else 'nav2_params_hybrid.yaml')
        nav2_params = os.path.join(bringup_pkg, 'config', 'navigation', fname)
    else:
        nav2_params = os.path.join(
            bringup_pkg, 'config', 'navigation', 'nav2_params.yaml')
    print(f"[INFO] planner={planner} energy={energy} "
          f"-> {os.path.basename(nav2_params)}")

    # ========== 정답 높이맵 (prior_map) ==========
    # 센서는 표면만 본다. global_costmap 은 track_unknown_space: false 라
    # 미관측을 자유공간으로 보므로, 플래너는 늘 '안 본 곳을 지나는 지상 경로'를
    # 찾아낸다. 그래서 장애물 내부를 관통하는 경로가 나오고, 비행이 선택될
    # 이유도 생기지 않는다.
    #   실측 (2026-10-05): 높이 관측률 4.8%. 지상 통과가 물리적으로 불가능한
    #   nogap 월드에서도 박스 영역의 100% 가 미관측이라 switch_points 가
    #   항상 빈 배열이었다.
    #
    # 이 연구는 '맵이 주어진 조건에서의 에너지 인식 경로 계획'이지 탐색이
    # 주제가 아니므로, 벤치마크 맵의 정답 높이맵을 ElevationLayer 에 심는다.
    # 맵이 없는 월드에서는 빈 문자열이 되어 센서만으로 동작한다.
    #
    # 기본은 꺼둔다 (use_prior_map:=true 로 켠다).
    use_prior_map = context.launch_configurations.get('use_prior_map', 'false')
    prior_map = ''
    if use_prior_map.lower() in ('true', '1', 'yes'):
        prior_map_candidates = [os.path.join(desc_pkg, 'maps', f'{world}.heightmap')]
        # 설치 전 소스 트리도 본다 (월드 파일을 찾는 방식과 같은 규칙).
        for _ws in ws_candidates:
            prior_map_candidates.append(
                os.path.join(_ws, 'src', 'drobot_description', 'maps',
                             f'{world}.heightmap'))
        prior_map = next((p for p in prior_map_candidates if os.path.exists(p)), '')
        if prior_map:
            print(f"[INFO] prior_map -> {prior_map}")
        else:
            print(f"[INFO] prior_map 파일 없음 ({world}) — 센서만으로 동작")

    # RewrittenYaml 은 '이미 있는 키'만 치환한다. params 파일의 두 elevation_layer
    # 블록에 prior_map: "" 가 들어 있어야 여기서 채워진다.
    nav2_params = RewrittenYaml(
        source_file=nav2_params,
        root_key='',
        param_rewrites={'prior_map': prior_map},
        convert_types=True,
    )
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
        #
        # '--headless-rendering' 은 ogre2 를 EGL 로 열게 한다.
        #   주의: 이 플래그만으로는 Gazebo 멈춤이 해결되지 않았다.
        #   실제 원인은 렌더 엔진 선택이었고, 월드의 Sensors 플러그인에서
        #   ogre2 -> ogre(v1) 로 바꿔야 돈다 (월드 파일 주석 참고).
        #   이 플래그는 그것과 별개로 headless 에서 두는 편이 맞아 유지한다.
        'gz_args': (f'-r {v_flag}{world_file}' if gz_gui
                    else f'-r -s {hr_flag}{v_flag}{world_file}'),
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
        period=startup_delay + 5.0,
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
        period=startup_delay + 5.0,
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
        # Gazebo 가 자리잡은 뒤에 로봇과 브리지를 올린다 (위 주석 참고)
        # Gazebo 가 렌더 컨텍스트를 세울 동안은 아무것도 더 올리지 않는다.
        # EKF 도 여기 넣는다 — 브리지의 /odom 이 있어야 의미가 있고,
        # 그 전에 띄우면 Gazebo 초기화 때 CPU 만 뺏는다.
        TimerAction(period=startup_delay,
                    actions=[spawn_robot, ros_gz_bridge, ekf_node,
                             start_goal_markers]),
        unpause,
        rviz2,
        # SLAM + Navigation (Gazebo 안정화 + 5초)
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
            default_value='mesh',
            choices=['primitives', 'mesh'],
            description=(
                '로봇 모델: mesh(원본 STL, 기본) | primitives(폐기 대상 — 치수·원점이 '
                '틀려 물리까지 어긋난다. urdf 파일 상단 주석 참고)'
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
        DeclareLaunchArgument(
            'energy',
            default_value='default',
            description="에너지 파라미터 세트: default | derived "
                        "(proposed 플래너에서만 의미 있음)",
        ),
        DeclareLaunchArgument(
            'startup_delay',
            default_value='8.0',
            description=(
                'Gazebo 가 뜨고 나서 로봇 스폰·브리지·Nav2 를 올리기까지 기다리는 초. '
                '동시에 올리면 Sensors 의 렌더 컨텍스트 초기화가 부하에 밀려 '
                '시뮬 루프가 멈추는 일이 있었다. 0 으로 두면 예전 동작.'
            ),
        ),
        DeclareLaunchArgument(
            'gz_verbose',
            default_value='false',
            description=(
                'Gazebo 를 -v 4 로 띄워 디버그 로그를 sim.log 에 남긴다. '
                '시뮬이 멈출 때 원인을 보려면 필요하다. 평소엔 로그가 길어져 꺼둔다.'
            ),
        ),
        DeclareLaunchArgument(
            'use_prior_map',
            default_value='false',
            description=(
                '벤치마크 맵의 정답 높이맵을 ElevationLayer 에 미리 심는다. '
                '미관측 영역이 자유공간으로 취급되어 플래너가 장애물 내부를 '
                '관통하는 문제를 막는다. 해당 .heightmap 이 없으면 무시된다.'
            ),
        ),
        DeclareLaunchArgument(
            'headless_rendering',
            default_value='true',
            description=(
                'headless(gz_gui:=false)일 때 Gazebo 에 headless 렌더링 플래그를 준다. '
                '렌더 엔진을 EGL 로 열게 한다. 참고: Gazebo 멈춤의 실제 원인은 '
                '이 플래그가 아니라 ogre2 였고, 월드에서 ogre(v1) 로 바꿔 해결했다. '
                'false 로 두면 예전 동작.'
            ),
        ),
        DeclareLaunchArgument(
            'render_engine',
            default_value='ogre2',
            choices=['ogre2', 'ogre'],
            description=(
                'Gazebo 센서 렌더 엔진 (월드 파일 값을 덮어쓴다). ogre2 가 기본 — '
                'ogre(v1) 에서는 gpu_lidar 가 모든 빔에 range_min 을 내 LiDAR 가 '
                '사실상 죽는다. ogre2 가 멈추는 머신(RTX 5070 Ti + gz-sim 8.11)에서만 ogre.'
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
