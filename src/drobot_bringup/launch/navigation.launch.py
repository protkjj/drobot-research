#!/usr/bin/env python3
"""
Drobot Navigation Launch File
Gazebo + SLAM + Nav2 + Goal Navigator 한번에 실행

실행 방법:
  ros2 launch drobot_bringup navigation.launch.py
  ros2 launch drobot_bringup navigation.launch.py world:=warehouse
  ros2 launch drobot_bringup navigation.launch.py world:=f1_circuit
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from ros_gz_bridge.actions import RosGzBridge


def get_spawn_position(bringup_pkg, world_name):
    """Load spawn position from centralized YAML config."""
    spawn_file = os.path.join(bringup_pkg, 'config', 'spawn_positions.yaml')
    try:
        with open(spawn_file, 'r') as f:
            data = yaml.safe_load(f)
        worlds = data.get('worlds', {})
        pos = worlds.get(world_name, data.get('default', {'x': 0.0, 'y': 0.0}))
        return str(pos['x']), str(pos['y'])
    except Exception:
        return '0.0', '0.0'


def launch_setup(context):
    """Setup launch with context for spawn positions."""
    # Package directories
    bringup_pkg = get_package_share_directory('drobot_bringup')
    desc_pkg = get_package_share_directory('drobot_description')
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # Get launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = context.launch_configurations.get('world', 'empty')

    # Get spawn position for this world
    default_x, default_y = get_spawn_position(bringup_pkg, world)
    spawn_x = default_x
    spawn_y = default_y

    # URDF (from drobot_description)
    urdf_file = os.path.join(desc_pkg, 'urdf', 'drobot.urdf.xacro')
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
    nav2_params = os.path.join(bringup_pkg, 'config', 'navigation', 'nav2_params.yaml')
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
            os.path.join(desc_pkg, '..'),  # parent of drobot_description share
            os.path.join(desc_pkg, 'models'),  # hospital world models (aws_robomaker etc.)
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
        # '-r'을 추가하여 센서 시스템을 강제로 로드하고 바로 시작하게 합니다.
        'gz_args': f'-r {world_file}', 
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
            '-z', '0.15'
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
        output='screen'
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

    # Goal Navigator (main node)
    rules_file = os.path.join(bringup_pkg, 'config', 'navigation', 'rules.yaml')
    goal_navigator = Node(
        package='drobot_scan_2d',
        executable='goal_navigator',
        name='goal_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'rules_file': rules_file,
        }],
    )

    # After 5s: unpause Gazebo
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
        # Localization
        ekf_node,
        slam_node,
        slam_lifecycle,
        # Navigation
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        velocity_smoother,
        nav_lifecycle,
        goal_navigator,
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
        OpaqueFunction(function=launch_setup),
    ])
