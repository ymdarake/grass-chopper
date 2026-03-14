"""
============================================================================
Nav2 ナビゲーション起動ファイル (Phase 4a)
============================================================================
slam_toolbox + Gazebo が起動済みの状態で、Nav2 スタックを起動します。

起動構成:
  1. twist_mux          (cmd_vel_nav + cmd_vel_teleop → /cmd_vel_raw)
  1.1 collision_monitor (cmd_vel_raw → /cmd_vel)
  2. controller_server   (RPP 経路追従)
  3. planner_server      (SmacPlanner2D 経路計画)
  4. behavior_server     (Spin, Backup, Wait リカバリー)
  5. bt_navigator        (Behavior Tree ナビゲーション管理)
  6. waypoint_follower   (複数ウェイポイント追従)
  7. lifecycle_manager   (上記ノードの状態管理)

注意:
  - AMCL / map_server は不要 (slam_toolbox が map→odom TF と /map を発行)
  - nav2_bringup/bringup_launch.py は AMCL 前提のため使わず個別起動

使い方:
  ros2 launch grass_chopper nav2_launch.py
============================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Nav2 スタックの LaunchDescription を生成"""

    pkg_share = get_package_share_directory('grass_chopper')

    # --- パラメータファイル ---
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    twist_mux_params_file = os.path.join(pkg_share, 'config',
                                         'twist_mux_params.yaml')
    collision_monitor_params_file = os.path.join(pkg_share, 'config',
                                                 'collision_monitor_params.yaml')

    # use_sim_time を全ノードに適用するためのパラメータ書き換え
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        param_rewrites={'use_sim_time': 'true'},
        convert_types=True,
    )

    collision_monitor_params = RewrittenYaml(
        source_file=collision_monitor_params_file,
        param_rewrites={'use_sim_time': 'true'},
        convert_types=True,
    )

    # ===================================================================
    # 1. twist_mux (cmd_vel マルチプレクサ)
    # ===================================================================
    # Nav2 の cmd_vel_nav と手動の cmd_vel_teleop を優先度で切り替え、
    # 最終的な /cmd_vel_raw (衝突監視前) に出力する
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[('/cmd_vel_out', '/cmd_vel_raw')],
        parameters=[twist_mux_params_file, {'use_sim_time': True}],
    )

    # ===================================================================
    # 1.1 Collision Monitor (衝突監視・緊急停止)
    # ===================================================================
    # twist_mux からの入力を監視し、衝突の危険がある場合は停止・減速して /cmd_vel に出力
    collision_monitor = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[collision_monitor_params],
    )

    # ===================================================================
    # 2. Controller Server (RPP: Regulated Pure Pursuit)
    # ===================================================================
    # planner が生成した経路に沿ってロボットを制御する
    # /cmd_vel 出力を /cmd_vel_nav にリマップし、twist_mux 経由で最終出力
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        respawn=False,
        parameters=[configured_params],
        remappings=[('/cmd_vel', '/cmd_vel_nav')],
    )

    # ===================================================================
    # 3. Planner Server (SmacPlanner2D)
    # ===================================================================
    # ゴールまでのグローバル経路を計画する
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        output='screen',
        respawn=False,
        parameters=[configured_params],
    )

    # ===================================================================
    # 4. Behavior Server (リカバリー動作)
    # ===================================================================
    # スタック時のスピン、バック、待機動作を提供
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        output='screen',
        respawn=False,
        parameters=[configured_params],
        remappings=[('/cmd_vel', '/cmd_vel_nav')],
    )

    # ===================================================================
    # 5. BT Navigator
    # ===================================================================
    # Behavior Tree でナビゲーション全体を管理
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        output='screen',
        respawn=False,
        parameters=[configured_params],
    )

    # ===================================================================
    # 6. Waypoint Follower
    # ===================================================================
    # 複数ウェイポイントを順次追従 (Phase 4b で本格利用)
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        output='screen',
        respawn=False,
        parameters=[configured_params],
    )

    # ===================================================================
    # 7. Lifecycle Manager
    # ===================================================================
    # Nav2 ノード群を順次 configure → activate する
    # bond_timeout: 300s で VM 低速起動に対応
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[configured_params],
    )

    # ===================================================================
    # 8. Coverage Tracker (オプション)
    # ===================================================================
    # coverage_tracking:=true で有効化
    coverage_tracking_arg = DeclareLaunchArgument(
        'coverage_tracking', default_value='false',
        description='カバレッジ追跡ノードを起動するか')

    coverage_tracker = Node(
        package='grass_chopper',
        executable='coverage_tracker_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'tool_radius': 0.5,
                      'update_frequency': 5.0}],
        condition=IfCondition(LaunchConfiguration('coverage_tracking')),
    )

    # ===================================================================
    # 9. Incline Monitor (傾斜検知・緊急停止, Phase 4f)
    # ===================================================================
    # IMU データから傾斜を監視し、危険時に /cmd_vel_incline へゼロ速度を発行
    # twist_mux で最優先 (priority: 30) として統合される
    incline_params_file = os.path.join(pkg_share, 'config',
                                       'incline_params.yaml')
    incline_monitor = Node(
        package='grass_chopper',
        executable='incline_monitor_node',
        output='screen',
        parameters=[incline_params_file, {'use_sim_time': True}],
    )

    return LaunchDescription([
        coverage_tracking_arg,
        twist_mux,
        collision_monitor,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        coverage_tracker,
        incline_monitor,
    ])
