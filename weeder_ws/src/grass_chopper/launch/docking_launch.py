"""
============================================================================
ドッキング起動ファイル (Phase 4e)
============================================================================
AprilTag 検出 + opennav_docking (SimpleChargingDock) を起動する。
Nav2 スタック (nav2_launch.py) が起動済みの状態で実行する。

起動構成:
  1. apriltag_node       (AprilTag 36h11 検出)
  2. docking_server      (opennav_docking SimpleChargingDock)
  3. lifecycle_manager   (docking_server のライフサイクル管理)

使い方:
  ros2 launch grass_chopper docking_launch.py
============================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """ドッキングスタックの LaunchDescription を生成"""

    pkg_share = get_package_share_directory('grass_chopper')

    # --- パラメータファイル ---
    apriltag_params_file = os.path.join(
        pkg_share, 'config', 'apriltag_params.yaml')
    docking_params_file = os.path.join(
        pkg_share, 'config', 'docking_server_params.yaml')

    # ===================================================================
    # 1. AprilTag 検出ノード
    # ===================================================================
    # /camera/image_raw を購読して AprilTag を検出し、TF を配信する。
    # VM 環境では raw transport を使用 (圧縮の CPU 負荷回避)。
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        output='screen',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=[apriltag_params_file, {'use_sim_time': True}],
    )

    # ===================================================================
    # 2. Docking Server (opennav_docking)
    # ===================================================================
    # SimpleChargingDock プラグインで AprilTag TF を使ったドッキングを行う。
    docking_server = Node(
        package='opennav_docking',
        executable='docking_server',
        name='docking_server',
        output='screen',
        parameters=[docking_params_file],
    )

    # ===================================================================
    # 3. Lifecycle Manager (ドッキングサーバー用)
    # ===================================================================
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_docking',
        output='screen',
        parameters=[docking_params_file],
    )

    return LaunchDescription([
        apriltag_node,
        docking_server,
        lifecycle_manager,
    ])
