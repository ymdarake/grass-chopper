"""
============================================================================
草刈りロボット 実機起動ファイル
============================================================================
sim_launch.py から Gazebo / ros_gz_bridge を除去し、
実機ハードウェアドライバに置き換えた launch ファイル。

起動するもの:
  1. robot_state_publisher (URDF → TF)
  2. serial_bridge_node (Pico UART 通信 → /odom, /joint_states, /cmd_vel)
  3. rplidar_ros (LiDAR ドライバ)
  4. bno055_driver (IMU ドライバ) [オプション]
  5. slam_toolbox (SLAM)

使い方:
  ros2 launch grass_chopper robot_launch.py
  ros2 launch grass_chopper robot_launch.py use_imu:=true
============================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _launch_setup(context, *args, **kwargs):
    """Launch引数を解決してノードを構築する"""

    pkg_share = get_package_share_directory('grass_chopper')

    # --- Launch引数の解決 ---
    serial_port = LaunchConfiguration('serial_port').perform(context)
    baud_rate = LaunchConfiguration('baud_rate').perform(context)
    use_imu = LaunchConfiguration('use_imu').perform(context)
    lidar_port = LaunchConfiguration('lidar_port').perform(context)
    imu_port = LaunchConfiguration('imu_port').perform(context)

    # --- URDF の読み込み ---
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_description.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # ===================================================================
    # 1. Robot State Publisher
    # ===================================================================
    # 実機では use_sim_time: false (実時刻を使用)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}]
    )

    # ===================================================================
    # 2. Serial Bridge (Pico UART 通信)
    # ===================================================================
    # Pico と UART で通信し、以下を担当:
    #   - /cmd_vel 購読 → Pico に速度指令送信
    #   - Pico からエンコーダ値受信 → /odom, /joint_states 発行
    serial_bridge = Node(
        package='grass_chopper',
        executable='serial_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': int(baud_rate),
            'use_sim_time': False,
            'wheel_radius': 0.05,           # URDF と一致させる
            'wheel_separation': 0.24,       # URDF と一致させる
            'ticks_per_rev': 1440,          # エンコーダ仕様に合わせる
            'watchdog_timeout_sec': 0.5,    # 通信途絶で全停止
        }],
    )

    # ===================================================================
    # 3. RPLidar ドライバ
    # ===================================================================
    # RPLidar A1 を USB 接続し /scan トピックを発行
    # シミュレーションと同じ /scan トピック → slam_toolbox がそのまま動く
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',       # URDF のリンク名と一致
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
    )

    # ===================================================================
    # 4. BNO055 IMU ドライバ (オプション)
    # ===================================================================
    # IMU データを /imu トピックに発行
    # incline_monitor_node がそのまま購読する
    bno055 = Node(
        package='bno055',
        executable='bno055',
        output='screen',
        parameters=[{
            'ros_topic_prefix': '',
            'connection_type': 'uart',
            'uart_port': imu_port,
            'uart_baudrate': 115200,
            'frame_id': 'imu_link',         # URDF のリンク名と一致
            'data_query_frequency': 50,     # シミュレーションと同じ 50Hz
        }],
        condition=IfCondition(LaunchConfiguration('use_imu')),
    )

    # ===================================================================
    # 5. SLAM Toolbox
    # ===================================================================
    # シミュレーションと同じ設定ファイルを使用 (use_sim_time のみ変更)
    slam_params_file = os.path.join(pkg_share, 'config',
                                    'mapper_params_online_async.yaml')

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'false',        # 実時刻を使用
            'slam_params_file': slam_params_file,
        }.items()
    )

    return [
        robot_state_publisher,
        serial_bridge,
        rplidar,
        bno055,
        slam_toolbox,
    ]


def generate_launch_description():
    """LaunchDescriptionを生成する関数"""
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='Pico の UART シリアルポート'),
        DeclareLaunchArgument('baud_rate', default_value='115200',
                              description='UART ボーレート'),
        DeclareLaunchArgument('use_imu', default_value='false',
                              description='BNO055 IMU を使用する場合は true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',
                              description='RPLidar のシリアルポート'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyAMA0',
                              description='BNO055 IMU の UART ポート'),
        OpaqueFunction(function=_launch_setup),
    ])
