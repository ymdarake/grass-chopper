"""
============================================================================
草刈りロボット シミュレーション一括起動ファイル
============================================================================
このlaunchファイルは以下を順番に起動します:

  1. Gazebo Harmonic シミュレーター (障害物ワールド付き)
  2. robot_state_publisher (URDFからTF座標変換を発行)
  3. Gazeboへのロボットモデル配置 (スポーン)
  4. ros_gz_bridge (GazeboとROS 2間のトピック変換)
  5. weeder_node (障害物回避ノード)

使い方:
  ros2 launch grass_chopper sim_launch.py
  ros2 launch grass_chopper sim_launch.py world:=slam_test.world x:=0.0 y:=-4.0
============================================================================
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _launch_setup(context, *args, **kwargs):
    """Launch引数を解決してノードを構築する"""

    # --- パッケージのパスを取得 ---
    pkg_share = get_package_share_directory('grass_chopper')

    # --- Launch引数の解決 ---
    world_name = LaunchConfiguration('world').perform(context)
    spawn_x = LaunchConfiguration('x').perform(context)
    spawn_y = LaunchConfiguration('y').perform(context)

    # --- URDF/Xacroファイルの読み込みと変換 ---
    # .xacro ファイルをXMLに変換してロボットの記述を取得
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_description.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # --- Worldファイルのパス ---
    world_file = os.path.join(pkg_share, 'worlds', world_name)

    # ===================================================================
    # 1. Gazebo Harmonic の起動
    # ===================================================================
    # ros_gz_sim パッケージの gz_sim.launch.py をインクルード
    # gz_args: Gazeboに渡す引数
    #   -r: シミュレーションを即座に実行開始
    #   world_file: 読み込むワールドファイル
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # ===================================================================
    # 2. Robot State Publisher の起動
    # ===================================================================
    # URDFを解析してTF (座標変換) ツリーを発行する
    # これにより、rviz2でロボットの各部品の位置関係を可視化できる
    # use_sim_time: True = Gazeboのシミュレーション時刻を使用
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # ===================================================================
    # 3. ロボットモデルをGazeboにスポーン
    # ===================================================================
    # robot_description トピックからURDFを読み取り、
    # Gazeboワールド内にロボットを配置する
    # -z 0.1: 地面から少し浮かせて配置（落下して着地する）
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',  # URDFの取得元トピック
            '-name', 'grass_chopper',        # Gazebo内でのモデル名
            '-x', spawn_x,                   # X軸の初期位置 [m]
            '-y', spawn_y,                   # Y軸の初期位置 [m]
            '-z', '0.1'                      # Z軸の初期位置 [m]
        ]
    )

    # ===================================================================
    # 4. ROS 2 ←→ Gazebo ブリッジ
    # ===================================================================
    # Gazebo Harmonicのトピックと ROS 2のトピックを相互変換する
    #
    # 書式: /トピック名@ROS2型[方向記号]GZ型
    #   ] = ROS→GZ (ROS 2からGazeboへ送信)
    #   [ = GZ→ROS (GazeboからROS 2へ送信)
    #   @ = 双方向
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # 速度指令: ROS 2 → Gazebo (ロボットを動かすコマンド)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # LiDARスキャン: Gazebo → ROS 2 (障害物検知データ)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # カメラ画像: Gazebo → ROS 2 (RGB画像)
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # シミュレーション時刻: Gazebo → ROS 2
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # オドメトリ: Gazebo → ROS 2 (ロボットの位置・速度推定)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # 関節状態: Gazebo → ROS 2 (車輪の回転角度)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # TF: Gazebo → ROS 2 (odom→base_link 座標変換, Phase 3 SLAM用)
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ]
    )

    # ===================================================================
    # 5. 障害物回避ノード
    # ===================================================================
    # LiDARデータを使って障害物を回避しながら前進するノード
    # config/weeder_params.yaml からパラメータを読み込む
    weeder_params_file = os.path.join(pkg_share, 'config', 'weeder_params.yaml')
    weeder_node = Node(
        package='grass_chopper',
        executable='weeder_node',
        output='screen',
        parameters=[weeder_params_file, {'use_sim_time': True}]
    )

    # ===================================================================
    # 6. SLAM Toolbox (Phase 3: 自己位置推定と地図作成)
    # ===================================================================
    # slam_toolbox の online_async モードで、走行しながら2D地図を生成する
    # /scan と /odom, /tf を使ってスキャンマッチングベースの SLAM を実行
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
            'use_sim_time': 'true',
            'slam_params_file': slam_params_file,
        }.items()
    )

    # --- 全ノードをまとめて返す ---
    return [
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge,
        weeder_node,
        slam_toolbox,
    ]


def generate_launch_description():
    """LaunchDescriptionを生成する関数（ROS 2 launchシステムが呼び出す）"""
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='obstacles.world',
                              description='ワールドファイル名 (worlds/ 内)'),
        DeclareLaunchArgument('x', default_value='0.0',
                              description='ロボットの初期X座標 [m]'),
        DeclareLaunchArgument('y', default_value='0.0',
                              description='ロボットの初期Y座標 [m]'),
        OpaqueFunction(function=_launch_setup),
    ])
