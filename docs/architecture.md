# アーキテクチャ概要

## 全体構成

本プロジェクトは「Mac のブラウザだけで ROS 2 ロボットシミュレーションを動かす」ことを目的とした 3 層構成です。

```
┌─────────────────────────────────────────────────────┐
│  ホスト (Mac)                                        │
│                                                      │
│  ・ソースコードの管理・編集                          │
│  ・Multipass による VM 制御                          │
│  ・ブラウザで noVNC に接続                           │
│                                                      │
│  weeder_ws/src/grass_chopper/                        │
│       │                                              │
│       │  multipass mount (双方向同期)                │
│       ▼                                              │
│  ┌──────────────────────────────────────────────┐   │
│  │  Multipass VM (Ubuntu 24.04)                  │   │
│  │                                                │   │
│  │  ┌──────────────────────────────────────────┐ │   │
│  │  │  Gazebo Harmonic (シミュレーション層)      │ │   │
│  │  │                                            │ │   │
│  │  │  ┌──────────┐  ┌──────────┐  ┌────────┐ │ │   │
│  │  │  │ Physics  │  │ DiffDrive│  │Sensors │ │ │   │
│  │  │  │ System   │  │ System   │  │System  │ │ │   │
│  │  │  └──────────┘  └──────────┘  └────────┘ │ │   │
│  │  │       ↕ Gazebo 内部トピック                │ │   │
│  │  │  cmd_vel  scan  camera/image_raw  clock  │ │   │
│  │  └──────────────────┬───────────────────────┘ │   │
│  │                     │                          │   │
│  │  ┌──────────────────▼───────────────────────┐ │   │
│  │  │  ros_gz_bridge (トピック変換層)           │ │   │
│  │  │                                            │ │   │
│  │  │  GZ cmd_vel  ←──  ROS /cmd_vel            │ │   │
│  │  │  GZ scan     ──→  ROS /scan               │ │   │
│  │  │  GZ camera   ──→  ROS /camera/image_raw   │ │   │
│  │  │  GZ clock    ──→  ROS /clock              │ │   │
│  │  └──────────────────┬───────────────────────┘ │   │
│  │                     │                          │   │
│  │  ┌──────────────────▼───────────────────────┐ │   │
│  │  │  ROS 2 ノード群 (アプリケーション層)      │ │   │
│  │  │                                            │ │   │
│  │  │  weeder_node          /scan を購読         │ │   │
│  │  │   (障害物回避)        /cmd_vel を発行      │ │   │
│  │  │                                            │ │   │
│  │  │  robot_state_publisher                     │ │   │
│  │  │   (URDF→TF変換)      /tf を発行           │ │   │
│  │  └────────────────────────────────────────────┘ │   │
│  │                                                │   │
│  │  ┌────────────────────────────────────────────┐ │   │
│  │  │  仮想デスクトップ (GUI層)                   │ │   │
│  │  │  Xvfb → LXDE → x11vnc → websockify        │ │   │
│  │  │                         ↑ :6080             │ │   │
│  │  └────────────────────────────────────────────┘ │   │
│  └──────────────────────────────────────────────────┘   │
│       ↑ http://<VM_IP>:6080/vnc.html                    │
│       │                                                  │
│  ブラウザ (Chrome / Safari / Firefox)                    │
└─────────────────────────────────────────────────────────┘
```

## レイヤーの役割

### 1. シミュレーション層 (Gazebo Harmonic)

物理世界のシミュレーションを担当します。

- **Physics System**: 重力・衝突・摩擦などの物理演算
- **DiffDrive System**: 左右車輪の回転差による移動制御
- **Sensors System**: LiDAR/カメラのデータ生成
- **SceneBroadcaster**: 3D 描画データの GUI への配信

全てのプラグインは Gazebo ネイティブで、ROS 2 に直接依存しません。
これにより Gazebo 単体でもシミュレーションが動作し、デバッグが容易です。

### 2. トピック変換層 (ros_gz_bridge)

Gazebo と ROS 2 の間でメッセージ型を変換する薄いレイヤーです。

```
方向記号:
  ]  ROS → GZ  (ロボットへの指令)
  [  GZ → ROS  (センサーデータ)

/cmd_vel         geometry_msgs/msg/Twist     ]  gz.msgs.Twist
/scan            sensor_msgs/msg/LaserScan   [  gz.msgs.LaserScan
/camera/image_raw sensor_msgs/msg/Image      [  gz.msgs.Image
/clock           rosgraph_msgs/msg/Clock     [  gz.msgs.Clock
```

### 3. アプリケーション層 (ROS 2 ノード)

ロボットの知能にあたる部分です。

- **mission_tree_node**: ミッション管理ステートマシン (IDLE/COVERAGE/RETURNING/CHARGING)
- **battery_sim_node**: バッテリーシミュレーション (`/battery_state` 配信)
- **coverage_commander_node**: カバレッジ走行 (NavigateToPose でウェイポイント順次送信)
- **coverage_tracker_node**: リアルタイムカバレッジ率計算 (`/coverage_ratio` 配信)
- **Nav2 スタック**: 経路計画 + 経路追従 (SmacPlanner2D + RPP)
- **slam_toolbox**: SLAM (map→odom TF + `/map` 配信)
- **robot_state_publisher**: URDF を解析して TF (座標変換ツリー) を発行
- **weeder_node**: LiDAR 障害物回避 (Phase 2, Nav2 移行後は未使用)

### 4. GUI 層 (noVNC)

VM 内の仮想デスクトップをブラウザに配信します。

```
Xvfb (:0)           仮想フレームバッファ (画面がなくても GUI を描画)
  ↓
LXDE                 軽量デスクトップ環境
  ↓
x11vnc (localhost)   VNC サーバー (画面をネットワーク共有)
  ↓
websockify (:6080)   WebSocket ↔ TCP 変換
  ↓
ブラウザ             noVNC クライアントが自動ロード
```

## データフロー

### 障害物回避の 1 サイクル

```
1. Gazebo の gpu_lidar が 360 本のレーザーを照射し距離を計測
   ↓
2. Sensors System がデータを GZ トピック "scan" に発行
   ↓
3. ros_gz_bridge が gz.msgs.LaserScan → sensor_msgs/msg/LaserScan に変換
   ↓
4. weeder_node が /scan を受信 (QoS: BEST_EFFORT)
   ↓
5. 前方 ±30度 の最小距離を計算
   ↓
6a. 0.5m 未満 → Twist(linear.x=0, angular.z=0.5) を /cmd_vel に発行
6b. 0.5m 以上 → Twist(linear.x=0.2, angular.z=0) を /cmd_vel に発行
   ↓
7. ros_gz_bridge が geometry_msgs/msg/Twist → gz.msgs.Twist に変換
   ↓
8. DiffDrive System が左右車輪の回転速度を計算し物理エンジンに反映
   ↓
1. に戻る (10Hz サイクル)
```

## ロボットモデル構成

```
base_link (本体シャーシ)
├── left_wheel_link   ← continuous joint (無限回転)
├── right_wheel_link  ← continuous joint (無限回転)
├── caster_link       ← fixed joint (摩擦ゼロの球体)
├── lidar_link        ← fixed joint (360度 gpu_lidar)
└── camera_link       ← fixed joint (RGB カメラ)
```

## 起動シーケンス

`ros2 launch grass_chopper sim_launch.py` が以下の順序で起動します:

```
1. gz_sim.launch.py
   → Gazebo Harmonic を起動し obstacles.world を読み込み

2. robot_state_publisher
   → robot_description.urdf.xacro を解析し /tf を発行開始

3. ros_gz_sim create
   → /robot_description トピックから URDF を読み取り Gazebo にスポーン

4. ros_gz_bridge parameter_bridge
   → 4 つのトピック変換を開始

5. weeder_node
   → /scan の購読と /cmd_vel の発行を開始
```
