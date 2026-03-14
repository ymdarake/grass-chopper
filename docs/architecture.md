# アーキテクチャ概要

## 全体構成

本プロジェクトは「Mac のブラウザだけで ROS 2 ロボットシミュレーションを動かす」ことを目的とした 4 層構成です。

```
┌──────────────────────────────────────────────────────────────┐
│  ホスト (Mac)                                                 │
│  ・ソースコード編集 (weeder_ws/)                              │
│  ・純粋ロジックテスト (make test-pure, 189件)                 │
│                                                               │
│  ┌──────────────────────────────────────────────────────────┐│
│  │  Multipass VM (Ubuntu 24.04, 4CPU / 8GB RAM / 30GB)      ││
│  │                                                           ││
│  │  ┌────────────────────────────────────────────────────┐  ││
│  │  │  シミュレーション層 (Gazebo Harmonic)                │  ││
│  │  │  Physics / DiffDrive / Sensors / IMU                 │  ││
│  │  │       ↕ Gazebo ネイティブトピック                     │  ││
│  │  └──────────────────────┬─────────────────────────────┘  ││
│  │                         │                                 ││
│  │  ┌──────────────────────▼─────────────────────────────┐  ││
│  │  │  ブリッジ層 (ros_gz_bridge)                          │  ││
│  │  │  cmd_vel (ROS→GZ)                                    │  ││
│  │  │  scan, camera, imu, odom, tf, clock (GZ→ROS)         │  ││
│  │  └──────────────────────┬─────────────────────────────┘  ││
│  │                         │                                 ││
│  │  ┌──────────────────────▼─────────────────────────────┐  ││
│  │  │  アプリケーション層 (ROS 2 ノード群)                 │  ││
│  │  │                                                      │  ││
│  │  │  [知覚]                                              │  ││
│  │  │    slam_toolbox        SLAM (map→odom TF + /map)     │  ││
│  │  │    apriltag_ros        AprilTag マーカー検出          │  ││
│  │  │                                                      │  ││
│  │  │  [計画・制御]                                        │  ││
│  │  │    Nav2 スタック       経路計画 + 経路追従            │  ││
│  │  │    coverage_commander  カバレッジ走行指令             │  ││
│  │  │    mission_tree        ミッション状態管理             │  ││
│  │  │                                                      │  ││
│  │  │  [状態推定]                                          │  ││
│  │  │    battery_sim         バッテリー残量シミュレーション │  ││
│  │  │    coverage_tracker    カバレッジ率追跡               │  ││
│  │  │                                                      │  ││
│  │  │  [安全]                                              │  ││
│  │  │    collision_monitor   LiDAR 衝突防止                │  ││
│  │  │    incline_monitor     IMU 傾斜検知・緊急停止        │  ││
│  │  │    twist_mux           速度指令の優先度制御           │  ││
│  │  │                                                      │  ││
│  │  │  [基盤]                                              │  ││
│  │  │    robot_state_publisher  URDF → TF 変換             │  ││
│  │  │    weeder_node         反応型障害物回避 (Phase 2)     │  ││
│  │  └────────────────────────────────────────────────────┘  ││
│  │                                                           ││
│  │  ┌────────────────────────────────────────────────────┐  ││
│  │  │  GUI 層 (noVNC)                                      │  ││
│  │  │  Xvfb → LXDE → x11vnc → websockify (:6080)          │  ││
│  │  └────────────────────────────────────────────────────┘  ││
│  └──────────────────────────────────────────────────────────┘│
│       ↑ http://localhost:6080/vnc.html                        │
│  ブラウザ (Chrome / Safari)                                   │
└──────────────────────────────────────────────────────────────┘
```

## レイヤーの役割

### 1. シミュレーション層 (Gazebo Harmonic)

物理世界のシミュレーションを担当します。

| プラグイン | 役割 |
|-----------|------|
| Physics System | 重力・衝突・摩擦の物理演算 |
| DiffDrive System | 左右車輪の回転差による移動制御 |
| Sensors System | LiDAR / カメラのデータ生成 |
| IMU System | 加速度・角速度のデータ生成 |
| SceneBroadcaster | 3D 描画データの GUI 配信 |

全プラグインは Gazebo ネイティブで、ROS 2 に直接依存しません。
Classic Gazebo の `libgazebo_ros_*.so` は使用禁止です。

### 2. ブリッジ層 (ros_gz_bridge)

Gazebo と ROS 2 の間でメッセージ型を変換する薄いレイヤーです。

```
方向記号:
  ]  ROS → GZ  (指令)
  [  GZ → ROS  (データ)

/cmd_vel            Twist          ]  gz.msgs.Twist
/scan               LaserScan      [  gz.msgs.LaserScan
/camera/image_raw   Image          [  gz.msgs.Image
/camera/camera_info CameraInfo     [  gz.msgs.CameraInfo
/imu                Imu            [  gz.msgs.IMU
/clock              Clock          [  gz.msgs.Clock
/odom               Odometry       [  gz.msgs.Odometry
/joint_states       JointState     [  gz.msgs.Model
/tf                 TFMessage      [  gz.msgs.Pose_V
```

### 3. アプリケーション層 (ROS 2 ノード群)

ロボットの知能にあたる部分です。Humble Object パターンにより、
純粋ロジック (8 モジュール) と ROS 2 アダプター (6 ノード) に分離されています。

#### 純粋ロジック (ROS 2 非依存)

| モジュール | 責務 |
|-----------|------|
| obstacle_avoidance.py | 反応型障害物回避 (状態遷移 + PD 制御) |
| coverage_planner.py | Boustrophedon カバレッジ経路生成 (Shapely) |
| coverage_tracker.py | カバレッジ率のリアルタイム計算 (NumPy) |
| map_region_detector.py | SLAM 地図からの領域・障害物自動検出 (OpenCV) |
| battery_simulator.py | クーロンカウント方式バッテリーシミュレーション |
| mission_behaviors.py | ミッション状態遷移の判断関数群 |
| docking_behavior.py | ドッキングのアプローチ計算・結果評価 |
| incline_monitor.py | 四元数 → RPY 変換 + 傾斜レベル判定 |

#### ROS 2 アダプター (薄いラッパー)

| ノード | 対応ロジック | 購読 | 発行 |
|--------|-----------|------|------|
| weeder_node | obstacle_avoidance | /scan | /cmd_vel |
| coverage_commander_node | coverage_planner | /map | /coverage_progress |
| coverage_tracker_node | coverage_tracker | /map, /tf | /coverage_ratio |
| battery_sim_node | battery_simulator | /cmd_vel, /is_charging | /battery_state |
| mission_tree_node | mission_behaviors | /battery_state, /coverage_* | /is_charging |
| incline_monitor_node | incline_monitor | /imu | /cmd_vel_incline |

### 4. GUI 層 (noVNC)

VM 内の仮想デスクトップをブラウザに配信します。

```
Xvfb (:0)           仮想フレームバッファ
  ↓
LXDE                 軽量デスクトップ環境
  ↓
x11vnc               VNC サーバー
  ↓
websockify (:6080)   WebSocket ↔ TCP 変換
  ↓
ブラウザ             noVNC クライアント
```

## データフロー

### 速度指令の流れ

複数の速度指令を安全に統合するパイプラインです。

```
cmd_vel_nav     (Nav2, priority:10)     ─┐
cmd_vel_teleop  (手動, priority:20)     ─┼→ twist_mux → /cmd_vel_raw
cmd_vel_incline (傾斜停止, priority:30) ─┘         │
                                                    ▼
                                          collision_monitor
                                         (停止ゾーン/減速ゾーン)
                                                    │
                                                    ▼
                                               /cmd_vel → Gazebo DiffDrive
```

### ミッション管理の流れ

```
mission_tree_node (2Hz ティック)
    │
    ├── /battery_state を監視 → SOC ≤ 20% で RETURNING 遷移
    ├── /coverage_ratio を監視 → 90% 以上で完了判定
    ├── /coverage_progress を監視 → 中断地点の記録
    │
    ├── COVERAGE 時: coverage_commander_node をサブプロセス起動
    ├── RETURNING 時: NavigateToPose でホーム位置へ
    ├── DOCKING 時: DockRobot アクション (opennav_docking)
    └── CHARGING 時: /is_charging = True → battery_sim が充電
```

### 障害物回避の 1 サイクル (Phase 2, Nav2 不使用時)

```
1. Gazebo gpu_lidar → 360 本レーザー照射
2. ros_gz_bridge → /scan (LaserScan)
3. weeder_node → 前方/左/右ゾーンの距離分析
4. obstacle_avoidance → 次の状態・Twist 計算
5. weeder_node → /cmd_vel 発行
6. ros_gz_bridge → DiffDrive → 車輪回転
```

## TF チェーン

```
map ──(slam_toolbox)──→ odom ──(DiffDrive)──→ base_link
                                                ├── lidar_link
                                                ├── camera_link
                                                ├── imu_link
                                                ├── left_wheel_link
                                                ├── right_wheel_link
                                                └── caster_link
```

| TF | 発行者 | 方式 |
|----|--------|------|
| map → odom | slam_toolbox | スキャンマッチング SLAM |
| odom → base_link | DiffDrive プラグイン | オドメトリ |
| base_link → 各リンク | robot_state_publisher | URDF 静的変換 |

**注意**: AMCL と slam_toolbox は共に `map → odom` を発行するため**同時起動禁止**。

## ロボットモデル

```
base_link (0.3m × 0.2m × 0.1m, 2.0kg)
├── left_wheel   半径 0.05m, 幅 0.04m, 0.3kg
├── right_wheel  半径 0.05m, 幅 0.04m, 0.3kg
├── caster       半径 0.025m, 0.1kg (摩擦ゼロ)
├── lidar        半径 0.03m, 高さ 0.04m (シャーシ上面 +0.02m)
├── camera       0.02 × 0.04 × 0.04m (シャーシ前面上部)
└── imu          0.01 × 0.01 × 0.01m (シャーシ中心)
```

## 起動シーケンス

### sim_launch.py (基盤)

```
1. Gazebo Harmonic 起動 (ワールドファイル読み込み)
2. robot_state_publisher 起動 (URDF → TF)
3. ros_gz_sim create (ロボットをワールドにスポーン)
4. ros_gz_bridge 起動 (9 トピック変換)
5. slam_toolbox 起動 (online_async SLAM)
6. weeder_node 起動 (nav2_mode=false の場合のみ)
```

### nav2_launch.py (ナビゲーション)

```
1. twist_mux (cmd_vel の優先度統合)
2. collision_monitor (衝突防止)
3. controller_server (RPP 経路追従)
4. planner_server (SmacPlanner2D 経路計画)
5. behavior_server (リカバリー動作)
6. bt_navigator (ビヘイビアツリー)
7. waypoint_follower
8. lifecycle_manager (上記ノードの状態管理)
9. incline_monitor (傾斜検知)
10. coverage_tracker (オプション)
```

### docking_launch.py (ドッキング)

```
1. apriltag_node (AprilTag 36h11 検出)
2. docking_server (opennav_docking)
3. lifecycle_manager_docking
```
