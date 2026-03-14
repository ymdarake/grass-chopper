# grass-chopper システム仕様書

## 1. プロジェクト概要

### 1.1 目的

シミュレーション上で**自律草刈りロボット**のアルゴリズムを段階的に開発・検証し、
最終的に実機に搭載できるレベルまで育てる。

### 1.2 設計思想

- **Mac のブラウザだけで完結**: Multipass VM + noVNC で XQuartz 不要
- **シミュレーション → 実機の移行コスト最小化**: ROS 2 標準インターフェースのみ使用し、制御コードは変更なしで実機に移行可能
- **Humble Object パターン**: ロジックを ROS 2 から分離し、Mac ホスト上で pytest のみでテスト可能に
- **TDD (テスト駆動開発)**: Red → Green → Refactor のサイクルを厳守

### 1.3 技術スタック

| レイヤー | 技術 |
|---------|------|
| ホスト OS | macOS |
| VM | Multipass (Ubuntu 24.04 LTS) |
| ロボットフレームワーク | ROS 2 Jazzy |
| シミュレーター | Gazebo Harmonic |
| リモート GUI | noVNC (Xvfb + LXDE + x11vnc + websockify) |
| 言語 | Python 3 |
| テスト | pytest (212件、Mac ホスト実行可) |

---

## 2. ロボット仕様

### 2.1 物理モデル

2輪差動駆動方式の小型ロボット。URDF/Xacro で定義。

```
base_link (本体シャーシ: 0.3m × 0.2m × 0.1m, 2.0kg)
├── left_wheel_link   ← continuous joint (半径 0.05m, 幅 0.04m)
├── right_wheel_link  ← continuous joint
├── caster_link       ← fixed joint (半径 0.025m, 摩擦ゼロ球体)
├── lidar_link        ← fixed joint (シャーシ上面中央)
├── camera_link       ← fixed joint (シャーシ前面上部)
└── imu_link          ← fixed joint (シャーシ中心)
```

| パラメータ | 値 |
|-----------|-----|
| 全長 × 全幅 × 全高 | 0.3m × 0.2m × 0.1m |
| 車輪間距離 | 0.24m |
| 車輪半径 | 0.05m |
| 総重量 | 約 2.8kg |

### 2.2 センサー

| センサー | 型式 | 仕様 | 用途 |
|---------|------|------|------|
| LiDAR | gpu_lidar | 360°, 360サンプル, 10Hz, 0.12〜10.0m | 障害物検知、SLAM |
| カメラ | camera | 640×480, RGB, 5fps | AprilTag 検出 |
| IMU | imu | 50Hz, 加速度+ジャイロ | 傾斜検知 |

### 2.3 アクチュエータ

| アクチュエータ | 方式 | 制御トピック |
|-------------|------|------------|
| 駆動 | 差動駆動 (gz-sim-diff-drive-system) | `/cmd_vel` (Twist) |

---

## 3. システムアーキテクチャ

### 3.1 レイヤー構成

```
┌──────────────────────────────────────────────────────────────┐
│  ホスト (Mac)                                                 │
│  ・ソースコード編集 (weeder_ws/)                              │
│  ・純粋ロジックテスト (make test-pure)                        │
│                                                               │
│  ┌──────────────────────────────────────────────────────────┐│
│  │  Multipass VM (Ubuntu 24.04, 4CPU / 8GB RAM / 30GB)      ││
│  │                                                           ││
│  │  [シミュレーション層] Gazebo Harmonic                     ││
│  │    Physics / DiffDrive / Sensors / IMU / SceneBroadcaster ││
│  │         ↕ Gazebo ネイティブトピック                        ││
│  │                                                           ││
│  │  [ブリッジ層] ros_gz_bridge                               ││
│  │    cmd_vel (ROS→GZ) / scan,camera,clock,odom,tf,imu (GZ→ROS) ││
│  │         ↕ ROS 2 標準トピック                              ││
│  │                                                           ││
│  │  [アプリケーション層] ROS 2 ノード群                      ││
│  │    知覚: slam_toolbox, apriltag_ros                       ││
│  │    計画: Nav2 (SmacPlanner2D, RPP, BT Navigator)          ││
│  │    実行: coverage_commander, mission_tree, battery_sim    ││
│  │    安全: collision_monitor, incline_monitor               ││
│  │                                                           ││
│  │  [GUI層] noVNC (Xvfb → LXDE → x11vnc → websockify :6080) ││
│  └──────────────────────────────────────────────────────────┘│
│       ↑ http://localhost:6080/vnc.html                        │
│  ブラウザ (Chrome / Safari)                                   │
└──────────────────────────────────────────────────────────────┘
```

### 3.2 データフロー: トピック一覧

#### 指令系 (ROS → Gazebo)

| トピック | 型 | 発行元 | 購読先 | 用途 |
|---------|-----|--------|-------|------|
| `/cmd_vel` | Twist | collision_monitor | gz_bridge → DiffDrive | 最終速度指令 |
| `/cmd_vel_raw` | Twist | twist_mux | collision_monitor | 衝突監視前の速度指令 |
| `/cmd_vel_nav` | Twist | controller_server | twist_mux | Nav2 経路追従 |
| `/cmd_vel_teleop` | Twist | (手動操作) | twist_mux | デバッグ用手動操作 |
| `/cmd_vel_incline` | Twist | incline_monitor | twist_mux | 傾斜緊急停止 |

#### センサー系 (Gazebo → ROS)

| トピック | 型 | 発行元 | 購読先 | 用途 |
|---------|-----|--------|-------|------|
| `/scan` | LaserScan | LiDAR | slam_toolbox, collision_monitor | 360° 距離データ |
| `/camera/image_raw` | Image | Camera | apriltag_ros | RGB 画像 |
| `/camera/camera_info` | CameraInfo | Camera | apriltag_ros | カメラ内部パラメータ |
| `/imu` | Imu | IMU | incline_monitor | 加速度・角速度 |
| `/clock` | Clock | Gazebo | 全ノード | シミュレーション時刻 |
| `/odom` | Odometry | DiffDrive | slam_toolbox, Nav2 | 位置・速度推定 |
| `/joint_states` | JointState | DiffDrive | robot_state_publisher | 車輪回転角 |
| `/tf` | TFMessage | DiffDrive, slam_toolbox | 全ノード | 座標変換 |

#### 状態管理系 (ROS ノード間)

| トピック | 型 | 発行元 | 購読先 | 用途 |
|---------|-----|--------|-------|------|
| `/battery_state` | BatteryState | battery_sim | mission_tree | バッテリー残量 |
| `/is_charging` | Bool | mission_tree | battery_sim | 充電中フラグ |
| `/coverage_ratio` | Float32 | coverage_tracker | mission_tree | カバレッジ率 |
| `/coverage_progress` | Int32MultiArray | coverage_commander | mission_tree | [完了index, 総数] |
| `/map` | OccupancyGrid | slam_toolbox | coverage_commander, Nav2 | 2D 地図 |

### 3.3 TF チェーン

```
map ──(slam_toolbox)──→ odom ──(DiffDrive via gz_bridge)──→ base_link
                                                               ├── lidar_link
                                                               ├── camera_link
                                                               ├── imu_link
                                                               ├── left_wheel_link
                                                               ├── right_wheel_link
                                                               └── caster_link
```

- `map → odom`: slam_toolbox が発行（AMCL は不使用）
- `odom → base_link`: DiffDrive プラグインが発行
- `base_link → 各リンク`: robot_state_publisher が URDF から発行

### 3.4 速度指令の優先度制御

```
cmd_vel_nav (Nav2, priority:10)       ─┐
cmd_vel_teleop (手動, priority:20)    ─┼→ twist_mux → cmd_vel_raw → collision_monitor → cmd_vel → Gazebo
cmd_vel_incline (傾斜停止, priority:30)─┘
```

- **twist_mux**: 複数の速度指令を優先度で切り替え（数値が大きいほど優先）
- **collision_monitor**: LiDAR データから停止ゾーン/減速ゾーンを監視
- **incline_monitor**: IMU データから傾斜を監視し、最優先で緊急停止

---

## 4. 機能仕様

### 4.1 障害物回避 (Phase 1-2)

LiDAR で周囲をスキャンし、障害物を回避しながら前進する反応型制御。

#### 状態遷移

```
FORWARD ──(前方障害物検知)──→ AVOID_LEFT or AVOID_RIGHT
   ↑                              │
   └──(前方クリア)────────────────┘

FORWARD ──(壁検知, 側方近接)──→ WALL_FOLLOW ──(壁消失)──→ FORWARD

FORWARD ──(三方向ブロック)──→ U_TURN ──(前方クリア)──→ FORWARD
```

| 状態 | 条件 | 動作 |
|------|------|------|
| FORWARD | 前方 0.5m 以上クリア | 直進 (0.2 m/s) |
| AVOID_LEFT | 前方障害物 + 左が空き | 左旋回 (0.5 rad/s) |
| AVOID_RIGHT | 前方障害物 + 右が空き | 右旋回 (-0.5 rad/s) |
| WALL_FOLLOW | 側方に壁を検知 | PD制御で壁と一定距離を維持 |
| U_TURN | 前方・左・右すべてブロック | その場旋回 (0.7 rad/s) |

#### パラメータ (`weeder_params.yaml`)

| パラメータ | デフォルト | 説明 |
|-----------|-----------|------|
| safe_distance | 0.5m | 障害物検知距離 |
| forward_speed | 0.2 m/s | 前進速度 |
| turn_speed | 0.5 rad/s | 旋回速度 |
| wall_target_distance | 0.4m | 壁沿い走行の目標距離 |
| wall_follow_kp | 1.5 | 壁沿い PD 制御の比例ゲイン |
| wall_follow_kd | 0.3 | 壁沿い PD 制御の微分ゲイン |

### 4.2 SLAM — 自己位置推定と地図作成 (Phase 3)

`slam_toolbox` (online_async モード) で走行しながら 2D 占有格子地図を生成。

| 項目 | 仕様 |
|------|------|
| アルゴリズム | スキャンマッチング SLAM |
| 入力 | `/scan` (LiDAR), `/odom` (オドメトリ) |
| 出力 | `/map` (OccupancyGrid), `map → odom` TF |
| 地図解像度 | 0.05m/pixel |
| 精度 | 10m×10m ワールドで壁寸法の誤差 1〜2% |

### 4.3 Nav2 自律ナビゲーション (Phase 4a)

Nav2 スタックによる経路計画と経路追従。

#### 構成ノード

| ノード | 役割 | 主要パラメータ |
|--------|------|--------------|
| SmacPlanner2D | グローバル経路計画 | tolerance: 0.25m |
| RPP Controller | 経路追従 (Regulated Pure Pursuit) | lookahead: 0.4m, max_vel: 0.25 m/s |
| BT Navigator | ビヘイビアツリーでナビゲーション管理 | — |
| Behavior Server | リカバリー動作 (Spin, Backup, Wait) | — |
| Lifecycle Manager | ノード状態管理 | bond_timeout: 300s |

#### Collision Monitor (衝突防止)

| ゾーン | 範囲 | 動作 |
|--------|------|------|
| StopPolygon | ロボット周囲 0.25m × 0.20m | 即時停止 |
| SlowdownPolygon | 前方 0.70m × 左右 0.40m | 速度 40% に制限 |

### 4.4 カバレッジ走行 (Phase 4b-c)

指定領域をジグザグ (Boustrophedon) パターンで網羅走行する。

#### 走行アルゴリズム

1. 走行領域を Shapely Polygon で定義（手動指定 or SLAM 地図から自動検出）
2. 障害物ポリゴンを差し引き (`Polygon.difference`)
3. Boustrophedon パターンでウェイポイント列を生成
4. `NavigateToPose` アクションでウェイポイントを逐次送信
5. カバレッジ率をリアルタイムで計算・配信

#### カバレッジ率追跡

| 項目 | 仕様 |
|------|------|
| 方式 | NumPy 2D グリッド + `/tf` ベース位置追跡 |
| ツール半径 | 0.5m (走行軌跡の幅) |
| 出力 | `/coverage_ratio` (Float32), `/coverage_grid` (OccupancyGrid) |

#### 地図ベース自動領域検出

```
/map (OccupancyGrid) → 二値化 → CLOSE → erode → findContours → Shapely Polygon
```

- SLAM 地図から走行可能領域と障害物を自動検出
- `auto_detect_region: true` で有効化

### 4.5 バッテリーシミュレーション (Phase 4d)

クーロンカウント方式でバッテリー残量 (SOC) を計算。

| 状態 | 消費電流 | 備考 |
|------|---------|------|
| 待機 (idle) | 0.5A | `/cmd_vel` がゼロ |
| 走行 (drive) | 2.0A | `/cmd_vel` が非ゼロ |
| 草刈り (mow) | 3.0A | (将来拡張用) |
| 充電 (charge) | -2.0A | `/is_charging` が True |

| パラメータ | デフォルト |
|-----------|-----------|
| 容量 | 5.0Ah |
| 満充電電圧 | 12.6V |
| 空電圧 | 10.0V |
| 低残量閾値 | SOC 20% |
| 危険残量閾値 | SOC 10% |

### 4.6 ミッション管理 (Phase 4d-e)

ステートマシンでロボットのライフサイクルを管理。

#### 状態遷移図

```
                ┌─────────────────────────────────────┐
                │                                     │
                ▼                                     │
    ┌────────────────┐                               │
    │     IDLE       │──(未完了エリアあり)──→┌──────────────┐
    │  (待機)        │                      │  COVERAGE    │
    └────────────────┘                      │ (カバレッジ)  │
           ▲                                └──────┬───────┘
           │                                       │
    (SOC 100%,                              (SOC ≤ 20%)
     カバレッジ完了)                                │
           │                                       ▼
    ┌──────┴─────────┐                      ┌──────────────┐
    │   CHARGING     │←──(ドック到着)──────│  RETURNING   │
    │  (充電中)       │                      │ (帰還中)     │
    └────────────────┘                      └──────┬───────┘
           ▲                                       │
           │                                (ホーム到着)
           │                                       │
           │                                       ▼
           │                                ┌──────────────┐
           └────────────(ドック成功)────────│   DOCKING    │
                                            │ (ドッキング)  │
                                            └──────────────┘
```

| 遷移 | 条件 | 動作 |
|------|------|------|
| IDLE → COVERAGE | 未完了エリアあり + バッテリー十分 | カバレッジ走行開始 |
| COVERAGE → RETURNING | SOC ≤ 20% | ホーム位置へ NavigateToPose |
| RETURNING → DOCKING | ホーム到着 | DockRobot アクション or 停止 |
| DOCKING → CHARGING | ドック成功 | `/is_charging` = True 発行 |
| CHARGING → IDLE | SOC 100% | `/is_charging` = False 発行 |
| IDLE → COVERAGE | 中断ウェイポイントあり | `start_index` で再開 |

#### カバレッジ中断・再開

- 走行中のウェイポイント進捗を `/coverage_progress` ([completed_index, total_waypoints]) で配信
- 充電完了後、中断地点 (`completed_index + 1`) からカバレッジを再開
- 全ウェイポイント完了まで COVERAGE → CHARGING → COVERAGE を繰り返す

### 4.7 精密ドッキング (Phase 4e)

opennav_docking + AprilTag による充電ステーションへの自動ドッキング。

| 項目 | 仕様 |
|------|------|
| マーカー | AprilTag 36h11, ID 0, 0.15m × 0.15m |
| ドックタイプ | SimpleChargingDock |
| アクション | `nav2_msgs/action/DockRobot` |
| リトライ | 最大 3 回、失敗時はフォールバック (ホーム位置で停止) |

### 4.8 安全機能 (Phase 4f)

#### 4.8.1 衝突防止 (Collision Monitor)

LiDAR スキャンデータからロボット周囲の障害物を検知し、速度を制限。

- **停止ゾーン**: ロボット周囲 0.25m — 障害物 3 点以上で即時停止
- **減速ゾーン**: 前方 0.70m — 障害物 3 点以上で速度 40% に制限
- **動的障害物対応**: リアルタイムスキャンで静的/動的を区別せず検知

#### 4.8.2 傾斜検知 (Incline Monitor)

IMU センサーから四元数 → ロール・ピッチ角を算出し、傾斜レベルを判定。

| 傾斜レベル | 条件 | 動作 |
|-----------|------|------|
| SAFE | max(roll, pitch) < 15° | 通常走行 |
| WARNING | 15° ≤ max(roll, pitch) < 25° | ログ警告 |
| EMERGENCY | max(roll, pitch) ≥ 25° | `/cmd_vel_incline` にゼロ速度発行（twist_mux 最優先で停止） |

#### 実機移行向けフィルタリング (Phase 5 前倒し準備)

| 機能 | クラス/関数 | 用途 |
|------|-----------|------|
| ローパスフィルタ | `LowPassFilter(alpha)` | IMU の高周波ノイズ除去 (alpha: 0〜1、小さいほど平滑化) |
| ヒステリシス判定 | `evaluate_incline_with_hysteresis()` | 閾値付近のチャタリング防止 (hysteresis_deg で不感帯設定) |
| IMU キャリブレーション | `calibrate_imu_offset(samples)` | 取り付け角度オフセットの推定 (水平面サンプルの平均) |

#### 4.8.3 速度指令の優先度

| 優先度 | ソース | 用途 |
|--------|-------|------|
| 30 (最高) | incline_monitor | 傾斜緊急停止 |
| 20 | teleop | 手動操作 |
| 10 | Nav2 controller | 自律ナビゲーション |

---

## 5. テスト仕様

### 5.1 テスト方針

| 種別 | 実行環境 | 件数 | 方法 |
|------|---------|------|------|
| 純粋ロジックテスト | Mac ホスト | 212件 | `make test-pure` (pytest) |
| 構文チェック | Mac ホスト | — | `make check-syntax` |
| VM 統合テスト | VM | 手動 | `make vm-sim` + `make vm-nav2` |

### 5.2 純粋ロジックテスト内訳

| テストファイル | 対象モジュール | 件数 |
|-------------|-------------|------|
| test_obstacle_avoidance.py | obstacle_avoidance.py | 41 |
| test_coverage_planner.py | coverage_planner.py | 27 |
| test_coverage_tracker.py | coverage_tracker.py | 20 |
| test_map_region_detector.py | map_region_detector.py | 13 |
| test_battery_simulator.py | battery_simulator.py | 18 |
| test_mission_behaviors.py | mission_behaviors.py | 27 |
| test_docking_behavior.py | docking_behavior.py | 12 |
| test_incline_monitor.py | incline_monitor.py | 36 |
| test_weeder_node.py | weeder_node.py | (VM 統合テスト) |

### 5.3 テスト環境 (Gazebo ワールド)

| ワールド | サイズ | 特徴 | Phase |
|---------|-------|------|-------|
| obstacles.world | — | 静的障害物 (箱 4 個) | 1-2 |
| slam_test.world | 10m×10m | ロの字型回廊 (ループクロージャ検証) | 3 |
| coverage_test.world | 8m×8m | オープンフィールド | 4b |
| coverage_obstacles.world | 8m×8m | 障害物 2 個付きフィールド | 4c |
| docking_test.world | 8m×8m | 充電ステーション + AprilTag | 4d-e |
| slope_test.world | 10m×10m | 傾斜面 (10° + 25°) | 4f |
| dynamic_obstacles.world | 8m×8m | 移動可能な障害物 (VelocityControl) | 4f |

---

## 6. 起動手順

### 6.1 起動シーケンス

```
1. make vm-build          ← VM 内で colcon build
2. make vm-sim            ← Gazebo + SLAM + Bridge + weeder_node 起動
3. make vm-nav2           ← Nav2 スタック + twist_mux + collision_monitor + incline_monitor 起動
4. make vm-battery        ← バッテリーシミュレーション起動
5. make vm-mission        ← ミッション管理ノード起動
6. (オプション) make vm-docking  ← AprilTag + opennav_docking 起動
```

### 6.2 Launch ファイル構成

| ファイル | 起動内容 | 前提条件 |
|---------|---------|---------|
| sim_launch.py | Gazebo, robot_state_publisher, spawn, bridge, slam_toolbox, (weeder_node) | なし |
| nav2_launch.py | twist_mux, collision_monitor, controller/planner/behavior_server, bt_navigator, waypoint_follower, lifecycle_manager, incline_monitor, (coverage_tracker) | sim_launch.py 起動済み |
| docking_launch.py | apriltag_node, docking_server, lifecycle_manager_docking | sim_launch.py + nav2_launch.py 起動済み |

### 6.3 ヘッドレスモード

GUI 不要な場合 (統合テスト、CI 等) は `headless:=true` で低負荷起動:

```bash
make vm-sim-headless      # -s --headless-rendering (センサーデータは取得可)
```

---

## 7. モジュール構成

### 7.1 純粋ロジック層 (ROS 2 非依存)

Mac ホスト上で pytest のみで実行可能。外部依存は Python 標準ライブラリ + NumPy + Shapely + OpenCV のみ。

| モジュール | 責務 | 主要関数/クラス |
|-----------|------|--------------|
| obstacle_avoidance.py | 障害物回避ロジック | `RobotState`, `compute_next_state()`, `compute_angular_velocity()` |
| coverage_planner.py | カバレッジ経路計画 | `CoverageParams`, `generate_boustrophedon_path()` |
| coverage_tracker.py | カバレッジ率追跡 | `GridConfig`, `CoverageTracker`, `get_coverage_ratio()` |
| map_region_detector.py | 地図自動領域検出 | `extract_free_regions()`, `detect_obstacles_from_map()` |
| battery_simulator.py | バッテリーシミュレーション | `BatteryParams`, `BatterySimulator` |
| mission_behaviors.py | ミッション判断ロジック | `MissionState`, `CoverageProgress`, `should_return_home()` |
| docking_behavior.py | ドッキング行動ロジック | `DockingParams`, `DockingAction`, `evaluate_docking_result()` |
| incline_monitor.py | 傾斜検知ロジック | `InclineLevel`, `quaternion_to_rpy()`, `evaluate_incline()`, `LowPassFilter`, `evaluate_incline_with_hysteresis()`, `calibrate_imu_offset()` |

### 7.2 ROS 2 アダプター層 (Humble Object)

各モジュールの ROS 2 ラッパー。トピック購読/発行とパラメータ管理のみ担当。

| ノード | 対応ロジック | 購読 | 発行 |
|--------|-----------|------|------|
| weeder_node | obstacle_avoidance | /scan | /cmd_vel |
| coverage_commander_node | coverage_planner | /map | /coverage_progress |
| coverage_tracker_node | coverage_tracker | /map, /tf | /coverage_ratio, /coverage_grid |
| battery_sim_node | battery_simulator | /cmd_vel, /is_charging | /battery_state |
| mission_tree_node | mission_behaviors | /battery_state, /coverage_ratio, /coverage_progress | /is_charging |
| incline_monitor_node | incline_monitor | /imu | /cmd_vel_incline |

---

## 8. 実機移行計画 (Phase 5)

### 8.1 変更不要なコンポーネント

| コンポーネント | 理由 |
|-------------|------|
| 全純粋ロジックモジュール (8 個) | ROS 2 非依存 |
| coverage_commander_node | NavigateToPose 標準アクションのみ使用 |
| mission_tree_node | 標準アクション + トピックのみ使用 |
| Nav2 スタック | 標準インターフェース |
| slam_toolbox | `/scan` + `/odom` のみ依存 |

### 8.2 置き換えが必要なコンポーネント

| シミュレーション | 実機 | 作業量 |
|-------------|------|--------|
| Gazebo Harmonic | 不要 (物理世界) | 削除 |
| gz-sim-diff-drive-system | ros2_control + diff_drive_controller + HW I/F | 大 |
| ros_gz_bridge | 不要 (ドライバが直接 ROS トピック発行) | 削除 |
| gpu_lidar (GZ) | 実機 LiDAR ドライバ (rplidar_ros 等) | 中 |
| camera (GZ) | 実機カメラドライバ (usb_cam 等) | 小 |
| IMU (GZ) | 実機 IMU ドライバ + キャリブレーション | 中 |
| battery_sim_node | 実機バッテリー監視 (ADC or BMS 通信) | 中 |
| sim_launch.py | robot_launch.py を新規作成 | 中 |

### 8.3 実機で追加が必要な機能

| 機能 | 理由 |
|------|------|
| IMU キャリブレーション | 取り付け角度オフセットの補正 |
| IMU フィルタリング | 振動・加速度ノイズの除去 (EKF 等) |
| 傾斜検知ヒステリシス | 閾値付近でのチャタリング防止 |
| GPS/RTK-GPS | 屋外広域での位置推定 (LiDAR SLAM だけでは不十分) |
| 草刈り刃の制御 | モーター ON/OFF、回転数制御 |

---

## 9. 開発フェーズ進捗

```
Phase 1   ██████████ 完了  基本の障害物回避
Phase 2   ██████████ 完了  賢い回避行動 (壁沿い, U ターン)
Phase 3   ██████████ 完了  SLAM (slam_toolbox)
Phase 4a  ██████████ 完了  Nav2 ナビゲーション基盤
Phase 4b  ██████████ 完了  カバレッジ走行
Phase 4c  ██████████ 完了  カバレッジ発展 (障害物, 地図自動検出)
Phase 4d  ██████████ 完了  バッテリー + ミッション管理
Phase 4e  ██████████ 完了  フルオートノマス・ミッション
Phase 4f  ██████████ 完了  安全性と堅牢性 (衝突防止, 傾斜検知)
Phase 5   ░░░░░░░░░░ 未着手  実機移行
```
