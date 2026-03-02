# Phase 3: 自己位置推定と地図作成 (SLAM) — 実装計画

## 概要

本フェーズでは、Phase 1 で構築した LiDAR + 差動駆動ロボットのシミュレーション環境に `slam_toolbox` を統合し、ロボットが走行しながら 2D 占有格子地図（Occupancy Grid Map）を自動生成できるようにする。

具体的には以下の 3 つの機能を実現する:

1. **オドメトリ (`/odom`) + LiDAR (`/scan`) を活用したリアルタイム自己位置推定**
2. **`slam_toolbox` (online_async モード) によるフィールドの 2D 地図の自動生成**
3. **生成した地図の保存 (`map_saver_cli`) と、将来の再利用に向けた基盤整備**

Phase 4 (nav2 による自律ナビゲーション) の前提条件となるフェーズであり、ここで生成された地図が Phase 4 のコストマップの入力となる。

## 前提条件

### Phase 1 の完了状態（確認済み）

- LiDAR (`gpu_lidar`) による 360 度スキャンデータの取得: `/scan` トピック
- 差動駆動 (`gz-sim-diff-drive-system`) によるオドメトリ発行: `/odom` トピック (50Hz)
- `robot_state_publisher` による `base_link` → `lidar_link` 等の静的 TF 発行
- `ros_gz_bridge` による ROS 2 ↔ Gazebo のトピックブリッジ

### Phase 2 との関係

Phase 2（賢い回避行動）は Phase 3 の前提条件ではない。slam_toolbox は `/scan` と `/odom` のみに依存するため、Phase 1 の状態からでも SLAM を開始できる。ただし、Phase 2 のパラメータ化が完了していれば、SLAM テスト時の走行制御がより柔軟になる。

### 現在の TF tree 構成（URDF から分析）

```
odom ─(DiffDrive)─> base_link ─(fixed)─> lidar_link
                                ─(fixed)─> camera_link
                                ─(fixed)─> left_wheel_link
                                ─(fixed)─> right_wheel_link
                                ─(fixed)─> caster_link
```

**重要な発見: 現在 `odom → base_link` の TF が ROS 2 側にブリッジされていない。**

`gz-sim-diff-drive-system` は Gazebo 内部トピック `/model/grass_chopper/tf` に `gz.msgs.Pose_V` 型で TF を発行しているが、現在の `sim_launch.py` のブリッジ設定にはこのトピックが含まれていない。`/odom` トピック（`nav_msgs/msg/Odometry`）はブリッジされているが、TF としては発行されていないため、slam_toolbox が要求する `odom → base_link` の TF lookup が失敗する。

## 必要なパッケージと環境構築

### 追加 APT パッケージ

| パッケージ名 | 用途 |
|---|---|
| `ros-jazzy-slam-toolbox` | SLAM アルゴリズム本体 (online_async モード) |
| `ros-jazzy-nav2-map-server` | `map_saver_cli` による地図保存 (PGM + YAML) |
| `ros-jazzy-tf2-tools` | `view_frames` 等による TF tree のデバッグ |

いずれも `ros-jazzy-desktop` には含まれていないため、別途インストールが必要。

### cloud-init (`setup.yaml`) への追加

`runcmd` セクションの既存 `apt-get install` 行の後に追加する:

```yaml
  # Phase 3: SLAM 関連パッケージ
  - apt-get install -y ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server ros-jazzy-tf2-tools
```

### package.xml への依存関係追加

```xml
  <!-- Phase 3: SLAM -->
  <exec_depend>slam_toolbox</exec_depend>
  <exec_depend>nav2_map_server</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
```

## TF Tree 要件

### slam_toolbox が要求する TF フレーム関係

```
map ─(slam_toolbox が発行)─> odom ─(DiffDrive + bridge)─> base_link ─(robot_state_publisher)─> lidar_link
```

| 変換 | 発行元 | 現在の状態 | 対応 |
|---|---|---|---|
| `map` → `odom` | slam_toolbox | 未設定 | Step 3 で slam_toolbox 起動時に自動発行 |
| `odom` → `base_link` | DiffDrive + ros_gz_bridge | **未ブリッジ (要対応)** | Step 1 で TF ブリッジを追加 |
| `base_link` → `lidar_link` | robot_state_publisher | 発行済み | 変更不要 |

### slam_toolbox のフレーム設定パラメータ

```yaml
odom_frame: odom          # DiffDrive の frame_id と一致
map_frame: map            # slam_toolbox が発行する地図フレーム
base_frame: base_link     # DiffDrive の child_frame_id と一致
scan_topic: /scan         # ros_gz_bridge でブリッジ済みのトピック
```

## 実装ステップ

### Step 1: TF ブリッジの追加 (最重要・最優先)

- **目的**: Gazebo の DiffDrive プラグインが発行する `odom → base_link` の TF を ROS 2 側にブリッジし、slam_toolbox が TF lookup できるようにする
- **変更対象ファイル**: `weeder_ws/src/grass_chopper/launch/sim_launch.py`
- **実装内容**:

`bridge` ノードの `arguments` に TF ブリッジを追加し、`remappings` で `/tf` にリマップする:

```python
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            # 既存のブリッジ設定 (変更なし)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # --- Phase 3 追加: TF ブリッジ ---
            '/model/grass_chopper/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/model/grass_chopper/tf', '/tf'),
        ]
    )
```

- **テスト方針**:
  1. シミュレーションを起動し、`ros2 topic echo /tf` で `odom → base_link` の変換が流れていることを確認
  2. `ros2 run tf2_tools view_frames` で TF tree を PDF 出力し、チェーンが繋がっていることを確認
- **完了条件**: `ros2 run tf2_ros tf2_echo odom base_link` が正常に変換を表示すること

### Step 2: slam_toolbox パラメータファイルの作成

- **目的**: slam_toolbox の動作を制御するパラメータ YAML ファイルを作成する
- **変更対象ファイル**: 新規作成 `weeder_ws/src/grass_chopper/config/mapper_params_online_async.yaml`
- **実装内容**:

```yaml
slam_toolbox:
  ros__parameters:
    # === フレーム設定 ===
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan

    # === シミュレーション設定 ===
    use_sim_time: true

    # === 動作モード ===
    mode: mapping
    debug_logging: false
    throttle_scans: 1              # 全スキャンを処理 (LiDAR 10Hz なので間引き不要)

    # === TF 発行設定 ===
    transform_publish_period: 0.02 # map→odom の TF 発行周期 (50Hz)

    # === 地図設定 ===
    resolution: 0.05               # 地図解像度 5cm/pixel
    map_update_interval: 1.0
    max_laser_range: 10.0          # LiDAR の最大有効距離

    # === スキャンマッチング設定 ===
    use_scan_matching: true
    use_scan_barycenter: true
    minimum_travel_distance: 0.3
    minimum_travel_heading: 0.3
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_scan_maximum_distance: 1.5
    link_match_minimum_response_fine: 0.1

    # === ループクロージャ設定 ===
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    # === タイムアウト設定 ===
    transform_timeout: 0.2
    tf_buffer_duration: 30.0

    # === その他 ===
    minimum_time_interval: 0.5
    stack_size_to_use: 40000000
    enable_interactive_mode: true

    # === ソルバー設定 ===
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None
```

- **テスト方針**: `python3 -c "import yaml; yaml.safe_load(open('...'))"` で YAML 構文確認
- **完了条件**: YAML ファイルがパースエラーなく読み込めること

### Step 3: launch ファイルへの slam_toolbox 統合

- **目的**: `sim_launch.py` に slam_toolbox の起動を追加する
- **変更対象ファイル**: `weeder_ws/src/grass_chopper/launch/sim_launch.py`
- **実装内容**:

```python
    # SLAM Toolbox (Phase 3)
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

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
```

- **テスト方針**:
  1. `ros2 topic list` で `/map` トピックが存在すること
  2. `ros2 topic echo /map --once` で OccupancyGrid メッセージが受信できること
  3. `ros2 run tf2_ros tf2_echo map odom` で TF が発行されていること
- **完了条件**: slam_toolbox が起動し、`/map` トピックに地図データが発行されていること

### Step 4: config ディレクトリの setup.py 登録

- **目的**: `config/` ディレクトリのファイルが `colcon build` でインストールされるようにする
- **変更対象ファイル**: `weeder_ws/src/grass_chopper/setup.py`
- **実装内容**: `data_files` に config ディレクトリを追加:
  ```python
  (os.path.join('share', package_name, 'config'),
      glob(os.path.join('config', '*.yaml'))),
  ```
- **完了条件**: ビルド成功後に config ファイルがインストールされること

### Step 5: SLAM テスト用ワールドの作成

- **目的**: ループクロージャを検証できる閉じた環境（回廊型ワールド）を作成する
- **変更対象ファイル**: 新規作成 `weeder_ws/src/grass_chopper/worlds/slam_test.world`
- **実装内容**:

**設計方針:**
- 外枠 10m x 10m、内枠 6m x 6m の「ロの字型」回廊（廊下幅 2.0m）
- 各コーナーに異なる形状のランドマーク配置（非対称にすることで誤認識を防止）
- 直線廊下の中間に凹凸（Hallway Problem 防止）

**ワールド構成:**
```
+---------------------------+
|                           |
|  +-------------------+   |
|  |                   |   |
|  |   (内壁: 6x6)    |●  | ← コーナーB: 円柱
|  |                   |   |
|  +-------------------+   |
|          ■               |
|  ロボット初期位置 (0, -4) |
+---------------------------+
   ▲ コーナーA: 非対称ボックス群
```

主な構成要素:
1. 外周壁 4 枚 (10m x 0.2m x 1.0m)
2. 内壁 4 枚 (6m x 0.2m x 1.0m)
3. コーナーランドマーク 4 個 (各コーナーで異なる形状)
4. 廊下中間の特徴物 2 個 (Hallway Problem 防止)
5. 必須 World プラグイン 4 種

- **完了条件**: ロボットが回廊内を走行し、ループクロージャが検出される地図が生成されること

### Step 6: SLAM 用 launch ファイルの作成（オプション）

- **目的**: SLAM テスト専用の launch ファイルを作成する
- **変更対象ファイル**: 新規作成 `weeder_ws/src/grass_chopper/launch/slam_launch.py`
- **実装内容**: `sim_launch.py` をベースに、ワールドを `slam_test.world` に切り替え、rviz2 の起動を追加
- **完了条件**: rviz2 上で `/map` トピックの地図が表示されること

### Step 7: 地図の保存機能の確認

- **目的**: 生成した地図を PGM + YAML 形式で保存し、再利用できることを確認する
- **実装内容**:

```bash
# 方法 A: nav2_map_server で保存
ros2 run nav2_map_server map_saver_cli -f ~/weeder_ws/maps/slam_test_map

# 方法 B: slam_toolbox のサービスで保存 (シリアライズ形式)
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/ubuntu/weeder_ws/maps/slam_test_map'}"
```

- **完了条件**: PGM + YAML 形式の地図ファイルが正常に保存されること

## パラメータ設定の設計根拠

| パラメータ | 設定値 | 根拠 |
|---|---|---|
| `resolution` | 0.05 (5cm/pixel) | ロボットサイズ (30cm x 20cm) に対して十分な粒度 |
| `max_laser_range` | 10.0 | URDF の LiDAR `range.max` と一致 |
| `throttle_scans` | 1 | LiDAR が 10Hz と低頻度のため間引き不要 |
| `minimum_travel_distance` | 0.3 | ロボット全長 (30cm) と同程度 |
| `minimum_travel_heading` | 0.3 | 約 17 度。回転中の頻繁なスキャン追加を抑制 |
| `transform_publish_period` | 0.02 | DiffDrive の odom_publish_frequency (50Hz) と同等 |
| `do_loop_closing` | true | Phase 3 の主要検証項目 |
| `transform_timeout` | 0.2 | ソフトウェアレンダリング環境での遅延を考慮 |

## テスト戦略

### 1. TF tree の結合テスト (Step 1 完了後)

```bash
ros2 run tf2_tools view_frames        # TF tree の確認
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link lidar_link
```

### 2. slam_toolbox の起動テスト (Step 3 完了後)

```bash
ros2 topic list | grep -E "(map|slam)"
ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /map --once
```

### 3. ループクロージャのテスト (Step 5 完了後)

1. `slam_test.world` でシミュレーションを起動
2. ロボットを回廊を一周させる
3. rviz2 で `/map` を可視化し、以下を確認:
   - 回廊の形状が正しく描画されていること
   - 一周して戻った地点で地図のズレが補正されること
   - 壁の二重線（ゴースト）が発生していないこと

## リスクと未解決事項

### 1. odom → base_link TF ブリッジの動作確認 (リスク: 中)

**問題**: `gz-sim-diff-drive-system` が発行する TF の Gazebo 内部トピック名が `/model/grass_chopper/tf` であることは推定であり、実際のトピック名は Gazebo のバージョンやモデル名によって異なる可能性がある。

**対策**:
- 実装前に `gz topic -l` コマンドで実際のトピック名を確認する
- 代替案として URDF に `<tf_topic>/tf</tf_topic>` を追加する方法もある

### 2. ソフトウェアレンダリング環境でのパフォーマンス (リスク: 中)

**問題**: VM 環境 (`LIBGL_ALWAYS_SOFTWARE=1`) で slam_toolbox + Gazebo + rviz2 を同時に動かすと、CPU 負荷が過大になる可能性がある。

**対策**:
- `throttle_scans` を 2 以上に増やしてスキャン処理を間引く
- `map_update_interval` を 2.0 以上に延ばして地図更新頻度を下げる
- rviz2 は必要なときだけ起動する

### 3. QoS 互換性 (リスク: 低)

**問題**: slam_toolbox と ros_gz_bridge の QoS 不一致の可能性。

**対策**: slam_toolbox は BEST_EFFORT を許容する設計。問題時は `ros2 topic info /scan --verbose` で確認。

### 4. Gazebo の clock と TF のタイムスタンプ同期 (リスク: 低〜中)

**問題**: `/clock` トピックのブリッジ遅延により、TF と `/scan` のタイムスタンプにズレが生じる可能性。

**対策**: `transform_timeout` を 0.2 秒に設定して余裕を持たせている。

### 5. slam_toolbox の ROS 2 Jazzy での安定性 (リスク: 低)

`ros-jazzy-slam-toolbox` パッケージの存在は確認済み。問題発生時は GitHub Issues を確認し、必要に応じてソースビルドを検討。

## 実装順序のまとめ

```
Step 1: TF ブリッジの追加 (sim_launch.py)          ← 最重要・最優先
  ↓
Step 2: slam_toolbox パラメータ YAML 作成
  ↓
Step 3: launch ファイルへの slam_toolbox 統合
  ↓
Step 4: setup.py に config ディレクトリ登録
  ↓
Step 5: SLAM テスト用ワールド作成
  ↓
Step 6: SLAM 用 launch ファイル作成 (オプション)
  ↓
Step 7: 地図保存機能の確認
```

## 参考情報

- slam_toolbox GitHub: https://github.com/SteveMacenski/slam_toolbox
- slam_toolbox パラメータリファレンス: `/opt/ros/jazzy/share/slam_toolbox/config/`
- ros_gz_bridge TF ブリッジ: `gz.msgs.Pose_V` ↔ `tf2_msgs/msg/TFMessage`
- nav2_map_server: https://docs.nav2.org/configuration/packages/configuring-map-server.html
- Gazebo Harmonic DiffDrive: https://gazebosim.org/api/sim/latest/classgz_1_1sim_1_1systems_1_1DiffDrive.html
