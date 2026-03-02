# Phase 4: 経路計画と自律ナビゲーション (nav2) — 実装計画

## 概要

Phase 3 (SLAM) で生成・保存した 2D 地図を活用し、nav2 (Navigation2) フレームワークを導入して、地図上の目標地点への自律移動、コストマップベースの障害物回避、複数ウェイポイントの巡回走行、動的障害物への対応を実現する。さらに、草刈りロボット特有のカバレッジプランニング（領域網羅走行）のプロトタイプを Boustrophedon パターンのウェイポイント生成として実装する。

Phase 2 の `weeder_node` による単純な反射的回避ロジックは、nav2 のコストマップ + コントローラーに完全に置き換える。

## 前提条件

Phase 3 の完了により、以下が利用可能であること:

- **静的地図ファイル**: `slam_toolbox` で生成した 2D 占有格子地図 (`.pgm` + `.yaml`)
- **TF チェーン**: `odom` → `base_link` が ROS 側で発行されている状態
- **ros_gz_bridge**: `/cmd_vel`, `/scan`, `/odom`, `/clock`, `/camera/image_raw`, `/joint_states` がブリッジ済み

## 必要なパッケージと環境構築

### apt パッケージ (setup.yaml の runcmd に追加)

```bash
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install -y ros-jazzy-robot-localization
sudo apt install -y ros-jazzy-twist-mux    # cmd_vel 競合管理用
pip3 install shapely                        # カバレッジパターン生成用
```

### cloud-init (setup.yaml) への追加行

```yaml
runcmd:
  - apt-get install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-robot-localization ros-jazzy-twist-mux
  - sudo -u ubuntu pip3 install shapely
```

### package.xml への依存追加

```xml
<exec_depend>navigation2</exec_depend>
<exec_depend>nav2_bringup</exec_depend>
<exec_depend>robot_localization</exec_depend>
<exec_depend>nav2_msgs</exec_depend>
<exec_depend>nav_msgs</exec_depend>
```

## nav2 アーキテクチャ概要

```
                    ┌─────────────────────┐
                    │  BT Navigator       │ ← ナビゲーションの司令塔
                    │  (Behavior Tree)    │
                    └─────────┬───────────┘
                              │
          ┌───────────────────┼───────────────────┐
          ▼                   ▼                   ▼
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│ Planner Server  │ │Controller Server│ │ Behavior Server │
│ SmacPlanner2D   │ │ RPP Controller  │ │ Spin / Backup / │
│ A→B の最適経路  │ │ cmd_vel を発行  │ │ Wait / ClearMap │
└─────────────────┘ └─────────────────┘ └─────────────────┘
          │                   │
          ▼                   ▼
┌─────────────────┐ ┌─────────────────┐
│ Global Costmap  │ │ Local Costmap   │
│ static + obs +  │ │ rolling window  │
│ inflation layer │ │ obs + inflation │
└─────────────────┘ └─────────────────┘

追加コンポーネント:
┌─────────────────┐ ┌─────────────────┐ ┌─────────────────┐
│ Map Server      │ │ AMCL            │ │ Waypoint        │
│ (.pgm地図提供)   │ │ (map→odom TF)   │ │ Follower        │
└─────────────────┘ └─────────────────┘ └─────────────────┘

Lifecycle Manager が全ノードの起動順序と状態管理を統括
```

### 各コンポーネントの役割

| コンポーネント | 役割 | 本プロジェクトでの設定 |
|---|---|---|
| **BT Navigator** | BT XML に従って各サーバーを呼び出し | デフォルト BT XML を使用 |
| **Planner Server** | スタート→ゴールの大局的経路を計算 | SmacPlanner2D |
| **Controller Server** | 経路に沿って `/cmd_vel` を発行 | RegulatedPurePursuit (RPP) |
| **Behavior Server** | スタック時の回復動作 | Spin, Backup, Wait |
| **Map Server** | Phase 3 の静的地図を配信 | `.pgm` + `.yaml` |
| **AMCL** | 静的地図 + LiDAR で `map` → `odom` TF を発行 | パーティクルフィルタ |
| **Waypoint Follower** | 複数目的地の順次巡回 | カバレッジパターン経路の実行 |

## 実装ステップ

### Step 1: nav2 パラメータファイル (YAML) の作成

**ファイル**: `weeder_ws/src/grass_chopper/config/nav2_params.yaml`

nav2 の全コンポーネントのパラメータを1つの YAML に集約。本ロボット (0.3m x 0.2m, 差動駆動, LiDAR 360度/10Hz) に最適化し、ソフトウェアレンダリング環境での低計算負荷にも配慮する。

詳細は後述の「パラメータ設定」セクションに記載。

### Step 2: nav2 用 launch ファイルの作成

**ファイル**: `weeder_ws/src/grass_chopper/launch/nav2_launch.py`

```
起動順序:
1. sim_launch.py (既存 + ekf_node)
   → Gazebo, robot_state_publisher, spawn, bridge, ekf_node
2. nav2_launch.py (新規)
   → map_server, amcl, planner_server, controller_server,
     behavior_server, bt_navigator, waypoint_follower,
     lifecycle_manager
```

`nav2_bringup` の `bringup_launch.py` をインクルードする形で実装:

```python
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
    ),
    launch_arguments={
        'map': map_yaml_file,
        'params_file': nav2_params_file,
        'use_sim_time': 'true',
        'autostart': 'true',
    }.items()
)
```

### Step 3: TF チェーンの完成確認

Phase 3 で以下の TF チェーンが構築されている前提:

```
map ─(AMCL)─→ odom ─(ekf_node)─→ base_link ─(robot_state_publisher)─→ lidar_link
```

**重要**: GZ 内部の TF を ros_gz_bridge でブリッジしては**いけない**。`robot_state_publisher` や `ekf_node` と競合する。

### Step 4: weeder_node の廃止と nav2 への移行

| Phase 2 (weeder_node) | Phase 4 (nav2) |
|---|---|
| 前方 ±30度 の距離チェック | local_costmap (obstacle_layer) — 360度全方位 |
| 0.5m 安全距離で左回転 | inflation_layer + RPP コントローラー |
| 直進のみ | SmacPlanner2D + RPP — 目的地への最適経路 |
| `/cmd_vel` 直接発行 | Controller Server が `/cmd_vel` を発行 |

**移行手順**:
1. `sim_launch.py` から `weeder_node` の起動を削除
2. nav2 の Controller Server が `/cmd_vel` を発行
3. `weeder_node.py` はファイルとして残し、将来の草刈り制御ノードへの転用に備える
4. 安全距離の強制は `nav2_collision_monitor` で実現

### Step 5: 基本的な自律移動のテスト

```bash
# rviz2 の "2D Goal Pose" でマップ上をクリック、または:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 1.0, z: 0.0}}}}"
```

### Step 6: ウェイポイント巡回ノードの実装

**ファイル**: `weeder_ws/src/grass_chopper/grass_chopper/waypoint_commander.py`

nav2 の `FollowWaypoints` アクションを呼び出して複数地点を巡回するノード。

### Step 7: カバレッジパターン（Boustrophedon）ウェイポイント生成

**ファイル**: `weeder_ws/src/grass_chopper/grass_chopper/coverage_planner.py`

Shapely ライブラリで矩形領域に対する Boustrophedon (牛耕式) パターンのウェイポイントリストを生成。

**アルゴリズム概要**:
1. 対象領域を `Polygon` として定義
2. ロボットの作業幅 (0.2m) ごとに平行な直線を生成
3. 各直線と Polygon の交差を計算
4. 偶数行は左→右、奇数行は右→左と交互に反転してジグザグ経路を形成
5. 各端点を `PoseStamped` に変換

### Step 8: 動的障害物の追加

**ファイル**: `weeder_ws/src/grass_chopper/worlds/obstacles_dynamic.world`

`gz-sim-trajectory-system` プラグインを使った移動障害物を追加:

```xml
<plugin filename="gz-sim-trajectory-system"
        name="gz::sim::systems::Trajectory">
  <link_name>link</link_name>
  <loop>true</loop>
  <waypoint><time>0.0</time><pose>3.0 -2.0 0.25 0 0 0</pose></waypoint>
  <waypoint><time>5.0</time><pose>3.0  2.0 0.25 0 0 0</pose></waypoint>
  <waypoint><time>10.0</time><pose>3.0 -2.0 0.25 0 0 0</pose></waypoint>
</plugin>
```

### Step 9: collision_monitor の導入 (安全レイヤー)

草刈りロボットは刃を回転させるため、安全性が最重要。コストマップの更新遅延を補完するため、LiDAR データを直接監視する `nav2_collision_monitor` を導入。

**cmd_vel のデータフロー (Phase 4)**:

```
Controller Server ─(/cmd_vel_nav)─→ collision_monitor ─(/cmd_vel)─→ ros_gz_bridge → Gazebo
                                          ↑
                                     /scan (LiDAR 直接監視)
                                     緊急時: 速度を 0 に上書き
```

## パラメータ設定

### 主要パラメータ抜粋

| カテゴリ | パラメータ | 値 | 根拠 |
|---|---|---|---|
| AMCL | max_particles | 2000 | 低負荷 (デフォルト5000) |
| Controller | controller_frequency | 10.0 Hz | LiDAR レートに合わせて低負荷化 |
| RPP | desired_linear_vel | 0.3 m/s | 草刈り巡航速度 |
| RPP | lookahead_dist | 0.6 m | ロボット 2 台分先読み |
| Planner | SmacPlanner2D | tolerance: 0.125m | ゴール許容誤差 |
| Local Costmap | width/height | 3.0m | ロボット周囲 3m のウィンドウ |
| Costmap | resolution | 0.05m | 地図解像度と統一 |
| Costmap | footprint | 0.3m x 0.2m | ロボット実寸 |
| Inflation | inflation_radius | 0.55m | ロボット対角半径 + 安全マージン |
| collision_monitor | StopZone | ロボット外形 + 3cm | 即座に停止 |
| collision_monitor | SlowdownZone | ロボット外形 + 20cm | 速度 30% に制限 |

## コストマップ設計

### 二層構造

- **Global Costmap**: static_layer + obstacle_layer + inflation_layer, 更新 1Hz
  - 用途: Planner Server (大局経路計算)
- **Local Costmap**: obstacle_layer + inflation_layer, rolling window 3m x 3m, 更新 5Hz
  - 用途: Controller Server (局所回避 + 経路追従)

### inflation_radius の算出根拠

- ロボット対角線長: `sqrt(0.3^2 + 0.2^2) / 2 = 0.18m`
- 安全マージン: `0.37m`
- `inflation_radius = 0.55m`

## ウェイポイント巡回設計

```
┌─────────────────────┐    ウェイポイントリスト    ┌─────────────────────┐
│ coverage_planner.py │ ──────────────────────→ │waypoint_commander.py│
│ (Boustrophedon      │    [PoseStamped, ...]   │ (FollowWaypoints    │
│  パターン生成)       │                         │  Action Client)     │
└─────────────────────┘                         └──────────┬──────────┘
                                                           │
                                                           ▼
                                                ┌─────────────────────┐
                                                │ nav2 Waypoint       │
                                                │ Follower Server     │
                                                └─────────────────────┘
```

## テスト戦略

### ユニットテスト

| テスト対象 | テスト内容 |
|---|---|
| `coverage_planner.py` | 正しいウェイポイント数、領域外不出、ジグザグ順序 |
| `waypoint_commander.py` | PoseStamped の Yaw 角が進行方向を向いているか |

### 統合テスト (シミュレーション)

| テストシナリオ | 合否基準 |
|---|---|
| 単一ゴールへの移動 | 目標地点から 0.2m 以内に到達 |
| 静的障害物の回避 | 衝突せずに到達 |
| 動的障害物の回避 | 衝突せずに到達 |
| Boustrophedon 巡回 | カバレッジ率 80% 以上 |
| スタックからの回復 | Spin/Backup で脱出、ゴール到達 |
| 長時間巡回 | 10分以上でクラッシュ・メモリリークなし |

### プランナー比較テスト

| 比較項目 | SmacPlanner2D + RPP | NavFn + DWB | SmacPlanner2D + MPPI |
|---|---|---|---|
| 経路の滑らかさ | - | - | - |
| 動的障害物への反応速度 | - | - | - |
| 計算負荷 (CPU 使用率) | - | - | - |
| カバレッジ経路追従の精度 | - | - | - |

## 新規ファイル一覧

```
weeder_ws/src/grass_chopper/
├── config/nav2_params.yaml                  # nav2 パラメータ設定
├── maps/                                    # Phase 3 で作成
│   ├── obstacle_world.pgm
│   └── obstacle_world.yaml
├── grass_chopper/
│   ├── waypoint_commander.py                # ウェイポイント巡回ノード
│   └── coverage_planner.py                  # Boustrophedon 生成
├── launch/nav2_launch.py                    # nav2 スタック起動
├── worlds/
│   ├── obstacles_dynamic.world              # 動的障害物テスト用
│   └── coverage_test.world                  # カバレッジ巡回テスト用
└── test/
    ├── test_coverage_planner.py
    └── test_waypoint_commander.py
```

## リスクと未解決事項

### 高リスク

| リスク | 対策案 |
|---|---|
| **ソフトウェアレンダリング環境での計算負荷** | コストマップ更新頻度を下げる、max_particles を減らす、VM を 6CPU/12GB に増設検討 |
| **TF チェーンの不整合** | Phase 3 で ekf_node を確実に動作させてから Phase 4 に進む。`view_frames` で確認 |
| **gz-sim-trajectory-system の衝突特性** | キネマティック移動のため貫通の可能性。センサーベースの回避を前提とする |

### 中リスク

| リスク | 対策案 |
|---|---|
| **AMCL の初期位置推定失敗** | `set_initial_pose: true` で既知の初期位置を設定 |
| **RPP のカバレッジ経路追従精度** | `lookahead_dist` を短くするか MPPI に切り替え |
| **Shapely ベースの Boustrophedon の限界** | プロトタイプは矩形のみ。将来的に `opennav_coverage` 検討 |

### 未解決事項

| 項目 | 調査タイミング |
|---|---|
| opennav_coverage の Jazzy 対応状況 | Shapely プロトタイプの限界が見えた段階 |
| カバレッジ率の定量評価方法 | Step 7-8 の実装時 |
| nav2 の Behavior Tree カスタマイズ | 統合テスト時 |
| collision_monitor と controller_server の cmd_vel リマップ | Step 9 実装時 |
| VM リソースの十分性 (4CPU/8GB) | Phase 4 初期テスト時 |

## 参考情報

- [Nav2 公式ドキュメント](https://docs.nav2.org/)
- [Nav2 Configuration Guide](https://docs.nav2.org/configuration/)
- [Regulated Pure Pursuit Controller](https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html)
- [SmacPlanner2D](https://docs.nav2.org/configuration/packages/configuring-smac-planner.html)
- [nav2_collision_monitor](https://docs.nav2.org/configuration/packages/configuring-collision-monitor.html)
- [opennav_coverage (GitHub)](https://github.com/open-navigation/opennav_coverage)
- [Fields2Cover (GitHub)](https://github.com/Fields2Cover/Fields2Cover)
- [Gazebo Harmonic Trajectory System](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1Trajectory.html)
- [robot_localization (wiki)](https://docs.ros.org/en/jazzy/p/robot_localization/)
