# grass-chopper

ROS 2 + Gazebo で動く**自律草刈りロボット**のシミュレーション環境です。
Mac のブラウザだけで完結します（XQuartz 不要）。

## 何ができるか

シミュレーション上で、草刈りロボットが**完全自律**で動作します:

1. **SLAM で地図を自動生成** — 走行しながら周囲の 2D 地図を作成
2. **カバレッジ走行** — 指定領域をジグザグ (Boustrophedon) パターンで網羅
3. **障害物を自動回避** — LiDAR で検知し、停止・減速・迂回
4. **バッテリー管理** — 残量低下で自動帰還、充電ステーションへドッキング
5. **充電後に自動再開** — 中断地点からカバレッジ走行を再開
6. **傾斜検知** — IMU で危険な傾斜を検知し緊急停止

```
走行開始 → カバレッジ走行 → バッテリー低下 → 自動帰還 → ドッキング → 充電 → 中断地点から再開 → ...
```

### 開発進捗

```
Phase 1   ██████████  基本の障害物回避
Phase 2   ██████████  賢い回避行動 (壁沿い, U ターン)
Phase 3   ██████████  SLAM (slam_toolbox)
Phase 4a  ██████████  Nav2 ナビゲーション基盤
Phase 4b  ██████████  カバレッジ走行
Phase 4c  ██████████  カバレッジ発展 (障害物, 地図自動検出)
Phase 4d  ██████████  バッテリー + ミッション管理
Phase 4e  ██████████  フルオートノマス・ミッション
Phase 4f  ██████████  安全性と堅牢性 (衝突防止, 傾斜検知)
Phase 5   ░░░░░░░░░░  実機移行 (未着手)
```

---

## クイックスタート

### 前提条件

- macOS
- [Multipass](https://multipass.run/) (`brew install multipass`)

### セットアップ

```bash
git clone https://github.com/ymdarake/grass-chopper.git
cd grass-chopper
bash run_sim.sh    # VM 作成 + ROS 2/Gazebo インストール (初回 15〜25分)
```

進捗確認:
```bash
multipass exec ros2-vm -- tail -f /var/log/cloud-init-output.log
```

### シミュレーション起動

```bash
make vm-build       # VM 内でビルド
make vm-sim         # Gazebo + SLAM + Bridge 起動
# 別ターミナルで:
make vm-nav2        # Nav2 スタック起動
```

ブラウザで `http://<VM_IP>:6080/vnc.html` を開くと Gazebo の画面が見えます (パスワード: `ubuntu`)。

> VM の仮想ネットワークにブラウザから直接アクセスできない場合は `make vm-forward` でポートフォワードし、`http://localhost:6080/vnc.html` を使用してください。

### フルオートノマス・ミッション

```bash
make vm-sim-mission   # ドッキングテスト環境起動
make vm-nav2          # Nav2 スタック起動
make vm-battery       # バッテリーシミュレーション起動
make vm-mission       # ミッション管理ノード起動
```

ロボットがカバレッジ走行 → バッテリー低下で自動帰還 → 充電 → 再開を自律的に行います。

---

## アーキテクチャ

```
ホスト (Mac)
├── ソースコード編集 + テスト (make test-pure)
│
│   Multipass マウント (自動同期)
│
└── Multipass VM (Ubuntu 24.04, 4CPU / 8GB RAM)
    │
    ├── [シミュレーション層] Gazebo Harmonic
    │     Physics / DiffDrive / Sensors / IMU
    │          ↕ ネイティブトピック
    │
    ├── [ブリッジ層] ros_gz_bridge
    │     cmd_vel (ROS→GZ), scan/camera/imu/odom/tf/clock (GZ→ROS)
    │          ↕ ROS 2 標準トピック
    │
    ├── [アプリケーション層] ROS 2 ノード群
    │     知覚: slam_toolbox, apriltag_ros
    │     計画: Nav2 (SmacPlanner2D + RPP)
    │     実行: coverage_commander, mission_tree, battery_sim
    │     安全: collision_monitor, incline_monitor
    │
    └── [GUI層] noVNC → ブラウザ (:6080)
```

### 設計原則

- **Gazebo ネイティブプラグイン方式**: Classic Gazebo の `libgazebo_ros_*.so` は使わず、`gz-sim-*-system` + `ros_gz_bridge` で通信
- **Humble Object パターン**: ロジック (8 モジュール) と ROS 2 アダプター (6 ノード) を分離。Mac 上で pytest のみでテスト可能
- **TDD**: テストファースト。189 テストが Mac ホストで即時実行可能

---

## ロボット仕様

### 物理モデル

2輪差動駆動方式。URDF/Xacro で定義。

```
base_link (0.3m × 0.2m × 0.1m, 2.0kg)
├── left_wheel / right_wheel  (半径 0.05m, 車輪間 0.24m)
├── caster_link               (摩擦ゼロ球体)
├── lidar_link                (360° LiDAR, 10Hz)
├── camera_link               (640×480 RGB, 5fps)
└── imu_link                  (加速度+ジャイロ, 50Hz)
```

### 速度指令の優先度

```
cmd_vel_incline (傾斜停止, 最優先)  ─┐
cmd_vel_teleop  (手動, 優先)        ─┼→ twist_mux → collision_monitor → cmd_vel → Gazebo
cmd_vel_nav     (Nav2, 通常)        ─┘
```

### ミッション状態遷移

```
IDLE ──(未完了あり)──→ COVERAGE ──(バッテリー低下)──→ RETURNING
 ↑                                                       │
 │                                                   (到着)
(充電完了)                                               ↓
CHARGING ←──(ドック成功)── DOCKING ←──(ホーム到着)───┘
```

### 安全機能

| 機能 | センサー | 動作 |
|------|---------|------|
| 衝突防止 (Collision Monitor) | LiDAR | 近接障害物で停止/減速 |
| 傾斜検知 (Incline Monitor) | IMU | 25° 以上で緊急停止 |

---

## テスト

```bash
make test-pure       # 純粋ロジックテスト 189件 (Mac, rclpy 不要)
make check-syntax    # Python + YAML 構文チェック
```

### テスト内訳

| テストファイル | 対象 | 件数 |
|-------------|------|------|
| test_obstacle_avoidance.py | 障害物回避ロジック | 41 |
| test_coverage_planner.py | カバレッジ経路計画 | 27 |
| test_mission_behaviors.py | ミッション管理 | 27 |
| test_coverage_tracker.py | カバレッジ率追跡 | 20 |
| test_battery_simulator.py | バッテリーシミュレーション | 18 |
| test_incline_monitor.py | 傾斜検知 | 17 |
| test_map_region_detector.py | 地図自動領域検出 | 13 |
| test_docking_behavior.py | ドッキング行動 | 12 |

### テスト環境 (Gazebo ワールド)

| ワールド | 用途 |
|---------|------|
| obstacles.world | 障害物回避テスト |
| slam_test.world | SLAM ループクロージャ検証 |
| coverage_test.world | カバレッジ走行 (8m×8m) |
| coverage_obstacles.world | 障害物ありカバレッジ |
| docking_test.world | フルミッション (充電ステーション付き) |
| slope_test.world | 傾斜検知 (10°/25° 傾斜面) |
| dynamic_obstacles.world | 動的障害物テスト |

---

## Makefile コマンド一覧

### テスト・チェック

| コマンド | 用途 |
|---------|------|
| `make test-pure` | 純粋ロジックテスト 189件 |
| `make check-syntax` | Python + YAML 構文チェック |

### VM 操作

| コマンド | 用途 |
|---------|------|
| `make vm-build` | VM 内で colcon ビルド |
| `make vm-sim` | Gazebo + SLAM + Bridge 起動 |
| `make vm-sim-headless` | ヘッドレス (GUI なし) 起動 |
| `make vm-nav2` | Nav2 スタック起動 |
| `make vm-battery` | バッテリーシミュレーション起動 |
| `make vm-mission` | ミッション管理ノード起動 |
| `make vm-docking` | AprilTag + opennav_docking 起動 |
| `make vm-incline` | 傾斜検知ノード起動 |
| `make vm-kill` | 全残留プロセス停止 |
| `make vm-stop` | VM 停止 |
| `make vm-forward` | noVNC ポートフォワード |
| `make vm-topics` | ROS 2 トピック一覧 |

### テスト環境別起動

| コマンド | ワールド |
|---------|---------|
| `make vm-sim` | obstacles.world |
| `make vm-sim-nav2` | slam_test.world |
| `make vm-sim-coverage` | coverage_test.world |
| `make vm-sim-obstacles` | coverage_obstacles.world |
| `make vm-sim-mission` | docking_test.world |
| `make vm-sim-slope` | slope_test.world |
| `make vm-sim-dynamic` | dynamic_obstacles.world |

---

## ファイル構成

```
grass-chopper/
├── Makefile                                # ビルド・VM操作コマンド
├── setup.yaml                              # cloud-init (VM自動構築)
├── run_sim.sh                              # VM起動スクリプト
├── docs/
│   ├── specification.md                    # システム仕様書
│   ├── architecture.md                     # アーキテクチャ概要
│   ├── roadmap.md                          # 開発ロードマップ
│   └── technology-decisions.md             # 技術選定理由
└── weeder_ws/src/grass_chopper/            # ROS 2 パッケージ
    ├── grass_chopper/                      # Python モジュール
    │   ├── obstacle_avoidance.py           # 障害物回避ロジック
    │   ├── weeder_node.py                  # 障害物回避 ROS 2 ノード
    │   ├── coverage_planner.py             # カバレッジ経路計画
    │   ├── coverage_commander_node.py      # カバレッジ走行 ROS 2 ノード
    │   ├── coverage_tracker.py             # カバレッジ率追跡
    │   ├── coverage_tracker_node.py        # カバレッジ率 ROS 2 ノード
    │   ├── map_region_detector.py          # 地図自動領域検出
    │   ├── battery_simulator.py            # バッテリーシミュレーション
    │   ├── battery_sim_node.py             # バッテリー ROS 2 ノード
    │   ├── mission_behaviors.py            # ミッション判断ロジック
    │   ├── mission_tree_node.py            # ミッション管理 ROS 2 ノード
    │   ├── docking_behavior.py             # ドッキング行動ロジック
    │   ├── incline_monitor.py              # 傾斜検知ロジック
    │   └── incline_monitor_node.py         # 傾斜検知 ROS 2 ノード
    ├── test/                               # 189 テスト (Mac ホスト実行可)
    ├── config/                             # パラメータ設定 (13 YAML)
    ├── launch/                             # 起動ファイル (3 Python)
    ├── urdf/                               # ロボットモデル (Xacro)
    ├── worlds/                             # テスト環境 (7 SDF)
    └── models/                             # AprilTag マーカーモデル
```

---

## トラブルシューティング

### cloud-init が終わらない

初回は 15〜25分 かかります。進捗確認:
```bash
multipass exec ros2-vm -- tail -f /var/log/cloud-init-output.log
```

### noVNC に接続できない

```bash
multipass exec ros2-vm -- systemctl status vnc-desktop.service
multipass exec ros2-vm -- sudo systemctl restart vnc-desktop.service
```

### Gazebo が黒い画面

```bash
# ソフトウェアレンダリング有効か確認
multipass exec ros2-vm -- bash -c 'echo $LIBGL_ALWAYS_SOFTWARE'
# → 1 が表示されること
```

### DISPLAY 番号の不一致

VNC 再起動後に DISPLAY 番号が変わることがあります:
```bash
multipass exec ros2-vm -- bash -c 'ps aux | grep Xvfb | grep -v grep'
```

### 残留プロセスでシミュレーションが起動しない

```bash
make vm-kill    # 全プロセス一括停止
```

---

## ドキュメント

| ドキュメント | 内容 |
|-----------|------|
| [システム仕様書](docs/specification.md) | 全機能の詳細仕様、トピック一覧、状態遷移図 |
| [アーキテクチャ概要](docs/architecture.md) | レイヤー構成、データフロー、起動シーケンス |
| [ロードマップ](docs/roadmap.md) | 開発フェーズ、実機移行計画 |
| [技術選定の理由](docs/technology-decisions.md) | 各技術の選定理由と比較 |

## 技術スタック

| コンポーネント | バージョン |
|-------------|-----------|
| OS (VM) | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy |
| シミュレーター | Gazebo Harmonic |
| ナビゲーション | Nav2 (SmacPlanner2D + RPP) |
| SLAM | slam_toolbox (online_async) |
| ドッキング | opennav_docking + AprilTag |
| 言語 | Python 3 |

## ライセンス

Apache-2.0
