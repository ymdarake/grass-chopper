# Phase 4c-4 / 4d: 地図自動検出・バッテリー・ミッション管理・ドッキング — 仕様書

## 概要

Phase 4c の残りタスク（地図ベース自動領域検出）と Phase 4d（行動管理）を統合実装した。
ロボットが自律的にカバレッジ走行 → 低バッテリー帰還 → 充電 → 再開のライフサイクルを回せるようになった。

**実装方式:** py_trees_ros の導入を検討したが、MVP ではシンプルなステートマシンを採用。
Nav2 はナビゲーションモジュールとして呼び出す「ハイブリッドパターン」。

---

## アーキテクチャ

```
              mission_tree_node (ステートマシン)
              ├── IDLE: カバレッジ開始判定
              ├── COVERAGE: coverage_commander_node をサブプロセスで起動
              ├── RETURNING: NavigateToPose でホームへ帰還
              └── CHARGING: 満充電待ち
                    │
                    ▼ NavigateToPose (帰還)
              Nav2 スタック (既存)
                    │ /cmd_vel_nav
                    ▼
              twist_mux → ros_gz_bridge → Gazebo

  並行ノード:
  - battery_sim_node    → /battery_state 配信
  - coverage_tracker_node → /coverage_grid, /coverage_ratio 配信
```

### ノード間トピック

```
battery_sim_node ──/battery_state──→ mission_tree_node
mission_tree_node ──/is_charging──→ battery_sim_node
coverage_tracker_node ──/coverage_ratio──→ mission_tree_node
mission_tree_node ──subprocess──→ coverage_commander_node
coverage_commander_node ──NavigateToPose──→ Nav2
mission_tree_node ──NavigateToPose──→ Nav2 (帰還用)
```

---

## Step 4c-4: 地図ベース自動領域検出

### 目的

SLAM `/map` (OccupancyGrid) から走行領域・障害物を自動抽出し、手動座標指定を不要にする。

### モジュール: `map_region_detector.py` (純粋ロジック)

```python
def occupancy_grid_to_binary(grid_data, config, free_threshold=50) -> np.ndarray
def extract_free_regions(binary_mask, config, min_area=1.0, robot_radius=0.2) -> list[Polygon]
def detect_obstacles_from_map(grid_data, config, min_area=0.1) -> list[Polygon]
def select_largest_region(regions) -> Polygon | None
```

**処理パイプライン:**

```
OccupancyGrid (1D int8)
  → 二値化 (free=255 / occupied=0)
  → CLOSE (モルフォロジー穴埋め)
  → erode (robot_radius 分の安全マージン)
  → findContours (RETR_CCOMP: 穴あり対応)
  → ピクセル座標 → ワールド座標変換
  → Shapely Polygon
```

**依存:** OpenCV (cv2), NumPy, Shapely, GridConfig (coverage_tracker.py から再利用)

### ROS 2 アダプター: `coverage_commander_node.py` への追加

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `auto_detect_region` | bool | false | `/map` から自動検出 |
| `auto_detect_robot_radius` | double | 0.2 | 安全マージン [m] |

`auto_detect_region: true` 時、`/map` (TRANSIENT_LOCAL) を購読し、初回受信時に
`extract_free_regions` → `select_largest_region` で走行領域を、
`detect_obstacles_from_map` で障害物を自動検出する。手動パラメータはフォールバック。

### テスト: 13 件

- `occupancy_grid_to_binary`: free/occupied/unknown の変換 (3)
- `extract_free_regions`: 正方形部屋, 穴あき, 複数部屋, min_area, erode, 空 (6)
- `detect_obstacles_from_map`: 障害物あり/なし (2)
- `select_largest_region`: 最大選択, 空リスト (2)

---

## Step 4d-1: バッテリーシミュレーション

### 目的

クーロンカウント方式でバッテリー残量をシミュレーションし、ミッション管理の判断材料にする。

### モジュール: `battery_simulator.py` (純粋ロジック)

```python
@dataclass(frozen=True)
class BatteryParams:
    capacity_ah: float = 5.0        # バッテリー容量 [Ah]
    voltage_full: float = 12.6      # 満充電電圧 [V]
    voltage_empty: float = 10.0     # 空電圧 [V]
    idle_current_a: float = 0.5     # アイドル電流 [A]
    drive_current_a: float = 2.0    # 走行電流 [A]
    mow_current_a: float = 3.0     # 草刈り電流 [A]
    charge_current_a: float = 2.0   # 充電電流 [A]
    low_threshold: float = 0.2      # 低バッテリー閾値
    critical_threshold: float = 0.1 # クリティカル閾値

class BatterySimulator:
    def update(self, dt: float, state: str) -> None   # state: idle/drive/mow/charge
    def get_percentage(self) -> float                   # 0.0 ~ 1.0
    def get_voltage(self) -> float                      # 線形補間
    def is_low(self) -> bool
    def is_critical(self) -> bool
    def is_full(self) -> bool                           # >= 0.999
```

### ROS 2 ノード: `battery_sim_node.py`

| 項目 | 内容 |
|---|---|
| ノード名 | `battery_sim_node` |
| 配信 | `/battery_state` (sensor_msgs/BatteryState) |
| 購読 | `/cmd_vel` (走行判定), `/is_charging` (Bool, 充電状態) |
| 更新頻度 | `update_frequency` Hz (デフォルト 1.0) |

走行判定: `|linear.x| + |angular.z| > 0.01` → drive, それ以外 → idle

### テスト: 14 件

初期満充電, 放電速度比較 (idle < drive < mow), 充電回復, クランプ (0~1),
電圧線形補間, 閾値判定 (low/critical/full), パラメータ frozen

---

## Step 4d-2: ミッション管理

### 目的

バッテリー残量・カバレッジ率に応じてロボットの行動を自律的に切り替える。

### モジュール: `mission_behaviors.py` (純粋ロジック)

```python
@dataclass(frozen=True)
class MissionState:
    battery_percentage: float   # バッテリー残量
    is_charging: bool           # 充電中か
    coverage_ratio: float       # カバレッジ率
    target_coverage: float      # 目標カバレッジ率
    is_coverage_running: bool   # カバレッジ走行実行中か

def should_return_home(state, low_threshold) -> bool   # 低バッテリー + 未充電
def should_start_coverage(state) -> bool               # 充電中でない + 走行中でない + 未達成
def should_continue_charging(state) -> bool            # 充電中 + 未満充電
def compute_home_pose(cx, cy, hx, hy) -> (x, y, yaw)  # ホーム方向の yaw を計算
```

### ROS 2 ノード: `mission_tree_node.py`

| 項目 | 内容 |
|---|---|
| ノード名 | `mission_tree_node` |
| 購読 | `/battery_state` (BatteryState), `/coverage_ratio` (Float32) |
| 配信 | `/is_charging` (Bool) |
| アクションクライアント | `navigate_to_pose` (NavigateToPose) — 帰還用 |
| ティック頻度 | `tick_frequency` Hz (デフォルト 2.0) |

### ステートマシン

```
         ┌──────────────────────────────────────┐
         │                                      │
         ▼                                      │
       IDLE ──(should_start_coverage)──→ COVERAGE
         │                                  │
         │ (should_return_home)              │ (should_return_home)
         ▼                                  ▼
     RETURNING ←─────────────────────── RETURNING
         │
         ├──(成功)──→ CHARGING ──(満充電)──→ IDLE
         └──(失敗)──→ IDLE
```

| 状態 | 処理 |
|---|---|
| IDLE | `should_start_coverage` → COVERAGE へ遷移 + coverage_commander_node 起動 |
| COVERAGE | `should_return_home` → coverage_commander_node 停止 + RETURNING へ遷移。プロセス終了検知 → IDLE |
| RETURNING | NavigateToPose 完了待ち。成功 → CHARGING、失敗 → IDLE |
| CHARGING | `/is_charging=true` 配信。`should_continue_charging=false` → IDLE |

### カバレッジコマンダーの起動方式

`subprocess.Popen` で `ros2 run grass_chopper coverage_commander_node` を起動。
`coverage_commander_params_file` パラメータで設定ファイルを指定可能（`os.path.expanduser` でチルダ展開）。

### テスト: 12 件

帰還判定 (低バッテリー/充電中/通常), カバレッジ開始判定 (充電中/走行中/目標達成/開始),
充電継続判定 (満充電/未満/非充電), ホームポーズ計算 (基本/同位置)

---

## Step 4d-3: ドッキング / 帰還行動 (MVP)

### 目的

低バッテリー時にホーム位置へ帰還する。MVP は NavigateToPose による単純な移動で、
到着 = 充電開始とする。将来は opennav_docking + AprilTag での精密ドッキングへ移行予定。

### モジュール: `docking_behavior.py` (純粋ロジック)

```python
@dataclass(frozen=True)
class DockingParams:
    home_x: float = 0.0             # 充電ステーション x [m]
    home_y: float = 0.0             # 充電ステーション y [m]
    home_yaw: float = 0.0           # ドック正面の方向 [rad]
    approach_distance: float = 0.5  # 接近距離 [m]
    dock_tolerance: float = 0.1     # 到達判定許容誤差 [m]

def compute_approach_pose(params) -> (x, y, yaw)              # 接近ポーズ
def is_at_dock(x, y, params) -> bool                          # ドック到達判定
def compute_dock_alignment_twist(cx, cy, cyaw, params) -> (v, w)  # P 制御アライメント
```

### テストワールド: `docking_test.world`

- 8m × 8m フィールド (coverage_test.world ベース)
- 充電ステーション: ワールド座標 (3.5, -3.5)、緑色 0.4m × 0.4m ボックス
- ロボット初期位置: ワールド座標 (-3.5, -3.5)

### テスト: 8 件

パラメータデフォルト値, frozen 検証, 接近ポーズ (東/北), ドック判定 (at/not),
アライメント (aligned/offset)

---

## 設定ファイル

### `config/battery_params.yaml`

```yaml
battery_sim_node:
  ros__parameters:
    update_frequency: 1.0
    capacity_ah: 5.0
    voltage_full: 12.6
    voltage_empty: 10.0
    idle_current_a: 0.5
    drive_current_a: 2.0
    mow_current_a: 3.0
    charge_current_a: 2.0
    low_threshold: 0.2
    critical_threshold: 0.1
```

### `config/mission_params.yaml`

```yaml
mission_tree_node:
  ros__parameters:
    tick_frequency: 2.0
    home_x: 0.0
    home_y: 0.0
    low_threshold: 0.2
    target_coverage: 0.9
    coverage_commander_params_file: "~/weeder_build/install/grass_chopper/share/grass_chopper/config/coverage_params.yaml"
```

---

## 統合テスト結果

### 起動手順

```bash
make vm-sim-mission   # Gazebo + SLAM + Bridge (docking_test.world)
make vm-nav2          # Nav2 スタック
make vm-battery       # バッテリーシミュレーション
make vm-mission       # ミッション管理
```

### テスト結果

| # | テスト | 結果 |
|---|---|---|
| 1 | カバレッジ走行 (4 ウェイポイント) | 全成功 (各 10〜17 秒で到達) |
| 2 | ミッション状態遷移 (idle → coverage → idle) | 正常動作 |
| 3 | カバレッジ完了後のプロセス終了 | code: 0 で正常終了 |
| 4 | target_coverage 未到達時の自動再開 | 自動で coverage 再開 |
| 5 | バッテリーシミュレーション | 正常放電 (99% → ...) |

### 統合テスト中に発見・修正した問題

| 問題 | 原因 | 修正 |
|---|---|---|
| 全ウェイポイント ABORTED (status 6) | デフォルト region [-3,-3,3,3] がコストマップ範囲外 | mission_params.yaml に coverage_params.yaml のパスを設定 |
| params ファイル読み込みエラー | `~` が subprocess で展開されない | `os.path.expanduser` で展開 |
| カバレッジ完了後プロセスが残留 | `rclpy.spin` が終了しない | `SystemExit(0)` で自己終了 |

---

## ファイル一覧

### 新規ファイル

| ファイル | Step | 説明 |
|---|---|---|
| `grass_chopper/map_region_detector.py` | 4c-4 | 地図自動検出 (純粋ロジック) |
| `test/test_map_region_detector.py` | 4c-4 | テスト 13 件 |
| `grass_chopper/battery_simulator.py` | 4d-1 | バッテリーシミュレーション (純粋ロジック) |
| `grass_chopper/battery_sim_node.py` | 4d-1 | バッテリー ROS 2 ノード |
| `test/test_battery_simulator.py` | 4d-1 | テスト 14 件 |
| `config/battery_params.yaml` | 4d-1 | バッテリーパラメータ |
| `grass_chopper/mission_behaviors.py` | 4d-2 | ミッション判断 (純粋ロジック) |
| `grass_chopper/mission_tree_node.py` | 4d-2 | ミッション管理 ROS 2 ノード |
| `test/test_mission_behaviors.py` | 4d-2 | テスト 12 件 |
| `config/mission_params.yaml` | 4d-2 | ミッションパラメータ |
| `grass_chopper/docking_behavior.py` | 4d-3 | ドッキング行動 (純粋ロジック) |
| `test/test_docking_behavior.py` | 4d-3 | テスト 8 件 |
| `config/docking_params.yaml` | 4d-3 | ドッキングパラメータ |
| `worlds/docking_test.world` | 4d-3 | 充電ステーション付きワールド |

### 変更ファイル

| ファイル | 変更内容 |
|---|---|
| `coverage_commander_node.py` | auto_detect_region, コストマップ待機, SystemExit 追加 |
| `setup.py` | battery_sim_node, mission_tree_node エントリーポイント追加 |
| `package.xml` | py_trees 関連依存追加 |
| `setup.yaml` | opencv-python-headless, ros-jazzy-py-trees-ros 追加 |
| `Makefile` | vm-sim-mission, vm-battery, vm-mission ターゲット追加 |

---

## 純粋ロジックテスト

Mac ホストで全 155 件パス:

```bash
make test-pure   # 0.28 秒
```

| モジュール | テスト件数 |
|---|---|
| test_obstacle_avoidance | 41 |
| test_coverage_planner | 27 |
| test_coverage_tracker | 20 |
| test_map_region_detector | 13 |
| test_battery_simulator | 14 |
| test_mission_behaviors | 12 |
| test_docking_behavior | 8 |
| 既存その他 | 20 |
| **合計** | **155** |
