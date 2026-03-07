# Phase 4e / 4f / 5a: フルオートノマス・安全性・実機準備 — 実装計画

## Context

Phase 4d まで完了。ロボットは自律的にカバレッジ走行 → IDLE → 再走行のループが回る。
ただし以下が未実装:
- バッテリー低下時の帰還 → 充電 → **中断地点からの再開**（エンドツーエンド未検証）
- 精密ドッキング (NavigateToPose のみ、充電接点への精密接触なし)
- 安全層 (collision_monitor, 傾斜検知)
- 実機移行の準備 (ros2_control, センサーノイズ)

## Gemini 調査で判明した技術選定

| 項目 | 結論 |
|------|------|
| 優先順位 | ミッション完成 → 安全性 → 実機準備 の順 |
| MPPI | Eigen 移行で 45-50% 高速化 (2025)。4CPU VM では厳しいが batch_size 削減で試行可能 |
| opennav_docking | Jazzy 対応済み。カメラ解像度・FPS を下げれば VM で動作可能 |
| collision_monitor | Jazzy で動的パラメータ + Collision Detector ノード追加。安全層として必須 |
| ros2_control | 実機移行直前 (Phase 5a) で十分。今入れるとシミュレーション固有のチューニングに時間を取られる |
| センサーノイズ | SLAM/Nav2 のロバスト性検証に有効。実機移行前に実施 |

---

## 3 フェーズ分割

```
Phase 4e: フルオートノマス・ミッション完成 (ロジック層)
  ├── カバレッジ再開ロジック (中断地点からの再開)
  └── opennav_docking 精密ドッキング (AprilTag)
        │
        ▼
Phase 4f: 安全性とロバスト性の強化 (セーフティ層)
  ├── collision_monitor 導入 (緊急停止/減速)
  ├── 傾斜検知 (IMU + 安全停止)
  └── 動的障害物テスト (trajectory-system)
        │
        ▼
Phase 5a: 実機アーキテクチャ移行 (ハードウェア抽象化層)
  ├── ros2_control 移行 (gz_ros2_control + diff_drive_controller)
  ├── MPPI コントローラー試行 (オプション)
  └── センサーノイズ注入 (LiDAR/オドメトリ)
```

---

## Phase 4e: フルオートノマス・ミッション完成

### 目標

「放っておけば草を刈り、充電し、また再開する」というコア価値の完成。

### Step 4e-1: カバレッジ再開ロジック

**目的:** バッテリー帰還後、充電完了したら中断地点のウェイポイントから作業を再開する。

**現状の問題:**
- coverage_commander_node は独立プロセスとして起動・終了する
- 中断時のウェイポイント進捗が失われる
- 充電後に再起動すると最初から走行し直す

**設計案:**

```python
# mission_tree_node に追加
@dataclass
class CoverageProgress:
    waypoints: list[Waypoint]       # 全ウェイポイント
    completed_index: int            # 完了済みインデックス
    coverage_params_file: str       # 使用したパラメータファイル

# coverage_commander_node に追加
# 走行進捗を /coverage_progress トピックで配信
# → mission_tree_node が購読して記憶
# → 再起動時に --ros-args -p start_index:=N で中断地点から再開
```

**新規パラメータ (coverage_commander_node):**
- `start_index` (int, default: 0): 開始ウェイポイントのインデックス

**新規トピック:**
- `/coverage_progress` (std_msgs/Int32): 現在のウェイポイントインデックスを配信

**純粋ロジック:**
- `mission_behaviors.py` に `CoverageProgress` dataclass 追加
- テスト: 進捗保存/復元、中断地点からの再開判定

**TDD テスト (6 件):**
- 進捗初期化、インデックス更新、中断・再開判定
- 全完了後の再開 (最初から)
- 進捗リセット条件

### Step 4e-2: opennav_docking 精密ドッキング

**目的:** AprilTag を使って充電ステーションに精密に接近する。

**前提条件:**
- `apt install ros-jazzy-opennav-docking ros-jazzy-apriltag-ros`
- URDF にカメラ搭載済み (`/camera/image_raw`)
- docking_test.world に充電ステーション配置済み

**VM 負荷対策:**
- カメラ解像度: 640×480 (URDF 変更)
- カメラ FPS: 5Hz (Phase 3 で低 FPS 設定済み)
- AprilTag 検出はドッキングフェーズのみ有効化

**新規ファイル:**
- `config/docking_server_params.yaml`: opennav_docking パラメータ
- `launch/docking_launch.py`: AprilTag 検出 + Docking Server 起動
- `worlds/docking_test.world` 更新: AprilTag モデル追加

**ステートマシン変更:**
```
RETURNING 状態の変更:
  旧: NavigateToPose → ホーム到着 → CHARGING
  新: NavigateToPose → 接近ポーズ到着 → DockRobot → ドック完了 → CHARGING
```

**TDD テスト (4 件):**
- ドッキングシーケンスの状態遷移
- ドッキング失敗時のリトライ/フォールバック

### Step 4e-3: 統合テスト (フルサイクル)

**テストシナリオ:**

| # | テスト | 合否基準 |
|---|--------|---------|
| 1 | カバレッジ走行 → 完了 → IDLE | ウェイポイント全完走 |
| 2 | 低バッテリー帰還 → ドッキング → 充電 | ドック到達 + 充電開始 |
| 3 | 充電完了 → 中断地点から再開 | 正しいウェイポイントから再開 |
| 4 | 1 サイクル (走行→帰還→充電→再開) | クラッシュなし |

---

## Phase 4f: 安全性とロバスト性の強化

### Step 4f-1: collision_monitor 導入

**目的:** LiDAR 生データを直接監視し、突発的な障害物に対して緊急停止/減速する安全層。

**構成:**
```
Controller Server ─(/cmd_vel_nav)─→ collision_monitor ─(/cmd_vel)─→ ros_gz_bridge
                                          ↑
                                     /scan (LiDAR 直接監視)
                                     StopZone: ロボット外形 + 3cm → 速度 0
                                     SlowdownZone: ロボット外形 + 20cm → 速度 30%
```

**新規ファイル:**
- `config/collision_monitor_params.yaml`
- `launch/nav2_launch.py` 更新: collision_monitor ノード追加

**TDD テスト (4 件):**
- ゾーン設定の妥当性検証 (純粋ロジック)
- cmd_vel リマッピングの確認

### Step 4f-2: 傾斜検知 (IMU + 安全停止)

**目的:** 急斜面に入った場合にカバレッジを中止して安全地帯へ退避。

**新規:**
- URDF に IMU センサープラグイン追加
- `grass_chopper/incline_monitor.py` (純粋ロジック): pitch/roll 閾値判定
- `grass_chopper/incline_monitor_node.py` (ROS 2): `/imu/data` 購読 → 傾斜検知時に `/cmd_vel` を 0 に上書き
- `worlds/slope_test.world`: 傾斜地テスト環境

**安全閾値:**
- 警告: pitch/roll > 15°
- 緊急停止: pitch/roll > 25°

**TDD テスト (6 件):**
- 傾斜角計算、閾値判定、安全停止トリガー

### Step 4f-3: 動的障害物テスト

**目的:** collision_monitor の動作を実環境に近い条件で検証。

**新規:**
- `worlds/dynamic_obstacles.world`: `gz-sim-trajectory-system` で移動する障害物
- 統合テスト: 動的障害物に対して collision_monitor が正しく停止/減速するか

---

## Phase 5a: 実機アーキテクチャ移行 (将来)

### Step 5a-1: ros2_control 移行

- `gz-sim-diff-drive-system` → `gz_ros2_control` + `diff_drive_controller`
- URDF に `<ros2_control>` タグ追加
- `TwistStamped` 対応 (Jazzy 必須)
- 実機移行時は Hardware Interface のみ差し替え

### Step 5a-2: MPPI コントローラー試行 (オプション)

- RPP で十分であれば不要
- VM では batch_size/time_steps を大幅に削減して軽量テスト

### Step 5a-3: センサーノイズ注入

- LiDAR: Gaussian noise 追加
- オドメトリ: ドリフト注入
- SLAM/Nav2 のロバスト性最終検証

---

## 推奨実装順序

```
Phase 4e:
  4e-1 (カバレッジ再開ロジック)
    │
    ▼
  4e-2 (opennav_docking)
    │
    ▼
  4e-3 (フルサイクル統合テスト)

Phase 4f:
  4f-1 (collision_monitor)
    │
    ▼
  4f-2 (傾斜検知)
    │
    ▼
  4f-3 (動的障害物テスト)

Phase 5a: (実機移行直前)
  5a-1 (ros2_control) → 5a-2 (MPPI) → 5a-3 (ノイズ注入)
```

---

## 新規ファイル一覧 (Phase 4e)

```
grass_chopper/
├── (mission_behaviors.py 変更: CoverageProgress 追加)
├── (mission_tree_node.py 変更: 再開ロジック + DockRobot)
├── (coverage_commander_node.py 変更: start_index + /coverage_progress)
test/
├── (test_mission_behaviors.py 変更: 再開ロジックテスト追加)
config/
├── docking_server_params.yaml         # 4e-2
launch/
├── docking_launch.py                  # 4e-2
worlds/
└── (docking_test.world 変更: AprilTag 追加)  # 4e-2
```

## 新規ファイル一覧 (Phase 4f)

```
grass_chopper/
├── incline_monitor.py                 # 4f-2
├── incline_monitor_node.py            # 4f-2
test/
├── test_incline_monitor.py            # 4f-2
config/
├── collision_monitor_params.yaml      # 4f-1
worlds/
├── slope_test.world                   # 4f-2
└── dynamic_obstacles.world            # 4f-3
```

---

## リスクと対策

| リスク | 対策 |
|--------|------|
| opennav_docking の VM 負荷 | カメラ解像度/FPS を下げる。ドッキング時のみ検出ノード起動 |
| AprilTag が Gazebo Harmonic でレンダリングされない | テクスチャ付き SDF モデルで代替。Gazebo の Material 設定確認 |
| カバレッジ再開時のウェイポイント位置ズレ | 再開時は中断ウェイポイントの手前 (N-1) から開始 |
| collision_monitor と Nav2 の cmd_vel 競合 | twist_mux の優先度設定で collision_monitor を最優先に |
| 傾斜地での SLAM 精度低下 | 2D SLAM の限界として許容。将来は 3D SLAM 検討 |

---

## 検証方法

```bash
# Phase 4e 純粋ロジックテスト
make test-pure   # 既存 155 + 新規約 10 テスト

# Phase 4e 統合テスト (フルサイクル)
make vm-build
make vm-sim-mission && make vm-nav2 && make vm-battery && make vm-mission
# バッテリー消費を速めるパラメータで帰還→充電→再開を検証

# Phase 4f 統合テスト
make vm-sim-dynamic   # 動的障害物ワールド
# collision_monitor の停止/減速を確認
```
