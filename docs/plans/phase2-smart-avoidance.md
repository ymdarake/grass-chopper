# Phase 2: 賢い回避行動 — 実装計画

## 概要

Phase 1 では「前方に障害物があれば常に左回転」という単純なリアクティブ制御を実装した。Phase 2 では以下の 5 つの改善を行い、より実用的な自律走行の基盤を構築する。

1. **制御パラメータの ROS 2 パラメータ化** — `declare_parameter` で外部 YAML から設定変更可能にする
2. **左右回避方向の選択** — LiDAR の左右空間を比較し、空いている方に回避する
3. **壁沿い走行 (Wall Following)** — PD 制御で障害物との距離を一定に保ちながら前進する
4. **行き止まり検知と U ターン** — 三方が塞がれた状態を検知し、その場回転で脱出する
5. **複数障害物の連続回避** — 状態遷移マシンにより上記行動をシームレスに切り替える

全ての機能は TDD (Red-Green-Refactor) で開発し、制御ロジックをコールバック関数から分離してテスタビリティを確保する。

## 前提条件

- Phase 1 が完了済み: `weeder_node.py` が `/scan` を購読し `/cmd_vel` を発行する基本動作が動いている
- LiDAR 仕様: 360度 / 360サンプル / 10Hz / 0.12-10.0m (`gpu_lidar`)
  - `angle_min = -pi`, `angle_max = pi`, `angle_increment = pi/180`
  - インデックスと角度の対応: `0 = -180度(後方)`, `90 = -90度(右)`, `180 = 0度(正面)`, `270 = +90度(左)`
- 駆動: 差動駆動 (`gz-sim-diff-drive-system`)
- QoS: `BEST_EFFORT`
- テストフレームワーク: `pytest` (ament_python 標準)
- 既存テストディレクトリ: なし (Phase 2 で `test/` ディレクトリを新規作成)

## 実装ステップ

### Step 1: テスト基盤の構築と既存ロジックのテスト作成

- **目的**: TDD の土台を作る。既存の Phase 1 ロジック(前方スキャン + 左回転回避)に対するテストを先に書き、リファクタリングの安全ネットとする。
- **変更対象ファイル**:
  - `weeder_ws/src/grass_chopper/test/__init__.py` (新規作成・空ファイル)
  - `weeder_ws/src/grass_chopper/test/test_weeder_node.py` (新規作成)
  - `weeder_ws/src/grass_chopper/setup.py` (`tests_require=['pytest']` は既存)
- **実装内容**:
  - `pytest` fixture で `rclpy.init()` / `rclpy.shutdown()` のライフサイクルを管理 (`scope="module"`)
  - `LaserScan` メッセージを生成するヘルパー関数 `create_laser_scan(distances, angle_min=-pi, angle_max=pi)` を作成
    - `distances`: 360要素の距離リスト or 単一 float(全方位同一距離)
  - `cmd_pub.publish` を `unittest.mock.MagicMock` でモック化
  - Phase 1 のテストケース:
    - 全方位 1.0m (安全) → 直進 (`linear.x=0.2, angular.z=0.0`)
    - 前方 0.3m (危険) → 左回転 (`linear.x=0.0, angular.z=0.5`)
    - 空の `ranges` → 何もパブリッシュしない or 安全とみなす
    - `inf` / `NaN` 混在データ → 正しくフィルタリング
- **テスト方針**: テストを先に書き、現在の `weeder_node.py` で全て Green になることを確認
- **完了条件**: `colcon test --packages-select grass_chopper` で全テスト Green

### Step 2: 制御パラメータの ROS 2 パラメータ化

- **目的**: ハードコードされた `safe_distance`, `forward_speed`, `turn_speed` を `declare_parameter` で外部設定可能にする。シミュレーション中の `ros2 param set` による動的変更にも対応する。
- **変更対象ファイル**:
  - `weeder_ws/src/grass_chopper/grass_chopper/weeder_node.py`
  - `weeder_ws/src/grass_chopper/config/weeder_params.yaml` (新規作成)
  - `weeder_ws/src/grass_chopper/setup.py` (`data_files` に `config/` を追加)
  - `weeder_ws/src/grass_chopper/launch/sim_launch.py` (パラメータファイルの読み込み追加)
  - `weeder_ws/src/grass_chopper/test/test_weeder_node.py` (テスト追加)
- **実装内容**:
  - `__init__` 内で `declare_parameter` を使用し、`ParameterDescriptor` で型と説明を記述:
    ```python
    from rcl_interfaces.msg import ParameterDescriptor, ParameterType

    safe_distance_desc = ParameterDescriptor(
        type=ParameterType.PARAMETER_DOUBLE,
        description='障害物検知の安全距離 [m]'
    )
    self.declare_parameter('safe_distance', 0.5, safe_distance_desc)
    self.safe_distance = self.get_parameter('safe_distance').value
    ```
  - `add_on_set_parameters_callback` で動的パラメータ変更コールバックを登録
  - YAML パラメータファイル (`config/weeder_params.yaml`):
    ```yaml
    weeder_node:
      ros__parameters:
        safe_distance: 0.5
        forward_speed: 0.2
        turn_speed: 0.5
    ```
  - `setup.py` の `data_files` に追加:
    ```python
    (os.path.join('share', package_name, 'config'),
        glob(os.path.join('config', '*.yaml'))),
    ```
  - `sim_launch.py` の `weeder_node` ノード定義に `parameters` を追加:
    ```python
    config_file = os.path.join(pkg_share, 'config', 'weeder_params.yaml')
    weeder_node = Node(
        package='grass_chopper',
        executable='weeder_node',
        parameters=[config_file, {'use_sim_time': True}]
    )
    ```
- **テスト方針** (TDD):
  - Red: パラメータ値が `declare_parameter` で取得できることを検証するテストを書く
  - Red: カスタムパラメータ値でノード生成し、その値が行動に反映されることを検証するテストを書く
  - Green: `declare_parameter` を実装して通す
  - Refactor: コールバック内のパラメータ取得をプロパティアクセスに統一
- **完了条件**:
  - `ros2 param list /weeder_node` で 3 つのパラメータが表示される
  - `ros2 param set /weeder_node safe_distance 1.0` で実行中に値を変更できる
  - YAML ファイルからの起動時読み込みが動作する
  - 全テスト Green

### Step 3: LiDAR ゾーン解析の抽出とリファクタリング

- **目的**: `scan_callback` に密結合している LiDAR データ解析ロジックを独立したメソッドに抽出し、Step 4 以降の状態遷移マシンで利用可能にする。
- **変更対象ファイル**:
  - `weeder_ws/src/grass_chopper/grass_chopper/weeder_node.py`
  - `weeder_ws/src/grass_chopper/test/test_weeder_node.py`
- **実装内容**:
  - 以下のゾーン解析メソッドを `WeederNode` に追加:
    ```python
    def get_zone_stats(self, ranges: list[float], start_deg: float, end_deg: float,
                       range_min: float, range_max: float) -> tuple[float, float]:
        """
        指定角度範囲の最小距離と平均距離を返す

        Args:
            ranges: LaserScan.ranges 配列
            start_deg: ゾーン開始角度 [度] (-180 ~ 180)
            end_deg: ゾーン終了角度 [度]
            range_min: 有効最小距離
            range_max: 有効最大距離
        Returns:
            (min_distance, avg_distance) のタプル。有効データなしの場合は (inf, inf)
        """
    ```
  - ゾーンの定義:
    - **正面 (FRONT)**: -30度 ~ +30度
    - **左側 (LEFT)**: +60度 ~ +120度
    - **右側 (RIGHT)**: -120度 ~ -60度
  - 角度からインデックスへの変換ユーティリティ:
    ```python
    def _deg_to_index(self, deg: float, angle_min: float, angle_increment: float) -> int:
        """角度[度]をranges配列のインデックスに変換"""
        rad = math.radians(deg)
        return int((rad - angle_min) / angle_increment)
    ```
  - 既存の `scan_callback` 内の前方範囲計算を `get_zone_stats` を使うようにリファクタリング
- **テスト方針** (TDD):
  - Red: `get_zone_stats` に対するテストを先に書く
    - 正面ゾーンの最小距離/平均距離が正しく計算されること
    - `inf`/`NaN` がフィルタリングされること
    - 有効データなしの場合に `(inf, inf)` が返ること
    - 左右ゾーンの計算が正しいこと
  - Red: `_deg_to_index` の変換テスト
  - Green: メソッドを実装
  - Refactor: `scan_callback` を `get_zone_stats` で書き直し、Step 1 のテストが全て通ることを確認
- **完了条件**: 全テスト Green かつ既存動作に変更なし (リグレッションなし)

### Step 4: 状態遷移マシンの導入と左右回避方向の選択

- **目的**: `enum` ベースの状態遷移マシンを導入し、Phase 1 の「常に左回転」を「空いている方向に回避」にアップグレードする。
- **変更対象ファイル**:
  - `weeder_ws/src/grass_chopper/grass_chopper/weeder_node.py`
  - `weeder_ws/src/grass_chopper/test/test_weeder_node.py`
- **実装内容**:
  - 状態の定義:
    ```python
    from enum import Enum, auto

    class RobotState(Enum):
        FORWARD = auto()      # 前方クリア: 直進
        AVOID_LEFT = auto()   # 左が空いている: 左回転回避
        AVOID_RIGHT = auto()  # 右が空いている: 右回転回避
        WALL_FOLLOW = auto()  # 壁沿い走行 (Step 5 で実装)
        U_TURN = auto()       # 行き止まり: Uターン (Step 6 で実装)
    ```
  - 状態遷移ロジック (`_update_state` メソッド):
    ```
    現在の状態に関わらず:
      if 行き止まり (前方 < safe_dist AND 左 < safe_dist AND 右 < safe_dist):
        → U_TURN
      elif 前方に障害物 (front_min < safe_dist):
        if left_avg > right_avg:
          → AVOID_LEFT
        else:
          → AVOID_RIGHT
      elif 回避中 AND 前方クリア (front_min > safe_dist * 1.2):
        → FORWARD
      else:
        → 現在の状態を維持
    ```
  - 行動の実行 (`_compute_twist` メソッド):
    ```
    FORWARD:     linear.x = forward_speed,  angular.z = 0.0
    AVOID_LEFT:  linear.x = 0.0,            angular.z = +turn_speed
    AVOID_RIGHT: linear.x = 0.0,            angular.z = -turn_speed
    ```
  - `scan_callback` の構造:
    ```python
    def scan_callback(self, msg):
        zones = self._analyze_zones(msg)      # Step 3 のゾーン解析
        self._update_state(zones)              # 状態遷移
        twist = self._compute_twist(zones)     # 行動決定
        self.cmd_pub.publish(twist)
    ```
- **テスト方針** (TDD):
  - Red: 左が空いている場合に `AVOID_LEFT` に遷移するテスト
  - Red: 右が空いている場合に `AVOID_RIGHT` に遷移するテスト
  - Red: 回避後に前方クリアで `FORWARD` に復帰するテスト
  - Red: `_compute_twist` が各状態に正しい Twist を生成するテスト
  - Green: 状態遷移マシンを実装
  - Refactor: 遷移ロジックの条件式を整理
- **完了条件**:
  - 全テスト Green
  - Gazebo 上で obstacle_1 (正面) に対して左右どちらかに回避する動作を確認

### Step 5: 壁沿い走行 (Wall Following)

- **目的**: 右側に壁がある場合に PD 制御で一定距離を保ちながら前進する。
- **変更対象ファイル**:
  - `weeder_ws/src/grass_chopper/grass_chopper/weeder_node.py`
  - `weeder_ws/src/grass_chopper/config/weeder_params.yaml` (PD パラメータ追加)
  - `weeder_ws/src/grass_chopper/test/test_weeder_node.py`
- **実装内容**:
  - 追加パラメータ:
    ```yaml
    weeder_node:
      ros__parameters:
        wall_target_distance: 0.5    # 壁との目標距離 [m]
        wall_follow_kp: 1.5          # 比例ゲイン
        wall_follow_kd: 0.3          # 微分ゲイン
        wall_follow_speed: 0.15      # 壁沿い時の前進速度 [m/s]
        angular_z_limit: 0.8         # 角速度の上限 [rad/s]
    ```
  - 壁沿い走行の遷移条件:
    ```
    FORWARD 状態で:
      if 右側最小距離 < wall_target_distance * 1.5 AND 前方クリア:
        → WALL_FOLLOW
    WALL_FOLLOW 状態で:
      if 前方に障害物:
        → AVOID_LEFT or AVOID_RIGHT
      if 右側に壁がなくなった (right_min > wall_target_distance * 2.0):
        → FORWARD
    ```
  - PD 制御ロジック (`_compute_wall_follow_twist` メソッド):
    ```python
    def _compute_wall_follow_twist(self, right_min: float) -> Twist:
        error = self.wall_target_distance - right_min
        p_term = self.wall_follow_kp * error
        d_term = self.wall_follow_kd * (error - self.prev_wall_error)
        self.prev_wall_error = error

        twist = Twist()
        twist.linear.x = self.wall_follow_speed
        twist.angular.z = max(-self.angular_z_limit,
                              min(self.angular_z_limit, p_term + d_term))
        return twist
    ```
    - 壁に近づきすぎ (error > 0): `angular.z > 0` (左に曲がって壁から離れる)
    - 壁から離れすぎ (error < 0): `angular.z < 0` (右に曲がって壁に近づく)
- **テスト方針** (TDD):
  - Red: 壁との距離が目標値ちょうどの場合 → `angular.z` がほぼ 0
  - Red: 壁に近すぎる場合 → `angular.z > 0` (左へ)
  - Red: 壁から遠すぎる場合 → `angular.z < 0` (右へ)
  - Red: 角速度リミットが正しく適用されるテスト
  - Red: FORWARD → WALL_FOLLOW への遷移条件テスト
  - Red: WALL_FOLLOW → FORWARD (壁がなくなった) への遷移テスト
  - Green: PD 制御と状態遷移を実装
- **完了条件**:
  - 全テスト Green
  - Gazebo 上で obstacle_wall (6m 前方の壁) に沿って走行する動作を確認

### Step 6: 行き止まり検知と U ターン

- **目的**: 前方/左/右の全てが塞がれた状態を検知し、その場回転で脱出する。
- **変更対象ファイル**:
  - `weeder_ws/src/grass_chopper/grass_chopper/weeder_node.py`
  - `weeder_ws/src/grass_chopper/config/weeder_params.yaml` (Uターン速度追加)
  - `weeder_ws/src/grass_chopper/test/test_weeder_node.py`
- **実装内容**:
  - 追加パラメータ:
    ```yaml
    u_turn_speed: 1.0   # Uターン時の回転速度 [rad/s]
    ```
  - 行き止まり検知条件:
    ```
    front_min < safe_distance AND left_min < safe_distance AND right_min < safe_distance
    ```
  - U ターン行動:
    ```
    U_TURN 状態:
      linear.x = 0.0
      angular.z = u_turn_speed  # 左方向に高速回転

    U_TURN → FORWARD の遷移条件:
      front_min > safe_distance * 1.5  # 前方が十分に開けたら
    ```
  - U_TURN は他の全ての状態より優先度が高い
- **テスト方針** (TDD):
  - Red: 三方が塞がれている場合 → `U_TURN` に遷移
  - Red: U_TURN 中に前方が開けた → `FORWARD` に復帰
  - Red: 2方向のみ塞がれている → U_TURN にならず AVOID_LEFT/RIGHT になること
  - Green: 実装
- **完了条件**:
  - 全テスト Green
  - Gazebo 上で行き止まりに進入 → Uターンで脱出する動作を確認

### Step 7: テスト用ワールドの拡張

- **目的**: Phase 2 の全機能を検証できる障害物配置を追加する。
- **変更対象ファイル**:
  - `weeder_ws/src/grass_chopper/worlds/obstacles.world` (拡張)
- **実装内容**:
  - 既存ワールドに以下のシナリオを追加:
    1. **通路 (壁沿い走行テスト)**: 平行な壁 2 枚 (幅 2m の通路)
    2. **行き止まり (Uターンテスト)**: 通路の先端を壁で塞ぐ
    3. **T字路 (回避方向選択テスト)**: T 字の壁配置
    4. **連続障害物 (スラロームテスト)**: 3 つの円柱をジグザグ配置
  - 全てのモデルは `<static>true</static>` で物理演算負荷を軽減
- **完了条件**:
  - 各シナリオでロボットが適切な行動を取ること
  - デッドロック (無限ループ回避) が発生しないこと

### Step 8: 統合テストとパラメータチューニング

- **目的**: 全機能を統合した状態で、拡張ワールド上でのエンドツーエンド動作を検証する。
- **実装内容**:
  - 統合テストシナリオ:
    1. 正面の obstacle_1 に向かって前進 → 左右回避
    2. obstacle_wall に到達 → 壁沿い走行開始
    3. 通路に進入 → 壁沿い走行
    4. 行き止まりに到達 → Uターン
    5. 連続障害物のスラローム回避
  - パラメータチューニングの指針:
    - `safe_distance`: 0.4-0.6m
    - `wall_follow_kp`: 1.0-2.0
    - `wall_follow_kd`: 0.1-0.5
    - `forward_speed`: 0.1-0.3
- **完了条件**:
  - 60 秒間の連続走行で衝突なし
  - 全シナリオを通過できること

## テスト戦略

### ユニットテスト (最優先)

| テスト対象 | テスト手法 |
|---|---|
| `get_zone_stats` | 各種 LaserScan データを入力し最小/平均距離を検証 |
| `_deg_to_index` | 角度とインデックスの対応を検証 |
| `_update_state` | ゾーン情報を入力し、遷移先状態を assert |
| `_compute_twist` | 各状態に対して正しい Twist 値を assert |
| `_compute_wall_follow_twist` | PD 制御出力を数値検証 |
| パラメータ動的変更 | `set_parameters` 後に値が反映されることを検証 |

### テスト手法

- **ノードの生成**: `pytest` fixture で `rclpy.init()` / `rclpy.shutdown()` をモジュールスコープで管理
- **パブリッシャーのモック化**: `node.cmd_pub.publish = MagicMock()` で出力を検証
- **LaserScan の生成**: ヘルパー関数でゾーンごとに異なる距離を設定可能にする
  ```python
  def create_laser_scan(front=2.0, left=2.0, right=2.0, back=2.0) -> LaserScan:
      """ゾーンごとに距離を設定した LaserScan を生成"""
  ```
- **実行コマンド**: `colcon test --packages-select grass_chopper --pytest-args -s -v`

## Gazebo テスト環境

### 既存ワールドの障害物配置

| モデル名 | 位置 (X, Y) | サイズ | 検証対象 |
|---|---|---|---|
| obstacle_1 | (2.0, 0.0) | 0.5m 箱 | 正面回避 |
| obstacle_2 | (3.0, 1.5) | 0.6m 箱 | 左前方回避 |
| obstacle_3 | (4.0, -1.0) | 0.8m 箱 | 右前方回避、連続回避 |
| obstacle_wall | (6.0, 0.0) | 0.3x4.0m 壁 | 壁沿い走行 |

### Step 7 で追加するモデル

| モデル名 | 位置 (X, Y) | サイズ | 検証対象 |
|---|---|---|---|
| corridor_wall_left | (10, 1.0) | 0.2x6.0m | 通路左壁 |
| corridor_wall_right | (10, -1.0) | 0.2x6.0m | 通路右壁 |
| dead_end_wall | (13, 0.0) | 0.2x2.2m | 行き止まり壁 |
| t_junction_top | (3, 5.0) | 4.0x0.2m | T字路上壁 |
| t_junction_left | (1, 4.0) | 0.2x2.0m | T字路左壁 |
| t_junction_right | (5, 4.0) | 0.2x2.0m | T字路右壁 |
| slalom_1 | (1, -4.0) | 半径 0.3m 円柱 | スラローム 1 |
| slalom_2 | (3, -3.5) | 半径 0.3m 円柱 | スラローム 2 |
| slalom_3 | (5, -4.5) | 半径 0.3m 円柱 | スラローム 3 |

## ディレクトリ構造 (Phase 2 完了後)

```
weeder_ws/src/grass_chopper/
├── package.xml
├── setup.py                                    # config/ を data_files に追加
├── setup.cfg
├── resource/grass_chopper
├── config/
│   └── weeder_params.yaml                      # [NEW] ROS 2 パラメータ設定
├── grass_chopper/
│   ├── __init__.py
│   └── weeder_node.py                          # [MODIFIED] 状態遷移マシン + PD制御
├── launch/
│   └── sim_launch.py                           # [MODIFIED] パラメータファイル読み込み
├── test/                                        # [NEW] テストディレクトリ
│   ├── __init__.py
│   └── test_weeder_node.py
├── urdf/
│   └── robot_description.urdf.xacro
└── worlds/
    └── obstacles.world                          # [MODIFIED] テストシナリオ追加
```

## リスクと未解決事項

### 1. LiDAR のインデックスと角度の対応 (リスク: 中)

Gemini CLI の調査では `angle_min=-pi` (index 0 = -180度) を前提としたが、実際の Gazebo gpu_lidar が発行する `LaserScan` メッセージの `angle_min` / `angle_max` が URDF の設定通りであるか、実機確認が必要。

**対策**: Step 3 の実装前に、`ros2 topic echo /scan --once` で実際の `angle_min`, `angle_max`, `angle_increment` を確認し、ハードコードを避ける (メッセージのフィールドから動的に計算する設計を採用)

### 2. PD 制御ゲインの初期値 (リスク: 低)

`Kp=1.5`, `Kd=0.3` は推奨値だが、ロボットの寸法 (0.3m x 0.2m)、速度 (0.2m/s)、LiDAR レート (10Hz) の組み合わせで最適値は異なる可能性がある。

**対策**: パラメータ化済みなので、Gazebo 上で `ros2 param set` でリアルタイム調整可能。Step 8 で体系的にチューニングする

### 3. 状態遷移のチャタリング (リスク: 中)

ゾーン境界付近の距離変動により、状態が高速に切り替わる可能性がある。

**対策**: 遷移条件にヒステリシスを設ける。例: FORWARD → WALL_FOLLOW は `right_min < 0.75m` だが、WALL_FOLLOW → FORWARD は `right_min > 1.0m`。初期実装では簡易な閾値で進め、チャタリングが発生したら Step 8 で対応する

### 4. 壁沿い走行の方向 (左壁 vs 右壁) (リスク: 低)

現在の設計は「右壁沿い」のみ。Phase 4 (nav2) では経路計画が回避方向を決定するため、壁沿い走行は Phase 4 で不要になる可能性が高い。

### 5. 10Hz の制御周期の十分性 (リスク: 低)

LiDAR が 10Hz のため、0.2m/s で進む場合、1 サイクルで 2cm 進む。安全距離 0.5m があれば 25 サイクル (2.5 秒) のマージンがあり十分。

### 6. テスト時の rclpy 初期化 (リスク: 低)

`pytest` で `rclpy.init()` を `scope="module"` で管理するが、テストの並列実行時にコンフリクトする可能性がある。`colcon test` はデフォルトでパッケージ単位の逐次実行なので問題ない。

## 参考情報

### ROS 2 パラメータ化
- `declare_parameter` で型 (`ParameterType.PARAMETER_DOUBLE`) と説明 (`ParameterDescriptor`) を明示する
- `add_on_set_parameters_callback` で `ros2 param set` による動的変更に対応
- YAML ファイルは `ノード名: ros__parameters:` の階層構造

### 壁沿い走行アルゴリズム
- PD 制御 (比例-微分) が基本。I (積分) 項は壁沿いでは不要な蓄積を起こすため省略推奨
- `error = target_distance - actual_distance`
- `angular.z = Kp * error + Kd * d(error)/dt`
- 角速度のリミッタは必須

### 状態遷移マシン設計
- Python の `enum.Enum` + `auto()` で状態を定義
- 遷移ロジック (`_update_state`) と行動生成 (`_compute_twist`) を分離して、テスタビリティを確保
- 優先度: `U_TURN > AVOID_LEFT/RIGHT > WALL_FOLLOW > FORWARD`
