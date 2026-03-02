# Phase 5: 実機移行 — 実装計画

## 概要

Phase 1〜4 でシミュレーション上に構築した自律走行アルゴリズム（障害物回避、SLAM、nav2 ナビゲーション、カバレッジ走行）を実機ロボットに移行する。ROS 2 の標準インターフェース設計により、`weeder_node.py`、`slam_toolbox`、`nav2` などの制御ノードはコード変更なしで実機でも動作する。本フェーズでは、ハードウェアの選定・調達、`ros2_control` による駆動制御、実機センサードライバの導入、実機用 launch ファイルの作成、および屋外環境向け測位の検討を行う。

## 前提条件

Phase 1〜4 の完了により、以下が利用可能であること:

- **Phase 1**: LiDAR 障害物回避ノード (`weeder_node.py`) — `/scan`, `/cmd_vel` のみ依存
- **Phase 2**: 状態遷移マシンによる賢い回避行動 — パラメータ YAML で設定変更可能
- **Phase 3**: slam_toolbox による 2D 地図生成・保存 — `/scan`, `/odom` のみ依存
- **Phase 4**: nav2 による自律ナビゲーション — 標準インターフェースで設計済み

## ハードウェア選定

### コンピュータ

| 候補 | CPU | GPU | メモリ | 価格帯 | 評価 |
|---|---|---|---|---|---|
| **Raspberry Pi 5 (8GB)** | ARM Cortex-A76 x4 | なし | 8GB | ~¥15,000 | nav2 には非力だが Phase 2 までなら十分。低消費電力。コスパ最良 |
| **Jetson Orin Nano (8GB)** | ARM Cortex-A78AE x6 | Ampere 1024-core | 8GB | ~¥45,000 | GPU ありで SLAM/nav2 に余裕。将来のカメラ AI にも対応。推奨 |
| **Intel NUC (i5)** | x86 i5 | Intel UHD | 16GB | ~¥60,000 | x86 互換で開発しやすいが高消費電力。屋外向けには不向き |

**推奨**: Jetson Orin Nano (8GB) — nav2 + SLAM を余裕で動かせ、将来的にカメラベースの物体認識にも対応可能。

> **注意**: 価格は変動が大きいため、購入時に最新価格を確認すること。

### フレーム・駆動系

| コンポーネント | 候補 | 備考 |
|---|---|---|
| **フレーム** | アルミフレーム自作 or RC カーベース | 草刈り刃の搭載を考慮した強度が必要 |
| **モーター** | DC ギアードモーター (12V, 30RPM 程度) | トルク重視 (草地走行のため) |
| **モータードライバ** | BTS7960 (43A 対応) | 大電流対応、PWM 制御 |
| **マイコン** | Arduino Mega or ESP32 | ros2_control の hardware_interface とシリアル通信 |
| **駆動方式** | **2輪 + スキッド** (キャスター廃止) | 草地ではキャスターが引っかかるため変更が必要 |

> **重要**: シミュレーションの 2輪+キャスター方式は実機の草地ではキャスターが草に引っかかるため、**2輪+スキッドプレート** または **4WD** への変更が必要。これは URDF の変更を伴う。

### センサー

| センサー | 候補 | 屋外対応 | 価格帯 | 備考 |
|---|---|---|---|---|
| **LiDAR** | RPLiDAR A3 | 80,000lux 対応 | ~¥50,000 | 屋外必須。A1/A2 は三角測距方式で屋外不可 |
| **LiDAR (代替)** | LDROBOT LD19 | DToF方式 | ~¥8,000 | 低価格だが検出距離 12m と短い |
| **カメラ** | Raspberry Pi Camera Module 3 | - | ~¥5,000 | USB カメラでも可 (`usb_cam` or `v4l2_camera`) |
| **IMU** | BNO055 or MPU9250 | - | ~¥2,000 | オドメトリの補正に必要 |
| **GPS** | u-blox ZED-F9P (RTK) | 必須 | ~¥30,000 | 草原の測位に必要。RTK 基地局も必要 |

> **重要**: 三角測距方式の LiDAR (RPLiDAR A1/A2, YDLIDAR X4) は屋外で完全に使用不可。RPLiDAR A3 (80,000lux対応) または DToF 方式の LDROBOT LD19 の二択。

### 電源

| コンポーネント | 仕様 | 備考 |
|---|---|---|
| **バッテリー** | LiPo 4S (14.8V) 5000mAh | モーター駆動用 |
| **DC-DC コンバーター** | 14.8V → 5V (for RPi/Jetson) | BEC or 降圧モジュール |
| **バッテリーモニター** | 電圧計 + ROS トピック発行 | 残量低下時の自動帰還に使用 |

## BOM (概算)

> **注意**: 以下の価格は 2024-2025 年時点の概算であり、変動が大きい。購入前に最新価格を確認すること。

| カテゴリ | アイテム | 概算価格 |
|---|---|---|
| コンピュータ | Jetson Orin Nano 8GB | ¥45,000 |
| LiDAR | RPLiDAR A3 | ¥50,000 |
| モーター x2 | DC ギアードモーター | ¥6,000 |
| モータードライバ | BTS7960 x2 | ¥3,000 |
| マイコン | Arduino Mega | ¥3,000 |
| カメラ | RPi Camera Module 3 | ¥5,000 |
| IMU | BNO055 | ¥2,000 |
| GPS | u-blox ZED-F9P | ¥30,000 |
| バッテリー | LiPo 4S 5000mAh | ¥8,000 |
| 電源回路 | DC-DC + BEC | ¥3,000 |
| フレーム・車輪 | アルミフレーム + ホイール | ¥15,000 |
| 配線・コネクタ | 各種 | ¥5,000 |
| **合計** | | **約 ¥175,000** |

## 実装ステップ

### Step 1: ros2_control の導入

- **目的**: Gazebo ネイティブの `gz-sim-diff-drive-system` を `ros2_control` + `diff_drive_controller` に置き換え、実機のモーター制御を可能にする
- **作業量**: 大
- **変更対象**:
  - `weeder_ws/src/grass_chopper/urdf/robot_description.urdf.xacro` — `<ros2_control>` タグ追加
  - `weeder_ws/src/grass_chopper_hardware/` — 新規 C++ パッケージ (hardware_interface 実装)
  - `weeder_ws/src/grass_chopper/config/controllers.yaml` — コントローラー設定

#### hardware_interface の設計

```
草刈りロボット hardware_interface
├── GrassChopperHardware (SystemInterface)
│   ├── on_init()       → シリアルポート初期化
│   ├── read()          → エンコーダ値をマイコンから取得
│   ├── write()         → モーター速度指令をマイコンに送信
│   ├── on_activate()   → モーターイネーブル
│   └── on_deactivate() → モーター停止
│
├── シリアル通信プロトコル (Arduino ↔ Jetson)
│   ├── 送信: "V,left_vel,right_vel\n"     [rad/s]
│   └── 受信: "E,left_enc,right_enc\n"     [ticks]
│
└── Arduino 側ファームウェア
    ├── PID 速度制御 (エンコーダフィードバック)
    ├── PWM 出力 → BTS7960 モータードライバ
    └── E-Stop ハードウェア割り込み
```

#### URDF への ros2_control 記述追加

```xml
<!-- use_sim 引数で sim/real を切り替え -->
<xacro:arg name="use_sim" default="true"/>

<ros2_control name="GrassChopperSystem" type="system">
  <hardware>
    <xacro:if value="$(arg use_sim)">
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </xacro:if>
    <xacro:unless value="$(arg use_sim)">
      <plugin>grass_chopper_hardware/GrassChopperHardware</plugin>
      <param name="serial_port">/dev/ttyACM0</param>
      <param name="baud_rate">115200</param>
    </xacro:unless>
  </hardware>
  <joint name="left_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

#### controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["left_wheel_joint"]
    right_wheel_names: ["right_wheel_joint"]
    wheel_separation: 0.24       # URDF と一致
    wheel_radius: 0.05           # URDF と一致
    publish_rate: 50.0
    odom_frame_id: odom
    base_frame_id: base_link
    pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
```

### Step 2: LiDAR ドライバの導入

- **目的**: Gazebo の `gpu_lidar` センサーを実機 LiDAR ドライバに置き換える
- **作業量**: 中
- **パッケージ**: `rplidar_ros` (RPLiDAR A3 の場合) or `ldlidar_stl_ros2` (LDROBOT LD19)
- **対応**:
  1. LiDAR ドライバパッケージをインストール
  2. `robot_launch.py` で LiDAR ドライバノードを起動 (トピック名 `/scan` で統一)
  3. `ros_gz_bridge` の `/scan` ブリッジは不要になるため削除

```bash
# RPLiDAR A3 の場合
sudo apt install ros-jazzy-rplidar-ros

# launch での起動
rplidar_node = Node(
    package='rplidar_ros',
    executable='rplidar_node',
    parameters=[{
        'serial_port': '/dev/ttyUSB0',
        'serial_baudrate': 256000,
        'frame_id': 'lidar_link',
        'scan_mode': 'Sensitivity',
    }]
)
```

### Step 3: 実機用 launch ファイルの作成

- **目的**: `sim_launch.py` と対になる `robot_launch.py` を作成し、実機環境を一括起動する
- **作業量**: 中
- **ファイル**: `weeder_ws/src/grass_chopper/launch/robot_launch.py`

```
sim_launch.py (シミュレーション):          robot_launch.py (実機):
├── Gazebo + World                         ├── (不要)
├── robot_state_publisher                  ├── robot_state_publisher (use_sim=false)
├── spawn_entity                           ├── (不要)
├── ros_gz_bridge                          ├── (不要)
├── controller_manager (gz_ros2_control)   ├── controller_manager (実機 HW I/F)
├── diff_drive_controller                  ├── diff_drive_controller
├── joint_state_broadcaster                ├── joint_state_broadcaster
├── rplidar_node                           ├── rplidar_node (実機 LiDAR)
├── usb_cam_node                           ├── usb_cam_node (実機カメラ)
├── weeder_node                            ├── weeder_node (コード変更なし!)
├── slam_toolbox (Phase 3)                 ├── slam_toolbox (コード変更なし!)
└── nav2 (Phase 4)                         └── nav2 (コード変更なし!)
```

### Step 4: 屋外測位の検討

- **目的**: 草原のような特徴点が乏しい環境で安定した自己位置推定を実現する
- **課題**: LiDAR SLAM (slam_toolbox) は壁や建物がない草原では破綻する
- **解決策**: RTK-GPS + IMU + Wheel Odometry のセンサーフュージョン

#### robot_localization によるデュアル EKF 構成

```
GPS (u-blox ZED-F9P)
  ↓ /gps/fix (NavSatFix)
navsat_transform_node
  ↓ /odometry/gps (Odometry in map frame)
  ↓
ekf_global (robot_localization)  ←── map → odom TF を発行
  ├── /odometry/gps
  ├── /imu/data
  └── /odom (wheel odometry)

ekf_local (robot_localization)   ←── odom → base_link TF を発行
  ├── /imu/data
  └── /odom (wheel odometry)
```

**ROS 2 パッケージ**:
- `robot_localization`: EKF/UKF によるセンサーフュージョン
- `nmea_navsat_driver` or `ublox_gps`: GPS ドライバ
- `imu_filter_madgwick`: IMU データのフィルタリング

> **未解決**: RTK-GPS には基地局 (Base Station) が必要。固定基地局を設置するか、NTRIP (インターネット経由の補正データ) を利用するかは、運用環境に依存するため追加調査が必要。

### Step 5: 安全設計

草刈りロボットは回転する刃を搭載するため、安全設計は最重要事項。

#### ハードウェア E-Stop (最優先)

```
緊急停止ボタン (NC接点)
  ↓
リレー回路 → モータードライバ電源遮断
  ↓
Arduino 割り込み → ROS 2 に E-Stop 状態を通知
```

- **NC (Normally Closed) 接点**: ケーブル断線時も安全側に動作
- **ソフトウェアに依存しない**: ROS 2 がクラッシュしてもモーターは停止する

#### ソフトウェア E-Stop

- `nav2_collision_monitor` (Phase 4 で導入済み)
- `twist_mux` による優先度付き速度指令管理
- Watchdog タイマー: 通信途絶 0.5 秒で全停止 (マイコン側で実装)

#### 草刈り刃の安全制御

- 刃の回転は別系統のモーターで制御
- 移動速度が 0 の場合は刃を停止するインターロック
- 傾斜センサーで転倒検知時に即停止
- 刃の周囲にバンパーセンサー (接触検知)

## シミュレーション ↔ 実機の切り替え方法

### launch 引数による切り替え

```bash
# シミュレーション
ros2 launch grass_chopper sim_launch.py

# 実機
ros2 launch grass_chopper robot_launch.py
```

### URDF の切り替え

`robot_description.urdf.xacro` に `use_sim` 引数を追加し、Gazebo プラグインと実機 hardware_interface を条件分岐:

```xml
<xacro:arg name="use_sim" default="true"/>

<!-- シミュレーション時: Gazebo ネイティブプラグイン -->
<xacro:if value="$(arg use_sim)">
  <gazebo>
    <plugin filename="gz-sim-diff-drive-system" .../>
  </gazebo>
</xacro:if>

<!-- 実機時: ros2_control -->
<xacro:unless value="$(arg use_sim)">
  <ros2_control name="GrassChopperSystem" type="system">
    ...
  </ros2_control>
</xacro:unless>
```

### パラメータの切り替え

実機用の YAML パラメータファイルを別途作成:

```
config/
├── weeder_params.yaml              # シミュレーション用
├── weeder_params_real.yaml         # 実機用 (safe_distance, speed を調整)
├── nav2_params.yaml                # シミュレーション用
├── nav2_params_real.yaml           # 実機用 (controller_frequency 等を調整)
└── controllers.yaml                # ros2_control 設定
```

## テスト戦略

### 実機テストの段階

1. **ベンチテスト**: ロボットを台に載せて車輪を浮かせた状態で動作確認
   - モーターの正転/逆転
   - エンコーダの値読み取り
   - ros2_control の基本動作
2. **屋内テスト**: 平坦な屋内フロアでの走行テスト
   - Phase 1 の障害物回避
   - SLAM による室内地図生成
3. **屋外テスト (舗装路)**: 屋外の舗装された場所でのテスト
   - LiDAR の屋外性能確認
   - GPS の精度確認
4. **屋外テスト (草地)**: 実際の草地でのテスト
   - 走行性能 (スリップ、トルク)
   - カバレッジ走行の実用性

### 安全テスト

| テスト項目 | 合否基準 |
|---|---|
| ハードウェア E-Stop | ボタン押下から 0.1 秒以内にモーター停止 |
| ソフトウェア E-Stop | collision_monitor による停止が 0.5 秒以内 |
| Watchdog タイマー | 通信途絶 0.5 秒で全停止 |
| 傾斜検知 | 30 度以上の傾斜で刃が停止 |
| ケーブル断線 | NC 接点により安全側に動作 |

## リスクと未解決事項

### 高リスク

| リスク | 影響 | 対策案 |
|---|---|---|
| **ros2_control hardware_interface の C++ 実装** | 最も作業量が大きく、組込み知識が必要 | 既存のサンプルコード (ros2_control_demos) を参考にする。まずシリアル通信の単体テストから始める |
| **屋外 LiDAR の選定** | 三角測距方式は屋外で使用不可。DToF 方式への変更が必要 | RPLiDAR A3 または LDROBOT LD19 の二択。購入前に屋外テストの動画・レポートを確認する |
| **草原での LiDAR SLAM 破綻** | 特徴点がないため slam_toolbox が機能しない | RTK-GPS + IMU + Odom のセンサーフュージョンで代替。Phase 4 の nav2 も GPS ベースの位置推定に対応可能 |

### 中リスク

| リスク | 影響 | 対策案 |
|---|---|---|
| **モーターのトルク不足** | 草地の抵抗でロボットが動けない | トルクの高いギアードモーターを選定。ギア比を上げる |
| **バッテリー稼働時間** | 草刈り完了前にバッテリー切れ | バッテリーモニターで残量監視、低残量時に自動帰還 |
| **Wi-Fi 通信の不安定性** | 屋外での ROS 2 DDS 通信が途切れる | DDS の QoS を RELIABLE に設定。Watchdog で安全停止 |
| **防水・耐候性** | 雨天時の故障 | IPX4 相当の防水ケースを使用。雨天時は運用しない運用ルール |

### 未解決事項 (要追加調査)

| 項目 | 詳細 | 優先度 |
|---|---|---|
| **RTK-GPS 基地局の運用方法** | 固定基地局 vs NTRIP。コストと精度のトレードオフ | 高 |
| **草刈り刃の機構設計** | 回転刃 vs 往復刃。安全性と効率のトレードオフ | 中 |
| **法規制の確認** | 自律走行ロボットの屋外運用に関する法的制約 | 中 |
| **駆動方式の最終決定** | 2輪+スキッド vs 4WD。草地の走行テストで判断 | 高 |
| **Arduino ファームウェアの PID チューニング** | モーターと減速比に依存するため実機で調整 | 中 |
| **ハードウェア部品の最新価格・入手性** | 価格変動が大きいため購入時に再調査 | 低 |

## 参考情報

- [ros2_control Documentation](https://control.ros.org/jazzy/)
- [ros2_control_demos (GitHub)](https://github.com/ros-controls/ros2_control_demos)
- [diff_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html)
- [rplidar_ros (GitHub)](https://github.com/Slamtec/rplidar_ros)
- [robot_localization](https://docs.ros.org/en/jazzy/p/robot_localization/)
- [u-blox ZED-F9P](https://www.u-blox.com/en/product/zed-f9p-module)
- [Jetson Orin Nano](https://developer.nvidia.com/embedded/jetson-orin-nano)
