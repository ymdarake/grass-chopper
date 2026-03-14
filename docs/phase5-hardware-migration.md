# Phase 5: 実機移行 調査レポート

## 1. ハードウェア構成案

### 1.1 コンピュータ

| 候補 | 消費電力 | 特徴 | 価格帯 | 推奨度 |
|------|---------|------|--------|--------|
| **Raspberry Pi 5 (8GB)** | 2.4W(idle)〜10W(負荷時) | 汎用、ROS 2 Jazzy 対応、NVMe SSD 対応 | ¥12,000〜15,000 | **推奨** |
| Raspberry Pi 4 (4GB) | 1.0W(idle)〜6W(負荷時) | 安価だが性能不足の可能性 | ¥8,000〜10,000 | 次点 |
| Jetson Orin Nano | 10W(idle)〜25W(負荷時) | AI/カメラ処理に強い (1024 CUDA コア)、5〜20V 入力対応 | ¥30,000〜50,000 | カメラ重視なら |

**推奨: Raspberry Pi 5 (8GB)**
- ROS 2 Jazzy + Nav2 + slam_toolbox を問題なく実行可能
- 消費電力が低くバッテリー持続時間に有利
- NVMe SSD でセンサーデータの I/O ボトルネックを解消

> **注意**: Pi 5 は 5.1V/5A の電源が必要。LiPo バッテリーから給電する場合は **12V → 5V バックコンバーター** が必須。電圧降下で Pi がシャットダウンする事故が多いので、変換器の定格に余裕を持たせること。

### 1.2 LiDAR

| 候補 | 測定範囲 | サンプル数 | 周波数 | 価格帯 | 推奨度 |
|------|---------|-----------|--------|--------|--------|
| **RPLidar A1M8** | 0.15〜12m | 360° / 8000点 | 5.5Hz | ¥12,000〜15,000 | **コスパ最良** |
| RPLidar A2M12 | 0.15〜18m | 360° / 8000点 | 10Hz | ¥20,000〜25,000 | 屋外向け |
| RPLidar A3 | 0.15〜25m | 360° / 16000点 | 15Hz | ¥40,000〜50,000 | 高精度 |

**推奨: RPLidar A1M8 (まずはこれで検証)**
- ROS 2 ドライバ (`rplidar_ros`) がライフサイクルノード対応で安定
- シミュレーションの `/scan` トピックと完全互換 — **コード変更ゼロ**
- 屋外では太陽光の影響を受けるが、近距離 (〜5m) なら実用的

### 1.3 IMU

| 候補 | 軸数 | インターフェース | 価格帯 | 推奨度 |
|------|------|----------------|--------|--------|
| **BNO055** | 9軸 (加速度+ジャイロ+磁気) | I2C/UART | ¥3,000〜5,000 | **推奨** |
| MPU9250 | 9軸 | I2C/SPI | ¥1,500〜3,000 | 安価だがキャリブレーション手間大 |
| ICM-20948 | 9軸 | I2C/SPI | ¥2,000〜4,000 | MPU9250 後継 |

**推奨: BNO055**
- オンチップセンサーフュージョン (四元数出力) — `incline_monitor.py` の `quaternion_to_rpy()` とそのまま接続可能
- ROS 2 ドライバ (`bno055_driver`) あり
- 既に実装済みの `calibrate_imu_offset()` で取り付けオフセット補正可能

### 1.4 モーター + ドライバ

| 構成要素 | 候補 | 価格帯 | 備考 |
|---------|------|--------|------|
| **駆動モーター** | 12V DC ギアモーター (エンコーダ付き) × 2 | ¥3,000〜5,000/個 | エンコーダ必須 (オドメトリ精度) |
| **モータードライバ** | Cytron MDD10A / L298N | ¥2,000〜4,000 | PWM + DIR 制御 |
| **マイコン (リアルタイム)** | Raspberry Pi Pico / Arduino | ¥500〜1,500 | モーター制御 + エンコーダ読取 |
| **草刈り刃モーター** | BLDC ブラシレスモーター + ESC | ¥5,000〜10,000 | FOC ドライバ推奨 |

**アーキテクチャ**:
```
Pi 5 (ROS 2)  ←─ USB/UART ─→  Pico (リアルタイムモーター制御)
                                  ├── 左モーター + エンコーダ
                                  ├── 右モーター + エンコーダ
                                  └── 刃モーター (BLDC)
```

> **なぜマイコンが必要か**: Pi 5 は Linux (非リアルタイム OS) なので、モーター PWM やエンコーダの高速読取はジッターが発生する。Pico で 1kHz のリアルタイム制御ループを回し、Pi 5 とは UART/USB で速度指令・エンコーダ値をやり取りする。

### 1.5 屋外測位 (GNSS/RTK)

| 候補 | 精度 | 価格帯 | 備考 |
|------|------|--------|------|
| **ArduSimple simpleRTK2B** | 1〜2cm (RTK Fix) | ¥25,000〜35,000 | F9P チップ、ROS 2 対応 |
| u-blox ZED-F9P ボード | 1〜2cm (RTK Fix) | ¥20,000〜30,000 | 最も普及 |
| BU-353S4 (単体 GPS) | 2〜5m | ¥5,000 | 精度不足、テスト用 |

**推奨: ArduSimple simpleRTK2B**
- OpenMower プロジェクトで実績あり
- `robot_localization` パッケージの `navsat_transform_node` で ROS 2 座標系に変換
- **RTK 基地局が必要**: 自前で設置するか、NTRIP (インターネット経由) を利用

> **重要**: 草原のような開けた環境では **LiDAR SLAM が破綻する** (特徴点がない)。RTK-GPS は必須。

### 1.6 電源系

| 構成要素 | 推奨 | 備考 |
|---------|------|------|
| **メインバッテリー** | 3S LiPo 11.1V 5000mAh 以上 | モーター + Pi + センサー共用 |
| **Pi 5 用電源** | 12V → 5V/5A バックコンバーター | 別系統推奨 (モーターノイズ分離) |
| **BMS** | 3S バランスチャージャー | 過放電・過充電保護 |
| **電圧監視** | ADC (ADS1115) or INA219 | `battery_sim_node` の実機版に接続 |

> **ハマりどころ**: モーターの突入電流で電圧が瞬間的に降下し、Pi がリブートする事故が頻発。**Pi と モーターの電源を分離するか、大容量コンデンサ (1000μF) を追加**すること。

### 1.7 概算 BOM (最小構成)

| カテゴリ | 品目 | 概算価格 |
|---------|------|---------|
| コンピュータ | Raspberry Pi 5 (8GB) + NVMe SSD + ケース | ¥20,000 |
| LiDAR | RPLidar A1M8 | ¥13,000 |
| IMU | BNO055 | ¥4,000 |
| モーター | DC ギアモーター (エンコーダ付) × 2 | ¥8,000 |
| モータードライバ | Cytron MDD10A | ¥3,000 |
| マイコン | Raspberry Pi Pico | ¥700 |
| バッテリー | 3S LiPo 5000mAh | ¥5,000 |
| 電源変換 | 12V→5V バックコンバーター | ¥1,500 |
| フレーム | アルミフレーム or 3D プリント | ¥5,000〜10,000 |
| 刃モーター | BLDC + ESC | ¥8,000 |
| 配線・コネクタ類 | — | ¥3,000 |
| **合計 (RTK なし)** | | **¥71,200〜76,200** |
| (オプション) RTK GPS | ArduSimple simpleRTK2B | +¥30,000 |

---

## 2. ソフトウェア移行の具体手順

### 2.1 変更不要なコンポーネント (そのまま動く)

| コンポーネント | 理由 |
|-------------|------|
| 全純粋ロジックモジュール (8 個) | ROS 2 非依存 |
| coverage_commander_node | NavigateToPose 標準アクション |
| mission_tree_node | 標準アクション + トピック |
| incline_monitor_node | `/imu` トピックのみ依存 |
| Nav2 スタック全体 | 標準インターフェース |
| slam_toolbox | `/scan` + `/odom` のみ依存 |

### 2.2 新規実装が必要なコンポーネント

#### Step 1: ros2_control hardware_interface

```python
# Pico (リアルタイム側) と UART で通信する hardware_interface
class GrassChopperHardware(SystemInterface):
    def on_configure():  # UART ポートを開く
    def read():          # Pico からエンコーダ値を読取 → position/velocity に格納
    def write():         # 速度指令を Pico に送信
```

**URDF の変更**:
```xml
<!-- Gazebo プラグインを ros2_control に置き換え -->
<ros2_control name="grass_chopper_hw" type="system">
  <hardware>
    <plugin>grass_chopper_hardware/GrassChopperHardware</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baud_rate">115200</param>
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

#### Step 2: Pico ファームウェア

```
受信: 左右モーター速度指令 (rad/s)
送信: 左右エンコーダ位置・速度
制御: PID 速度制御 (1kHz ループ)
```

#### Step 3: 実機用 launch ファイル

```python
# robot_launch.py — sim_launch.py から Gazebo/Bridge を除去し、ドライバノードを追加
def generate_launch_description():
    return LaunchDescription([
        robot_state_publisher,      # URDF → TF (変更なし)
        ros2_control_node,          # hardware_interface + diff_drive_controller
        rplidar_node,               # RPLidar A1 ドライバ
        bno055_node,                # IMU ドライバ
        slam_toolbox,               # SLAM (変更なし)
        # (オプション) robot_localization + navsat_transform (RTK GPS)
    ])
```

#### Step 4: battery_sim_node → 実機バッテリー監視

```python
# ADC (ADS1115) or INA219 で実電圧・電流を読み取り
# /battery_state トピックは同じフォーマット → mission_tree_node は変更不要
```

---

## 3. ハマりどころ一覧

### 3.1 ros2_control 関連

| 問題 | 症状 | 対策 |
|------|------|------|
| **read()/write() 内でメモリ確保** | 制御ループがスパイク的に遅延 | `on_configure()` で事前確保、ループ内は固定バッファのみ |
| **URDF の `<ros2_control>` ブロック不一致** | 起動時にエラーだが原因がわかりにくい | joint 名、interface 名を URDF のリンク名と厳密に一致させる |
| **wheel_radius / wheel_separation の誤差** | ロボットが円弧を描いたり曲がったりする | 実測値を mm 単位で測定し YAML に設定。走行テストで微調整 |
| **use_sim_time の不統一** | TF がずれる、Nav2 がタイムアウト | 実機では**全ノードで `use_sim_time: false`** に統一 |
| **Stamped Twist (Jazzy)** | diff_drive_controller が Twist を受け付けない | Jazzy の diff_drive_controller は `TwistStamped` が必須。`twist_mux` の `use_stamped` パラメータも合わせる |

### 3.2 ハードウェア関連

| 問題 | 症状 | 対策 |
|------|------|------|
| **モーター突入電流による電圧降下** | Pi がリブートする | Pi とモーターの電源分離、大容量コンデンサ追加 |
| **エンコーダなしで diff_drive** | オドメトリが大幅にずれる | **エンコーダは必須**。オープンループでは SLAM も Nav2 も破綻 |
| **LiDAR の USB 帯域** | スキャンデータが欠落する | Pi 5 の USB 3.0 ポートを使用、他の USB デバイスと分離 |
| **IMU の磁気干渉** | ヨー角がずれる、地磁気センサーが狂う | モーターから最低 10cm 離す、金属部品の近くに置かない |
| **屋外の太陽光** | LiDAR の反射が不安定になる | RPLidar A2 以上を検討、または超音波センサーを併用 |
| **防水** | 雨で基板がショート | IP65 以上のケース、コネクタ部にシリコンシーラント |
| **振動** | IMU データにノイズ | ゴムマウントで振動絶縁、`LowPassFilter` (実装済み) で除去 |

### 3.3 ソフトウェア関連

| 問題 | 症状 | 対策 |
|------|------|------|
| **SLAM が破綻 (屋外)** | 地図がグチャグチャになる | 草原では RTK-GPS + `robot_localization` EKF に切り替え |
| **オドメトリの共分散チューニング** | Nav2 の経路追従が不安定 | `pose_covariance_diagonal` を実機のドリフト量に合わせて調整 |
| **リアルタイムカーネル未導入** | 制御ループにジッター | `PREEMPT_RT` パッチ適用 or Pico 側で制御ループを完結 |
| **シリアル通信の遅延** | モーター応答が遅い | ボーレート 115200 以上、パケットサイズ最小化、チェックサム付与 |
| **残留プロセス** | ノードが起動しない | `systemd` でサービス管理、起動前に `killall` |

### 3.4 Jazzy 固有の注意点

| 問題 | 詳細 |
|------|------|
| **Ubuntu 24.04 必須** | Pi 5 は 24.04 対応済み。Pi 4 は公式イメージを使用 |
| **diff_drive_controller の TwistStamped** | Humble からの移行で最も引っかかるポイント。`use_stamped: true` に統一 |
| **bt_navigator の plugin_lib_names** | ビルトイン BT プラグインは自動登録。明示すると重複エラー (シミュレーションと同じ) |
| **Nav2 パラメータ変更** | `error_code_names` 必須 (シミュレーションで対応済み) |

---

## 4. 推奨する実機移行ステップ

### Phase 5a: 最小動作確認 (2〜3 週間)

1. Pi 5 + Pico + DC モーター × 2 + エンコーダ でシャーシ組み立て
2. Pico ファームウェア実装 (PID 速度制御 + UART 通信)
3. `ros2_control` hardware_interface 実装
4. `diff_drive_controller` でテレオペ (手動操作) 確認
5. **ゴール: `ros2 topic pub /cmd_vel` でロボットが前進・旋回する**

### Phase 5b: センサー統合 (1〜2 週間)

1. RPLidar A1 接続 → `/scan` トピック確認
2. BNO055 接続 → `/imu` トピック確認
3. `slam_toolbox` で屋内地図生成テスト
4. `incline_monitor_node` の動作確認 (手で傾けてログ確認)
5. **ゴール: SLAM 地図が生成でき、`incline_monitor` が反応する**

### Phase 5c: 自律走行 (1〜2 週間)

1. Nav2 スタック起動 → NavigateToPose テスト
2. カバレッジ走行テスト (小さな領域: 2m × 2m)
3. Collision Monitor の実機テスト
4. **ゴール: 指定領域をジグザグ走行できる**

### Phase 5d: 屋外対応 (2〜4 週間)

1. RTK-GPS 導入 → `robot_localization` EKF 設定
2. 屋外 SLAM vs GPS の切り替え判断
3. 草刈り刃モーター統合
4. バッテリー監視 (ADC) → `battery_state` トピック
5. ミッション管理フルサイクルテスト
6. **ゴール: 屋外の芝生を自律で草刈りできる**

---

## 5. 参考プロジェクト

| プロジェクト | URL | 特徴 |
|-----------|-----|------|
| **OpenMower** | github.com/ClemensElflein/OpenMower | YardForce 改造、RTK-GPS、ROS、最も成熟 |
| **Ardumower** | ardumower.de | Arduino ベース、RTK 対応、コミュニティ大 |
| **ros-mobile-robots** | ros-mobile-robots.com | Pi + ROS 2 のチュートリアル充実 |
| **Articulated Robotics** | articulatedrobotics.xyz | ros2_control の実践チュートリアル |

---

## 6. まとめ

| 項目 | 推奨 |
|------|------|
| コンピュータ | Raspberry Pi 5 (8GB) + NVMe SSD |
| LiDAR | RPLidar A1M8 (まず検証、屋外では A2 以上) |
| IMU | BNO055 (オンチップフュージョン) |
| 駆動 | DC ギアモーター (エンコーダ付) × 2 + Pico (リアルタイム制御) |
| GPS | ArduSimple simpleRTK2B (屋外必須) |
| バッテリー | 3S LiPo 5000mAh + 分離電源 |
| 最大のハマりどころ | モーター電源ノイズによる Pi リブート、エンコーダなしの SLAM 破綻、屋外での LiDAR SLAM 失敗 |
| 概算費用 | ¥71,000〜76,000 (RTK なし)、¥101,000〜106,000 (RTK あり) |
