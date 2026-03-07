# grass-chopper

ROS 2 + Gazebo で動く草刈りロボットのシミュレーション環境です。
Mac のブラウザだけで完結します（XQuartz 不要）。

**目標:** シミュレーション上で自律走行アルゴリズムを段階的に開発・検証し、
最終的に実機に搭載できるレベルまで育てること。
現在は Phase 4e（フルオートノマス・ミッション）まで実装済み。
詳細は [docs/roadmap.md](docs/roadmap.md) を参照。

**できること (現在):**
- LiDAR 障害物回避 + 壁沿い走行 (Phase 1-2)
- SLAM による 2D 地図自動生成 (Phase 3)
- Nav2 経路計画・自律走行 (Phase 4a)
- Boustrophedon カバレッジ走行 + 障害物対応 + 地図自動検出 (Phase 4b-c)
- バッテリー管理 + ミッション管理ステートマシン (Phase 4d)
- 充電後の中断地点からのカバレッジ自動再開 (Phase 4e)
- AprilTag + opennav_docking による精密ドッキング (Phase 4e)

## クイックスタート

### 1. Multipass をインストール

[Multipass](https://multipass.run/) は Ubuntu の軽量 VM を Mac 上で動かすツールです。

```bash
# Homebrew がまだなら: https://brew.sh/ を参照
brew install multipass
```

### 2. リポジトリを取得

```bash
git clone https://github.com/ymdarake/grass-chopper.git
cd grass-chopper
```

### 3. VM を作成してセットアップ

```bash
bash run_sim.sh
```

初回は ROS 2 や Gazebo のインストールで **15〜25分** かかります。
コーヒーでも飲んで待ちましょう。

進捗を別ターミナルで確認できます:
```bash
multipass exec ros2-vm -- tail -f /var/log/cloud-init-output.log
```

### 4. ブラウザで VM のデスクトップを開く

スクリプト完了時に表示される URL をブラウザで開きます。

```
http://<表示されたIP>:6080/vnc.html
```

パスワード入力欄が表示されるので **`ubuntu`** と入力して「Connect」をクリック。
Linux のデスクトップ画面が表示されれば成功です。

### 5. シミュレーションを起動

noVNC デスクトップ内で **デスクトップを右クリック → 「System Tools」→「LXTerminal」** でターミナルを開き、以下を実行:

```bash
source ~/weeder_ws/install/setup.bash
ros2 launch grass_chopper sim_launch.py
```

### 6. 動作確認

正常に起動すると:

- **Gazebo のウィンドウ** が開き、緑の地面と赤・青・黄のボックス、グレーの壁が見える
- **ロボット** (緑色の小さな箱型) が自動で前進を始める
- 赤いボックス (2m 先) に近づくと **左に回転して回避** する
- ターミナルに `障害物検知! 距離: 0.XX m → 回転回避中` のログが流れる

---

## ロボットの仕様

LiDAR で前方 ±30度 をスキャンし、障害物を回避しながら前進します。

| 状況 | 動作 |
|---|---|
| 前方 0.5m 以内に障害物なし | 直進 (0.2 m/s) |
| 前方 0.5m 以内に障害物あり | 停止して左回転 (0.5 rad/s) |

### センサー

| センサー | 仕様 |
|---|---|
| LiDAR (`gpu_lidar`) | 360度 / 360サンプル / 10Hz / 0.12〜10.0m |
| カメラ (`camera`) | 640x480 / RGB / 30fps |

### ワールド (障害物配置)

緑色の地面に4つの障害物を配置しています。

- 赤ボックス (正面 2m)
- 青ボックス (左前方 3m)
- 黄ボックス (右前方 4m)
- グレーの壁 (正面 6m)

---

## コードの編集と再ビルド

ホスト (Mac) 側でコードを編集すると、Multipass マウントで VM に自動同期されます。

```bash
# ホスト側: お好みのエディタで編集
code weeder_ws/src/grass_chopper/grass_chopper/weeder_node.py

# VM 内で再ビルド (Makefile 経由)
make vm-build

# シミュレーション起動
make vm-sim
```

> Makefile にビルド・起動コマンドが定義されています。`make vm-build` で VM 内のビルドを実行します。

---

## VM 管理

```bash
multipass shell ros2-vm      # VM にログイン
multipass stop ros2-vm       # VM 停止 (メモリ解放)
multipass start ros2-vm      # VM 再起動
multipass info ros2-vm       # VM 情報表示
multipass delete ros2-vm && multipass purge  # VM 削除
```

## ROS 2 コマンド例

VM 内のターミナルで実行:

```bash
ros2 topic list                    # トピック一覧
ros2 topic echo /scan              # LiDAR データ確認
ros2 topic echo /cmd_vel           # 速度指令確認
ros2 topic echo /camera/image_raw  # カメラ画像確認
ros2 node list                     # ノード一覧
ros2 node info /weeder_node        # ノード詳細
```

---

## カスタマイズ

### 回避パラメータ (`weeder_node.py`)

```python
self.safe_distance = 0.5   # 安全距離 [m] — 小さくすると接近してから回避
self.forward_speed = 0.2   # 前進速度 [m/s]
self.turn_speed = 0.5      # 回転速度 [rad/s]
```

### ロボット寸法 (`robot_description.urdf.xacro`)

```xml
<xacro:property name="chassis_length" value="0.3"/>    <!-- 本体の長さ [m] -->
<xacro:property name="chassis_width" value="0.2"/>     <!-- 本体の幅 [m] -->
<xacro:property name="wheel_radius" value="0.05"/>     <!-- 車輪の半径 [m] -->
<xacro:property name="wheel_separation" value="0.24"/> <!-- 車輪間の距離 [m] -->
```

### 障害物の追加 (`obstacles.world`)

`worlds/obstacles.world` に以下を追加:

```xml
<model name="my_obstacle">
  <pose>3.0 2.0 0.25 0 0 0</pose>  <!-- x y z roll pitch yaw -->
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
    </collision>
    <visual name="visual">
      <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
      <material>
        <ambient>0.8 0.4 0.0 1</ambient>
        <diffuse>0.8 0.4 0.0 1</diffuse>
      </material>
    </visual>
  </link>
</model>
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
# VNC サービスの状態を確認
multipass exec ros2-vm -- systemctl status vnc-desktop.service

# サービスを再起動
multipass exec ros2-vm -- sudo systemctl restart vnc-desktop.service

# ログを確認
multipass exec ros2-vm -- journalctl -u vnc-desktop.service -n 30
```

### Gazebo が黒い画面 / 起動しない

VM 内でソフトウェアレンダリングが有効か確認:

```bash
echo $LIBGL_ALWAYS_SOFTWARE
# → 1 が表示されること。されない場合:
export LIBGL_ALWAYS_SOFTWARE=1
```

### LiDAR データが来ない (距離が全て inf)

```bash
ros2 topic list | grep scan      # トピック存在確認
ros2 topic hz /scan              # データ流入確認
ros2 topic info /scan --verbose  # QoS 確認 (BEST_EFFORT)
```

### ホスト側の変更が反映されない

```bash
multipass info ros2-vm                                        # マウント状態確認
multipass umount ros2-vm:/home/ubuntu/weeder_ws               # アンマウント
multipass mount ./weeder_ws ros2-vm:/home/ubuntu/weeder_ws    # 再マウント
```

---

## ファイル構成

```
grass-chopper/
├── README.md
├── CLAUDE.md                               # プロジェクト固有設定
├── setup.yaml                              # cloud-init (VM自動構築)
├── run_sim.sh                              # VM起動スクリプト
├── docs/
│   ├── architecture.md                     # アーキテクチャ概要
│   └── technology-decisions.md             # 技術選定理由
└── weeder_ws/src/grass_chopper/
    ├── package.xml                         # ROS 2パッケージ定義
    ├── setup.py                            # ビルド設定
    ├── setup.cfg
    ├── resource/grass_chopper              # ament index マーカー
    ├── grass_chopper/
    │   ├── __init__.py
    │   └── weeder_node.py                  # 障害物回避ノード
    ├── launch/
    │   └── sim_launch.py                   # 一括起動ファイル
    ├── urdf/
    │   └── robot_description.urdf.xacro    # ロボットモデル
    └── worlds/
        └── obstacles.world                 # シミュレーション環境
```

## アーキテクチャ

```
 ホスト (Mac)
 ├── weeder_ws/src/grass_chopper/  ← お好みのエディタで編集
 │        │
 │        │  Multipass マウント (自動同期)
 │        ▼
 │   Multipass VM (Ubuntu 24.04)
 │   ├── ROS 2 Jazzy
 │   ├── Gazebo Harmonic ── ネイティブプラグイン ──┐
 │   │     (物理演算・センサー)                    │
 │   │                                             ▼
 │   ├── ros_gz_bridge ── トピック変換 ── ROS 2 ノード群
 │   │     /cmd_vel (ROS→GZ)                  weeder_node
 │   │     /scan    (GZ→ROS)                  robot_state_publisher
 │   │     /camera/image_raw (GZ→ROS)
 │   │     /clock   (GZ→ROS)
 │   │
 │   └── noVNC (Xvfb + LXDE + x11vnc + websockify)
 │              ↑ ポート 6080
 └── ブラウザ ──┘
      http://<VM_IP>:6080/vnc.html
```

詳細は以下のドキュメントを参照:

- [ロードマップ](docs/roadmap.md) — 開発フェーズ、シミュレーションで検証できること、実機移行の計画
- [アーキテクチャ概要](docs/architecture.md) — 全体構成、レイヤーの役割、データフロー、起動シーケンス
- [技術選定の理由と背景](docs/technology-decisions.md) — 各技術の選定理由、候補との比較、背景説明

## 技術スタック

| コンポーネント | バージョン/種類 |
|---|---|
| OS (VM) | Ubuntu 24.04 LTS |
| ROS 2 | Jazzy |
| シミュレーター | Gazebo Harmonic |
| 駆動制御 | gz-sim-diff-drive-system |
| トピック変換 | ros_gz_bridge |
| リモートGUI | noVNC + websockify |
| 言語 | Python 3 |

## ライセンス

Apache-2.0
