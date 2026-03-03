# grass-chopper プロジェクト固有設定

## プロジェクト概要

ROS 2 Jazzy + Gazebo Harmonic による草刈りロボットシミュレーション。
Multipass VM + noVNC で Mac ブラウザだけで動作する。

## 技術スタック

- **OS (VM)**: Ubuntu 24.04 LTS
- **ROS 2**: Jazzy (Python, ament_python ビルド)
- **シミュレーター**: Gazebo Harmonic
- **GUI**: noVNC (Xvfb + LXDE + x11vnc + websockify)
- **VM管理**: Multipass + cloud-init

## ディレクトリ構造

```
grass-chopper/
├── setup.yaml                              # cloud-init (VM自動構築)
├── run_sim.sh                              # VM起動スクリプト
├── docs/
│   ├── architecture.md                     # アーキテクチャ概要
│   └── technology-decisions.md             # 技術選定理由
└── weeder_ws/src/grass_chopper/            # ROS 2パッケージ
    ├── package.xml
    ├── setup.py / setup.cfg
    ├── grass_chopper/
    │   ├── obstacle_avoidance.py           # 純粋計算ロジック (ROS 2 非依存)
    │   └── weeder_node.py                  # ROS 2 アダプター (薄いラッパー)
    ├── test/
    │   ├── test_obstacle_avoidance.py      # 純粋ロジックテスト (Mac で実行可)
    │   └── test_weeder_node.py             # VM 統合テスト (rclpy 必要)
    ├── launch/sim_launch.py                # 一括起動
    ├── urdf/robot_description.urdf.xacro   # ロボットモデル
    └── worlds/obstacles.world              # Gazebo World
```

## アーキテクチャ原則

### Gazebo Harmonic ネイティブプラグイン方式

- Classic Gazebo の `libgazebo_ros_*.so` プラグインは **使用禁止**
- Gazebo ネイティブプラグイン (`gz-sim-*-system`) + `ros_gz_bridge` で通信
- 詳細は `docs/architecture.md` を参照

### ブリッジのトピック方向

```
ROS → GZ (指令):  /cmd_vel
GZ → ROS (データ): /scan, /camera/image_raw, /clock, /odom, /joint_states
```

## コーディング規約

### Python (ROS 2 ノード)

- ROS 2 の標準的なノード構成に従う (`rclpy.node.Node` 継承)
- QoS は Gazebo センサーデータに合わせて `BEST_EFFORT` を使用
- コメントは日本語で、初心者が理解できる粒度で記述

### URDF/Xacro

- 全リンクに `<inertial>` (mass + inertia) と `<collision>` を必ず定義
- 寸法は Xacro property で一元管理
- Gazebo プラグインは `<gazebo>` タグ内にネイティブ形式で記述

### World ファイル (SDF)

- 必須 World プラグイン 4 種を必ず含める:
  - `gz-sim-physics-system`
  - `gz-sim-user-commands-system`
  - `gz-sim-scene-broadcaster-system`
  - `gz-sim-sensors-system` (render_engine: ogre2)

### cloud-init (setup.yaml)

- APT リポジトリの追加は `runcmd` で実行 (packages セクションでは不可)
- サービス起動は `runcmd` 末尾で `systemctl enable --now`

## VM 設定

- **リソース**: 4 CPU / 8GB RAM / 30GB Disk
- **noVNC**: ポート 6080、VNC パスワード `ubuntu`
- **レンダリング**: `LIBGL_ALWAYS_SOFTWARE=1` (ソフトウェアレンダリング)

## ビルドと実行

```bash
# VM 起動 (初回)
bash run_sim.sh

# VM 内でビルド
multipass exec ros2-vm -- bash -c \
  'source /opt/ros/jazzy/setup.bash && cd ~/weeder_ws && colcon build --symlink-install'

# VM 内でシミュレーション起動
source ~/weeder_ws/install/setup.bash
ros2 launch grass_chopper sim_launch.py
```

## テスト方針

### Humble Object パターン

- 回避アルゴリズムの純粋計算ロジックは `obstacle_avoidance.py` に集約 (ROS 2 非依存)
- `weeder_node.py` は ROS 2 接続のみ担当する薄いアダプター
- 新しいロジックは必ず `obstacle_avoidance.py` に追加すること

### テスト実行

```bash
# Mac ホストで純粋ロジックテスト (rclpy 不要, 41テスト)
make test-pure

# 構文チェック
make check-syntax
```

- `test_obstacle_avoidance.py`: Mac ホストで即時実行可能な純粋ロジックテスト
- `test_weeder_node.py`: VM 統合テスト (rclpy 必要、VM 環境構築後に確認)
- URDF / World / cloud-init は構文チェック + 実環境での動作確認
- launch ファイルは統合テスト (シミュレーション起動確認)
