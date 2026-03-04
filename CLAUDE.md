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
├── .mcp.json                               # MCP サーバー設定 (Playwright)
├── .claude/skills/                         # Claude Code スキル
│   ├── vm-exec/SKILL.md                    # VM コマンド実行スキル
│   └── sim-screenshot/SKILL.md             # シミュレーション撮影スキル
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
    ├── config/
    │   ├── weeder_params.yaml              # 障害物回避パラメータ
    │   ├── mapper_params_online_async.yaml # SLAM パラメータ (Phase 3)
    │   ├── nav2_params.yaml                # Nav2 パラメータ (Phase 4a)
    │   └── twist_mux_params.yaml           # twist_mux 優先度設定 (Phase 4a)
    ├── launch/sim_launch.py                # 一括起動 (Gazebo + SLAM + weeder_node)
    ├── launch/nav2_launch.py               # Nav2 スタック起動 (Phase 4a)
    ├── urdf/robot_description.urdf.xacro   # ロボットモデル
    └── worlds/
        ├── obstacles.world                 # 障害物回避テスト環境
        └── slam_test.world                 # SLAM ループクロージャ検証環境
```

## アーキテクチャ原則

### Gazebo Harmonic ネイティブプラグイン方式

- Classic Gazebo の `libgazebo_ros_*.so` プラグインは **使用禁止**
- Gazebo ネイティブプラグイン (`gz-sim-*-system`) + `ros_gz_bridge` で通信
- 詳細は `docs/architecture.md` を参照

### ブリッジのトピック方向

```
ROS → GZ (指令):  /cmd_vel
GZ → ROS (データ): /scan, /camera/image_raw, /clock, /odom, /joint_states, /tf
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

### Launch ファイル (Python)

- `OpaqueFunction` 内で `TimerAction` を使って LifecycleNode (slam_toolbox 等) を遅延起動すると、autostart の状態遷移 (configure → activate) がブロックされる。LifecycleNode は `OpaqueFunction` の return リストに直接含めること
- 閉じた環境 (slam_test.world 等) では、ロボットのスポーン位置 (`-x`, `-y`) を必ず指定する。デフォルト (0,0) だと壁の内側にスポーンする事故が起きる
- `LaunchConfiguration` の値を `os.path.join` 等の Python 文字列操作で使うには `OpaqueFunction` + `.perform(context)` パターンが必要

### cloud-init (setup.yaml)

- APT リポジトリの追加は `runcmd` で実行 (packages セクションでは不可)
- サービス起動は `runcmd` 末尾で `systemctl enable --now`

## VM 設定

- **リソース**: 4 CPU / 8GB RAM / 30GB Disk
- **noVNC**: ポート 6080、VNC パスワード `ubuntu`
- **レンダリング**: `LIBGL_ALWAYS_SOFTWARE=1` (ソフトウェアレンダリング)
- **マウント**: ホスト `weeder_ws/` → VM `~/weeder_ws/` (ソース同期)
- **ビルド成果物**: VM ローカルの `~/weeder_build/` に出力 (マウント先はパーミッション制約あり)

## VM トラブルシューティング

### DISPLAY 番号の不一致

VNC サービス再起動後、Xvfb の DISPLAY 番号が `:99` → `:0` に変わることがある。
Gazebo 起動時に `qt.qpa.xcb: could not connect to display` エラーが出たら確認:

```bash
# 現在の DISPLAY 番号を検出
multipass exec ros2-vm -- bash -c 'ps aux | grep Xvfb | grep -v grep'
# 例: Xvfb :0 -screen 0 1280x800x24  → DISPLAY=:0 を使う
```

### 残留プロセスの一括停止

シミュレーションを再起動する前に、前回の残留プロセスを必ず停止する。
`parameter_bridge` 等が残留すると、新しいシミュレーションが正常に起動しない:

```bash
multipass exec ros2-vm -- bash -c 'killall -9 parameter_bridge ruby gz sim ros2 weeder_node robot_state_publisher 2>/dev/null; sleep 1; echo "cleaned"'
```

### colcon ビルドのマウント制約

Multipass マウント上では以下の制約がある:

- **`--symlink-install` 禁止**: マウント上でシンボリックリンクが root 所有になり壊れる
- **`COLCON_LOG_PATH`**: ログディレクトリを VM ローカルに設定 (`/tmp/colcon_log`)
- **ビルド成果物**: `~/weeder_build/` (VM ローカル) に出力

### map_saver_cli の使い方

SLAM で生成した地図を保存する際の注意:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/map_name -t map --ros-args -p use_sim_time:=true
```

- `-t map` オプションが必要（省略すると保存されない）
- `use_sim_time:=true` を忘れるとタイムスタンプ不一致で失敗する

## ビルドと実行

```bash
# VM 起動 (初回)
bash run_sim.sh

# VM 内でビルド
make vm-build

# VM 内でシミュレーション起動 (バックグラウンド)
make vm-sim

# Nav2 モードでシミュレーション起動 (weeder_node なし)
make vm-sim-nav2

# Nav2 スタック起動 (vm-sim-nav2 が起動済みの状態で実行)
make vm-nav2

# NavigateToPose テスト
make vm-nav2-test
```

## VM + ブラウザ ワークフロー

### スキル

| スキル | 用途 |
|--------|------|
| `/vm-exec` | VM 内でコマンド実行 (ビルド, ROS 2 コマンド等) |
| `/sim-screenshot` | noVNC 経由で Gazebo 画面を撮影 |

### CLI 操作 (multipass)

```bash
# VM 内でコマンド実行
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && <command>'

# VM 状態確認
multipass info ros2-vm

# ファイル転送 (ホスト → VM)
multipass transfer <local-path> ros2-vm:<remote-path>
```

### GUI 操作 (Playwright MCP + noVNC)

Multipass VM の仮想ネットワーク (192.168.64.x) はブラウザから直接アクセスできない。
ポートフォワードで localhost 経由にする:

```bash
# ポートフォワード開始 (別ターミナル or バックグラウンド)
make vm-forward
```

noVNC URL: `http://localhost:6080/vnc.html?autoconnect=true&password=ubuntu`

Playwright MCP ツールで noVNC にアクセスし、Gazebo / RViz の画面を確認・操作する。
設定は `.mcp.json` に定義済み。

### 典型的な開発フロー

1. Mac ホストでコード編集 + `make test-pure` で純粋ロジックテスト
2. `make vm-build` で VM 内ビルド
3. `make vm-sim` でシミュレーション起動
4. `/sim-screenshot` で Gazebo 画面を確認
5. `/vm-exec` で ROS 2 トピック監視 (`ros2 topic echo /scan --once` 等)

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
