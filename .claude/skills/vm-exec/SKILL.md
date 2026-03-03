---
name: vm-exec
description: Multipass VM (ros2-vm) 内でコマンドを実行する。VM 内の ROS 2 ビルド、シミュレーション起動、トピック確認、ファイル操作等に使用する。「VMで実行」「ビルドして」「ros2 topic list」「colcon build」「VM内で確認」「シミュレーション起動」等で発動。
---

# VM コマンド実行

Multipass VM `ros2-vm` 内でコマンドを実行する。

## 実行パターン

### 単純コマンド

```bash
multipass exec ros2-vm -- <command>
```

### bash -c (パイプ・環境変数・複数コマンド)

```bash
multipass exec ros2-vm -- bash -c '<commands>'
```

### ROS 2 コマンド (source 必須)

```bash
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && <ros2-command>'
```

## よく使うコマンド

### ビルド

```bash
multipass exec ros2-vm -- bash -c 'mkdir -p ~/weeder_build && source /opt/ros/jazzy/setup.bash && cd ~/weeder_ws && COLCON_LOG_PATH=/tmp/colcon_log colcon build --build-base ~/weeder_build/build --install-base ~/weeder_build/install'
```

**注意**: マウント上では `--symlink-install` 禁止（シンボリックリンクが壊れる）。`COLCON_LOG_PATH` は VM ローカルに設定。

### 残留プロセスの一括停止 (シミュレーション再起動前に必須)

```bash
multipass exec ros2-vm -- bash -c 'killall -9 parameter_bridge ruby gz sim ros2 weeder_node robot_state_publisher 2>/dev/null; sleep 1; echo "cleaned"'
```

### DISPLAY 番号の検出 (VNC 再起動後に確認)

```bash
multipass exec ros2-vm -- bash -c 'ps aux | grep Xvfb | grep -v grep'
# 出力例: Xvfb :0 -screen 0 1280x800x24  → DISPLAY=:0 を使う
```

### シミュレーション起動 (バックグラウンド)

```bash
# 基本 (obstacles.world, スポーン (0,0))
multipass exec ros2-vm -- bash -c 'export DISPLAY=:0 && export LIBGL_ALWAYS_SOFTWARE=1 && source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && ros2 launch grass_chopper sim_launch.py'

# SLAM テスト (slam_test.world, 南側廊下にスポーン)
multipass exec ros2-vm -- bash -c 'export DISPLAY=:0 && export LIBGL_ALWAYS_SOFTWARE=1 && source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && ros2 launch grass_chopper sim_launch.py world:=slam_test.world x:=0.0 y:=-4.0'
```

Bash ツールの `run_in_background: true` で実行する。停止は `TaskStop` を使う。

**注意**: `DISPLAY` の値は事前に DISPLAY 番号の検出コマンドで確認すること。

### トピック一覧 / 監視

```bash
# 一覧
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic list'
# 1回取得
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic echo /scan --once'
```

### ノード一覧

```bash
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 node list'
```

### VM 状態確認

```bash
multipass info ros2-vm
```

### ファイル転送 (ホスト → VM)

```bash
multipass transfer <local-path> ros2-vm:<remote-path>
```

## 注意事項

- ROS 2 コマンドは必ず `source /opt/ros/jazzy/setup.bash` を先行
- 自作パッケージ使用時は `source ~/weeder_build/install/setup.bash` も追加
- VM 未起動なら `multipass start ros2-vm` で起動
