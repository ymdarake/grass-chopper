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
multipass exec ros2-vm -- bash -c 'mkdir -p ~/weeder_build && source /opt/ros/jazzy/setup.bash && cd ~/weeder_ws && colcon --log-base ~/weeder_build/log build --symlink-install --build-base ~/weeder_build/build --install-base ~/weeder_build/install'
```

### シミュレーション起動 (バックグラウンド)

```bash
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && ros2 launch grass_chopper sim_launch.py'
```

Bash ツールの `run_in_background: true` で実行する。停止は `TaskStop` を使う。

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
