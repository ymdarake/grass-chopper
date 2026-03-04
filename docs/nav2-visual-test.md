# Nav2 目視確認手順

Phase 4a の Nav2 統合が正しく動作するかを noVNC + Playwright で目視確認する手順。

## 前提条件

- VM (`ros2-vm`) が起動済み
- `make vm-build` でビルド済み
- ポートフォワードが有効 (`make vm-forward` を別ターミナルで実行)

## 確認手順

### 1. 残留プロセスの停止

```bash
multipass exec ros2-vm -- bash -c 'killall -9 parameter_bridge ruby gz sim ros2 robot_state_publisher slam_toolbox controller_server planner_server behavior_server bt_navigator waypoint_follower lifecycle_manager twist_mux 2>/dev/null; sleep 1; echo "cleaned"'
```

### 2. Nav2 モードでシミュレーション起動

```bash
make vm-sim-nav2
```

slam_test.world (ロの字型回廊) でロボットが南側廊下 (0, -4) にスポーンする。
weeder_node は起動しない (Nav2 が cmd_vel を制御)。

### 3. Nav2 スタック起動 (別ターミナル)

```bash
make vm-nav2
```

lifecycle_manager が全ノードを configure → activate する。
`[lifecycle_manager_navigation]: Managed nodes are active` が表示されれば成功。

### 4. noVNC で Gazebo 画面を確認

`/sim-screenshot` スキルで撮影するか、ブラウザで noVNC にアクセス:

```
http://localhost:6080/vnc.html?autoconnect=true&password=ubuntu
```

**確認ポイント:**
- Gazebo がロの字型回廊を表示している
- ロボットが南側廊下に配置されている

### 5. NavigateToPose ゴール送信

```bash
make vm-nav2-test
```

ゴール: (0.0, 4.0) = 北側廊下。ロボットが回廊を通って移動する。

**注意:** SLAM 開始直後は地図が小さいため、遠いゴールは `error_code: 204` (No valid path) で失敗する場合がある。その場合は近いゴールから試す:

```bash
# 近距離テスト (1m 前方)
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback'
```

### 6. 目視確認項目

| # | 確認項目 | 合格基準 |
|---|---------|---------|
| 1 | Gazebo 画面にロボットが表示される | ロの字回廊の南側にいる |
| 2 | ゴール送信後にロボットが動き始める | cmd_vel が発行される |
| 3 | 壁に衝突しない | コストマップの inflation が機能 |
| 4 | ゴールに到達する | `Goal finished with status: SUCCEEDED` |
| 5 | SLAM 地図が更新される | `/sim-screenshot` で回廊の形状が見える |

### 7. ROS 2 コマンドでの確認

```bash
# ノード一覧 (Nav2 ノード群が存在すること)
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 node list'

# トピック確認
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic list | grep -E "cmd_vel|plan|costmap"'

# TF チェーン確認
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 run tf2_tools view_frames'

# ライフサイクル状態 (全て active [3] であること)
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && for node in controller_server planner_server behavior_server bt_navigator waypoint_follower; do echo -n "$node: "; ros2 lifecycle get /$node; done'
```

## トラブルシューティング

### slam_toolbox が unconfigured のまま

launch プロセスの出力をパイプ (`| head` 等) で打ち切ると lifecycle 遷移が中断される。
手動で activate する:

```bash
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 lifecycle set /slam_toolbox configure && sleep 2 && ros2 lifecycle set /slam_toolbox activate'
```

### NavigateToPose が rejected

Nav2 ノードが inactive の可能性がある。ライフサイクル状態を確認し、手動 activate:

```bash
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 lifecycle set /planner_server activate && ros2 lifecycle set /bt_navigator activate && ros2 lifecycle set /behavior_server activate && ros2 lifecycle set /waypoint_follower activate'
```

### error_code: 204 (No valid path found)

- SLAM がまだ十分な地図を構築していない → 近いゴールから試す
- /map が発行されていない → `ros2 topic info /map -v` で publisher count を確認
