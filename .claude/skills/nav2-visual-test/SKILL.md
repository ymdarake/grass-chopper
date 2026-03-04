---
name: nav2-visual-test
description: Nav2 ナビゲーションの自動目視確認テスト。VM 上で Gazebo + SLAM + Nav2 を起動し、NavigateToPose でロボットを移動させ、Playwright MCP で noVNC スクリーンショットを撮影して結果を報告する。「Nav2 テスト」「ナビゲーションテスト」「Nav2 動作確認」「自律移動テスト」等で発動。
---

# Nav2 Visual Test

Nav2 統合の自動テスト。Makefile ターゲット + Playwright MCP で実行。

## 前提条件

- VM (`ros2-vm`) 起動済み、`make vm-build` 済み
- `make vm-forward` を別ターミナルで実行中

## ワークフロー

### Step 1: 残留プロセス停止 + シミュレーション起動

```bash
# 残留プロセス停止
make vm-kill

# Nav2 モードでシミュレーション起動 (run_in_background: true)
make vm-sim-nav2
```

起動待ち: 別の Bash で `sleep 30` を実行。

### Step 2: Nav2 スタック起動

```bash
# Nav2 起動 (run_in_background: true)
make vm-nav2
```

起動待ち: 別の Bash で `sleep 30` を実行。

### Step 3: ライフサイクル状態確認

```bash
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && for node in controller_server planner_server behavior_server bt_navigator waypoint_follower; do echo -n "$node: "; ros2 lifecycle get /$node; done'
```

全ノードが `active [3]` であること。inactive なら:

```bash
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 lifecycle set /slam_toolbox configure && sleep 2 && ros2 lifecycle set /slam_toolbox activate'
```

### Step 4: "Before" スクリーンショット

`/sim-screenshot` スキルと同じ手順で noVNC に接続し撮影。filename: `sim-nav2-before.png`

### Step 5: NavigateToPose ゴール送信

```bash
make vm-nav2-test
```

**成功基準**: `Goal finished with status: SUCCEEDED`

SLAM 直後は地図が小さく遠距離ゴールは `error_code: 204` で失敗する場合がある。その場合は近距離テスト:

```bash
multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -4.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback'
```

### Step 6: "After" スクリーンショット

移動完了後に撮影。filename: `sim-nav2-after.png`

### Step 7: 結果レポート

| # | 確認項目 | 結果 |
|---|---------|------|
| 1 | Nav2 ノード全て active | OK / NG |
| 2 | NavigateToPose 成功 | OK / NG |
| 3 | ロボットが移動した (Before/After) | OK / NG |
| 4 | 壁に衝突していない | OK / NG |

### Step 8: クリーンアップ (任意)

```bash
make vm-kill
```

## トラブルシューティング

- **error_code: 204**: SLAM 地図不十分。近いゴールから試す
- **NavigateToPose rejected**: ノード inactive。Step 3 の手動 activate 実行
- **slam_toolbox unconfigured**: launch 出力をパイプ (`| head`) で打ち切らない。手動 configure → activate
- **noVNC 接続失敗**: `make vm-forward` が実行中か確認
