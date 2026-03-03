---
name: sim-screenshot
description: noVNC 経由で Gazebo シミュレーション画面のスクリーンショットを撮影する。Playwright MCP でブラウザ操作を行う。「シミュレーション確認」「画面を見せて」「スクリーンショット」「Gazebo の様子」「noVNC 確認」「画面見たい」等で発動。
---

# シミュレーション スクリーンショット

Playwright MCP で noVNC にアクセスし、Gazebo シミュレーション画面を撮影する。

VM IP は `multipass info ros2-vm` の IPv4 フィールドで確認する。

## 手順

### 1. noVNC に接続

```
mcp__playwright__browser_navigate
  url: "http://<VM_IP>:6080/vnc.html?autoconnect=true&password=ubuntu"
```

### 2. 描画完了を待機

```
mcp__playwright__browser_wait_for
  time: 3
```

### 3. スクリーンショット撮影

```
mcp__playwright__browser_take_screenshot
  type: "png"
```

ファイル名指定:

```
mcp__playwright__browser_take_screenshot
  type: "png"
  filename: "sim-screenshot.png"
```

### 4. ブラウザを閉じる (任意)

```
mcp__playwright__browser_close
```

## noVNC 画面操作

VNC 画面内の Gazebo GUI を操作する場合:

- `mcp__playwright__browser_click` — マウスクリック
- `mcp__playwright__browser_press_key` — キーボード入力
- `mcp__playwright__browser_snapshot` — アクセシビリティツリー取得 (VNC 内は限定的)

## トラブルシューティング

| 症状 | 対処 |
|------|------|
| 接続不可 | `multipass info ros2-vm` で VM 起動確認。`multipass exec ros2-vm -- systemctl status novnc` でサービス確認 |
| 画面が黒い | シミュレーション未起動。vm-exec で `ros2 launch grass_chopper sim_launch.py` を実行 |
| ブラウザ未インストール | `mcp__playwright__browser_install` を実行 |

## 接続情報

- URL: `http://<VM_IP>:6080/vnc.html?autoconnect=true&password=ubuntu`
- VNC パスワード: `ubuntu`
- 前提: Playwright MCP (`.mcp.json`) 設定済み、VM ポート 6080 フォワード済み
