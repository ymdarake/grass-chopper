# 技術選定の理由と背景

## プロジェクトの目的

「Mac ユーザーが、環境を汚さず、ブラウザだけで ROS 2 ロボットシミュレーションを始められる」
ことを目指しています。ROS 2 は Ubuntu 向けのため Mac では直接動かせませんが、
VM + noVNC の組み合わせにより、Mac のセットアップは Multipass のインストールだけで済みます。

---

## 仮想化: Multipass

### 選定理由

| 候補 | 採用 | 理由 |
|---|---|---|
| **Multipass** | **採用** | Canonical 公式。cloud-init 対応。Mac ネイティブ。軽量 |
| Docker | 不採用 | GPU パススルーが困難。Gazebo の GUI 表示に難あり |
| UTM / Parallels | 不採用 | cloud-init 非対応または有料。再現性が低い |
| WSL2 | 対象外 | Mac では使えない |

### 背景

Multipass は Ubuntu の開発元 Canonical が提供する軽量 VM マネージャーです。
`cloud-init` に対応しており、YAML ファイル 1 つで VM の初期構築を完全自動化できます。
これにより「setup.yaml をコード管理するだけで開発環境を再現できる」という
Infrastructure as Code のメリットが得られます。

---

## OS: Ubuntu 24.04 LTS (Noble)

### 選定理由

- ROS 2 Jazzy の **公式サポート対象**
- Gazebo Harmonic の **公式サポート対象**
- LTS (長期サポート) で安定性が高い
- Multipass のデフォルト OS

### 背景

ROS 2 はディストリビューションごとに対応する Ubuntu バージョンが決まっています。
Jazzy は Ubuntu 24.04 (Noble) がプライマリプラットフォームであり、
この組み合わせが最もテストされていて安定しています。

---

## ロボットフレームワーク: ROS 2 Jazzy

### 選定理由

| 候補 | 採用 | 理由 |
|---|---|---|
| **ROS 2 Jazzy** | **採用** | 2024年リリースの最新 LTS。Ubuntu 24.04 対応 |
| ROS 2 Humble | 不採用 | Ubuntu 22.04 向け。24.04 では非公式 |
| ROS 2 Rolling | 不採用 | 開発版。安定性に欠ける |
| ROS 1 | 不採用 | 2025年5月に EOL。新規採用は非推奨 |

### 背景

ROS 2 (Robot Operating System 2) はロボット開発のデファクトスタンダードです。
ノード間のパブリッシュ/サブスクライブ通信、センサーメッセージの標準化、
パッケージ管理、launch システムなど、ロボット開発に必要な基盤を提供します。

Jazzy は 2024年リリースの LTS (5年サポート) であり、
最新の Gazebo Harmonic とネイティブに統合されています。

---

## シミュレーター: Gazebo Harmonic

### 選定理由

| 候補 | 採用 | 理由 |
|---|---|---|
| **Gazebo Harmonic** | **採用** | ROS 2 Jazzy の推奨。最新のネイティブプラグインアーキテクチャ |
| Classic Gazebo (Gazebo 11) | 不採用 | 2025年1月に EOL。メンテナンス終了 |
| Webots | 不採用 | ROS 2 統合が弱い。シェアが小さい |
| Isaac Sim | 不採用 | NVIDIA GPU 必須。VM では動作困難 |

### 背景

Gazebo は ROS エコシステムで最も広く使われている物理シミュレーターです。
「Gazebo Harmonic」は旧名 Ignition Gazebo のリブランド版で、
以下の点で Classic Gazebo (Gazebo 11) から大きく進化しています:

- **プラグインアーキテクチャの刷新**: ECS (Entity Component System) ベース
- **ROS 非依存のプラグイン**: `gz-sim-*-system` は ROS なしでも動作
- **ros_gz_bridge による疎結合**: Gazebo と ROS 2 が独立して動作

### Classic Gazebo との違い（重要）

```
# Classic Gazebo (非推奨):
#   URDF内でROSプラグインを直接指定
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <ros><namespace>/robot</namespace></ros>
</plugin>

# Gazebo Harmonic (本プロジェクト):
#   GazeboネイティブプラグインでGZ内部トピックに出力し、
#   ros_gz_bridge でROS 2トピックに変換する2段階アーキテクチャ
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
  <topic>cmd_vel</topic>  <!-- Gazebo内部トピック -->
</plugin>
# + ros_gz_bridge で /cmd_vel (ROS) ↔ cmd_vel (GZ) を変換
```

この設計により、Gazebo 側のシミュレーションを ROS なしでテストでき、
デバッグが容易になります。

---

## GUI アクセス: noVNC 方式

### 選定理由

| 候補 | 採用 | 理由 |
|---|---|---|
| **noVNC** | **採用** | ブラウザだけで完結。追加ソフト不要 |
| XQuartz (X11 転送) | 不採用 | Mac へのインストールが必要。描画が遅い。設定が煩雑 |
| VNC Viewer | 不採用 | 別途クライアントアプリが必要 |
| RDP (xrdp) | 不採用 | Mac のリモートデスクトップ設定が必要 |

### 背景

Gazebo はGUI アプリケーションなので、VM 内の画面をホストに転送する手段が必要です。
noVNC は WebSocket 経由で VNC 接続をブラウザに中継するツールで、
**ホスト側は何もインストールする必要がありません**。

構成は 4 段階のパイプラインです:

```
Xvfb      仮想フレームバッファ。物理モニタなしで画面を描画
  ↓
LXDE      デスクトップ環境。ターミナルやファイラーを提供
  ↓
x11vnc    X11 の画面内容を VNC プロトコルで配信
  ↓
websockify  VNC (TCP) → WebSocket に変換。ブラウザが接続可能に
```

x11vnc は `localhost` のみでリッスンし、外部からは websockify 経由でのみ
アクセスできるようにすることで、セキュリティリスクを最小化しています。

---

## デスクトップ環境: LXDE

### 選定理由

| 候補 | 採用 | 理由 |
|---|---|---|
| **LXDE** | **採用** | 非常に軽量。VM の限られたリソースで安定動作 |
| GNOME | 不採用 | メモリ消費が大きい (1GB+)。VM では重い |
| KDE Plasma | 不採用 | GNOME 同様リソース消費が大きい |
| XFCE | 候補 | LXDE と同程度だが、インストールサイズがやや大きい |

### 背景

VM 上で Gazebo を動かしつつデスクトップも描画するため、
デスクトップ環境自体のリソース消費は最小限にする必要があります。
LXDE はメモリ消費 100〜200MB 程度で、Gazebo にリソースを回せます。

---

## レンダリング: ソフトウェアレンダリング

### 選定理由

Multipass VM にはホストの GPU がパススルーされないため、
`LIBGL_ALWAYS_SOFTWARE=1` を設定してCPU ベースのソフトウェアレンダリングを使用します。

### 背景

Gazebo Harmonic の Sensors System は `ogre2` レンダリングエンジンを使用します。
通常は GPU が必要ですが、`LIBGL_ALWAYS_SOFTWARE=1` により Mesa の
ソフトウェアラスタライザー (llvmpipe) にフォールバックされます。

パフォーマンスは GPU 比で大幅に劣りますが、LiDAR (gpu_lidar) や
カメラ (camera) のセンサーデータ生成には十分です。

---

## LiDAR センサータイプ: gpu_lidar

### 選定理由

| 候補 | 採用 | 理由 |
|---|---|---|
| **gpu_lidar** | **採用** | Gazebo Harmonic の標準。ogre2 経由でレンダリング |
| lidar (CPU版) | 不採用 | Harmonic での対応が限定的 |

### 背景

Gazebo Harmonic では `gpu_lidar` が標準の LiDAR センサータイプです。
名前に "gpu" とありますが、`LIBGL_ALWAYS_SOFTWARE=1` 環境では
ソフトウェアレンダリングにフォールバックして動作します。

---

## ROS-Gazebo 通信: ros_gz_bridge

### 選定理由

Gazebo Harmonic のアーキテクチャ上、必須のコンポーネントです。

### 背景

Classic Gazebo では `gazebo_ros_pkgs` を通じて Gazebo プラグインが
直接 ROS トピックを発行していました。
Gazebo Harmonic ではこの密結合が廃止され、代わりに:

1. Gazebo ネイティブプラグインが GZ 内部トピックにデータを発行
2. `ros_gz_bridge` が GZ トピック ↔ ROS 2 トピックを変換

この 2 段階アーキテクチャにより、Gazebo と ROS 2 が独立してテスト可能になり、
他のミドルウェアへの移植性も向上しています。

---

## 差動駆動制御: gz-sim-diff-drive-system

### 選定理由

| 候補 | 採用 | 理由 |
|---|---|---|
| **gz-sim-diff-drive-system** | **採用** | Gazebo ネイティブ。シンプルで十分 |
| gz_ros2_control + diff_drive_controller | 不採用 | 実機連携向け。初心者には複雑 |

### 背景

`ros2_control` は実機のハードウェアインターフェースを抽象化するフレームワークですが、
純粋なシミュレーション目的では不要な複雑さを持ち込みます。
Gazebo ネイティブの diff-drive プラグインなら、URDF に数行追加するだけで
差動駆動制御が動作します。

将来実機との連携が必要になった段階で `gz_ros2_control` に移行できます。

---

## QoS 設定: BEST_EFFORT

### 選定理由

Gazebo のセンサーデータは `BEST_EFFORT` で配信されるため、
サブスクライバー側も同じ QoS に合わせる必要があります。

### 背景

ROS 2 の QoS (Quality of Service) には主に 2 つのモードがあります:

- **RELIABLE**: メッセージの確実な到達を保証（再送あり）
- **BEST_EFFORT**: 最善努力で配信（パケットロスを許容）

センサーデータは高頻度 (10〜30Hz) で配信され、1 フレームのロスより
低レイテンシが重要なため、BEST_EFFORT が適切です。
サブスクライバーを RELIABLE にすると、BEST_EFFORT のパブリッシャーから
データを受信できない問題が発生します。

---

## 環境構築自動化: cloud-init

### 選定理由

| 候補 | 採用 | 理由 |
|---|---|---|
| **cloud-init** | **採用** | Multipass ネイティブ対応。YAML で宣言的に記述 |
| シェルスクリプト | 不採用 | 冪等性がない。エラーハンドリングが煩雑 |
| Ansible | 不採用 | 追加ツールのインストールが必要。過剰な複雑さ |

### 背景

cloud-init は Ubuntu の標準的な初期構築ツールで、Multipass が直接サポートしています。
`setup.yaml` に宣言的に記述するだけで、パッケージのインストール、ファイルの配置、
サービスの起動を自動化できます。
このファイルを Git で管理することで、開発環境の再現性が保証されます。

---

## VMリソース: 4CPU / 8GB RAM / 30GB Disk

### 選定理由

Gemini CLI との相談の結果、以下の根拠で決定しました:

- **CPU 4コア**: Gazebo の物理演算 + センサー処理がマルチスレッドで動作
- **メモリ 8GB**: ROS 2 Desktop + Gazebo + LXDE + noVNC の同時動作に必要
- **ディスク 30GB**: OS (3GB) + ROS 2 Desktop (5GB) + Gazebo (3GB) + キャッシュ

当初の要件 (2CPU/4GB/20GB) ではGazebo のレンダリングが不安定になるリスクがあったため、
余裕を持った設定を採用しています。
