# Role
あなたはROS 2とGazeboの専門家です。Macユーザー向けに、Multipassとcloud-initを用いた「コード管理可能な開発環境」を構築してください。

# Task Requirements
以下のファイルを現在のディレクトリに生成してください。

1. `setup.yaml` (cloud-init設定ファイル):
   - Ubuntu 24.04 (Noble) 用のROS 2 JazzyとGazebo Harmonicのインストール。
   - ブラウザでGUIを見るための `Xvfb`, `LXDE`, `noVNC`, `websockify` のインストールと自動起動設定。
   - ワークスペース `~/weeder_ws/src` の作成と、初期ビルド設定。

2. ロボット開発パッケージ (`weeder_ws/src/weeder_bot`):
   - `urdf/robot_description.urdf.xacro`: 2輪差動駆動、LiDAR、カメラを搭載。
   - `weeder_bot/weeder_node.py`: LiDARで障害物を検知し、回避しながら前進するPythonノード。
   - `launch/sim_launch.py`: Gazebo起動、モデル配置、ブリッジ接続の一括起動。
   - `package.xml`, `setup.py`: ROS 2の標準的なビルド設定。

3. `run_sim.sh` (起動用シェルスクリプト):
   - `multipass launch --name ros2-vm --cloud-init setup.yaml --cpus 2 --memory 4G --disk 20G` の実行コマンド。
   - VM起動後にブラウザでアクセスすべきURL (http://localhost:8080/vnc.html 等) の表示。

# Constraints
- Macのブラウザで完結する「noVNC」方式を採用してください（XQuartz不要）。
- コードには初心者が理解できるよう、詳細な日本語コメントを付与してください。