#!/bin/bash
# ============================================================================
# ROS 2 + Gazebo Harmonic シミュレーション環境 起動スクリプト
# ============================================================================
# このスクリプトは以下を自動で行います:
#   1. Multipass VMの作成と起動（cloud-init による自動設定）
#   2. cloud-init の完了待ち
#   3. ワークスペースのマウント
#   4. ROS 2パッケージのビルド
#   5. ブラウザアクセス用URLの表示
#
# 前提条件:
#   - Multipass がインストール済み (brew install multipass)
#   - このスクリプトと同じディレクトリに setup.yaml がある
#
# 使い方:
#   bash run_sim.sh
# ============================================================================

set -e  # エラーが発生したらスクリプトを停止

# --- 設定 ---
VM_NAME="ros2-vm"
CLOUD_INIT="setup.yaml"
# Gazebo + noVNC を快適に動かすための推奨リソース
CPUS=4
MEMORY="8G"
DISK="30G"

# スクリプトのディレクトリを取得（相対パスで実行しても動くように）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "============================================"
echo " ROS 2 Jazzy + Gazebo Harmonic 環境構築"
echo "============================================"

# ===================================================================
# 1. VMの存在チェック
# ===================================================================
# 既にVMが存在する場合は再作成せずスキップ
if multipass info "${VM_NAME}" &>/dev/null; then
    echo ""
    echo "[情報] VM '${VM_NAME}' は既に存在します。"
    echo "  削除して再作成する場合: multipass delete ${VM_NAME} && multipass purge"
    echo "  起動のみ行います..."
    multipass start "${VM_NAME}" 2>/dev/null || true
else
    # ===================================================================
    # 2. VMの作成と起動
    # ===================================================================
    echo ""
    echo "[1/5] VM を作成中... (数分かかります)"
    echo "  OS: Ubuntu 24.04 | CPU: ${CPUS} | メモリ: ${MEMORY} | ディスク: ${DISK}"
    multipass launch 24.04 \
        --name "${VM_NAME}" \
        --cloud-init "${SCRIPT_DIR}/${CLOUD_INIT}" \
        --cpus "${CPUS}" \
        --memory "${MEMORY}" \
        --disk "${DISK}"
    echo "  VM作成完了!"
fi

# ===================================================================
# 3. cloud-init の完了を待機
# ===================================================================
echo ""
echo "[2/5] cloud-init の完了を待機中... (ROS 2等のインストールに10-20分程度)"
echo "  進行状況の確認: multipass exec ${VM_NAME} -- tail -f /var/log/cloud-init-output.log"
multipass exec "${VM_NAME}" -- cloud-init status --wait
echo "  cloud-init 完了!"

# ===================================================================
# 4. ワークスペースのマウント
# ===================================================================
echo ""
echo "[3/5] ワークスペースをVMにマウント中..."
# ホストの weeder_ws をVMの ~/weeder_ws にマウント
# これにより、ホスト側でコードを編集するとVM側に即座に反映される
multipass mount "${SCRIPT_DIR}/weeder_ws" "${VM_NAME}":/home/ubuntu/weeder_ws
echo "  マウント完了! (ホスト側のコード変更がVM内に即時反映されます)"

# ===================================================================
# 5. ROS 2 ワークスペースのビルド
# ===================================================================
echo ""
echo "[4/5] ROS 2 ワークスペースをビルド中..."
multipass exec "${VM_NAME}" -- bash -c "\
    source /opt/ros/jazzy/setup.bash && \
    cd ~/weeder_ws && \
    colcon build --symlink-install"
echo "  ビルド完了!"

# ===================================================================
# 6. アクセス情報の表示
# ===================================================================
# VMのIPアドレスを取得
VM_IP=$(multipass info "${VM_NAME}" --format json | \
    python3 -c "import sys,json; ips=json.load(sys.stdin)['info']['${VM_NAME}']['ipv4']; print(ips[0] if ips else 'localhost')")

echo ""
echo "============================================"
echo " セットアップ完了!"
echo "============================================"
echo ""
echo " noVNC (ブラウザでGUIにアクセス):"
echo "   http://${VM_IP}:6080/vnc.html"
echo ""
echo " シミュレーションの起動方法:"
echo "   1. 上記URLをブラウザで開く"
echo "   2. LXDEデスクトップが表示されたらターミナルを開く"
echo "   3. 以下のコマンドを実行:"
echo ""
echo "     source ~/weeder_ws/install/setup.bash"
echo "     ros2 launch grass_chopper sim_launch.py"
echo ""
echo " VM操作コマンド:"
echo "   VMに入る:    multipass shell ${VM_NAME}"
echo "   VM停止:      multipass stop ${VM_NAME}"
echo "   VM起動:      multipass start ${VM_NAME}"
echo "   VM削除:      multipass delete ${VM_NAME} && multipass purge"
echo ""
echo " ホスト側のコード編集:"
echo "   ${SCRIPT_DIR}/weeder_ws/src/grass_chopper/ 以下のファイルを"
echo "   お好みのエディタで編集してください。変更はVM内に自動反映されます。"
echo "   編集後は再ビルドが必要です:"
echo "     multipass exec ${VM_NAME} -- bash -c 'source /opt/ros/jazzy/setup.bash && cd ~/weeder_ws && colcon build --symlink-install'"
echo ""
