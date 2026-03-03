#!/usr/bin/env python3
"""
TCP ポートフォワーダー: localhost:6080 -> VM_IP:6080

Multipass VM の仮想ネットワーク (192.168.64.x) はブラウザから
直接アクセスできない場合がある。このスクリプトで localhost 経由に
中継することで、ブラウザから noVNC にアクセス可能にする。

使い方:
    python3 scripts/port_forward.py [VM_IP]

    VM_IP を省略した場合は multipass info から自動取得する。
"""
import json
import socket
import subprocess
import sys
import threading


LOCAL_PORT = 6080
REMOTE_PORT = 6080


def get_vm_ip():
    """multipass info から VM の IP アドレスを取得する。"""
    try:
        result = subprocess.run(
            ["multipass", "info", "ros2-vm", "--format", "json"],
            capture_output=True, text=True, check=True,
        )
        info = json.loads(result.stdout)
        ips = info["info"]["ros2-vm"]["ipv4"]
        return ips[0] if ips else None
    except Exception as e:
        print(f"VM IP の取得に失敗: {e}", file=sys.stderr)
        return None


def forward(src, dst):
    try:
        while True:
            data = src.recv(4096)
            if not data:
                break
            dst.sendall(data)
    except Exception:
        pass
    finally:
        src.close()
        dst.close()


def handle(client, remote_host):
    remote = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        remote.connect((remote_host, REMOTE_PORT))
    except Exception as e:
        print(f"VM ({remote_host}:{REMOTE_PORT}) に接続できません: {e}")
        client.close()
        return
    t1 = threading.Thread(target=forward, args=(client, remote), daemon=True)
    t2 = threading.Thread(target=forward, args=(remote, client), daemon=True)
    t1.start()
    t2.start()


def main():
    remote_host = sys.argv[1] if len(sys.argv) > 1 else get_vm_ip()
    if not remote_host:
        print("エラー: VM IP を取得できませんでした。引数で指定してください。", file=sys.stderr)
        sys.exit(1)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", LOCAL_PORT))
    server.listen(5)
    print(f"ポートフォワード開始: localhost:{LOCAL_PORT} -> {remote_host}:{REMOTE_PORT}")
    print(f"ブラウザでアクセス: http://localhost:{LOCAL_PORT}/vnc.html?autoconnect=true&password=ubuntu")
    print("停止: Ctrl+C")
    try:
        while True:
            client, _ = server.accept()
            threading.Thread(target=handle, args=(client, remote_host), daemon=True).start()
    except KeyboardInterrupt:
        print("\n停止しました。")
    finally:
        server.close()


if __name__ == "__main__":
    main()
