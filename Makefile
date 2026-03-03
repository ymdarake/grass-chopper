.PHONY: check-syntax test-pure vm-build vm-sim vm-stop vm-info vm-topics vm-forward

## 構文チェック: Python + YAML
check-syntax:
	python3 scripts/syntax_check.py

## 純粋ロジックテスト (Mac ホストで実行可能, rclpy 不要)
test-pure:
	cd weeder_ws/src/grass_chopper && python3 -m pytest test/test_obstacle_avoidance.py -v

## VM 内で ROS 2 ワークスペースをビルド
## ※ マウント先にはビルド成果物を置けないため ~/weeder_build に出力
vm-build:
	multipass exec ros2-vm -- bash -c 'mkdir -p ~/weeder_build && source /opt/ros/jazzy/setup.bash && cd ~/weeder_ws && COLCON_LOG_PATH=/tmp/colcon_log colcon build --build-base ~/weeder_build/build --install-base ~/weeder_build/install'

## VM 内でシミュレーション起動
vm-sim:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && export LIBGL_ALWAYS_SOFTWARE=1 && export DISPLAY=:0 && ros2 launch grass_chopper sim_launch.py'

## VM 停止
vm-stop:
	multipass stop ros2-vm

## VM 情報表示
vm-info:
	multipass info ros2-vm

## ROS 2 トピック一覧
vm-topics:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic list'

## noVNC ポートフォワード (localhost:6080 -> VM:6080)
## ブラウザから VM の仮想ネットワークに直接アクセスできない場合に使用
vm-forward:
	python3 scripts/port_forward.py
