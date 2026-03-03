.PHONY: check-syntax test-pure vm-build vm-sim vm-info vm-topics

## 構文チェック: Python + YAML
check-syntax:
	python3 scripts/syntax_check.py

## 純粋ロジックテスト (Mac ホストで実行可能, rclpy 不要)
test-pure:
	cd weeder_ws/src/grass_chopper && python3 -m pytest test/test_obstacle_avoidance.py -v

## VM 内で ROS 2 ワークスペースをビルド
vm-build:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && cd ~/weeder_ws && colcon build --symlink-install'

## VM 内でシミュレーション起動
vm-sim:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_ws/install/setup.bash && ros2 launch grass_chopper sim_launch.py'

## VM 情報表示
vm-info:
	multipass info ros2-vm

## ROS 2 トピック一覧
vm-topics:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic list'
