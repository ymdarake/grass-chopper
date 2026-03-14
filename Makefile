.PHONY: check-syntax test-pure test-coverage vm-build vm-sim vm-sim-headless vm-sim-nav2 vm-sim-coverage vm-sim-obstacles vm-sim-mission vm-sim-mission-headless vm-nav2 vm-nav2-test vm-coverage vm-coverage-obstacles vm-mission vm-battery vm-docking vm-kill vm-stop vm-info vm-topics vm-forward

## 構文チェック: Python + YAML
check-syntax:
	python3 scripts/syntax_check.py

## 純粋ロジックテスト (Mac ホストで実行可能, rclpy 不要)
test-pure:
	cd weeder_ws/src/grass_chopper && python3 -m pytest test/test_obstacle_avoidance.py test/test_coverage_planner.py test/test_coverage_tracker.py test/test_map_region_detector.py test/test_battery_simulator.py test/test_mission_behaviors.py test/test_docking_behavior.py -v

## カバレッジプランナーテスト (Mac ホストで実行可能)
test-coverage:
	cd weeder_ws/src/grass_chopper && python3 -m pytest test/test_coverage_planner.py -v

## VM 内で ROS 2 ワークスペースをビルド
## ※ マウント先にはビルド成果物を置けないため ~/weeder_build に出力
vm-build:
	multipass exec ros2-vm -- bash -c 'mkdir -p ~/weeder_build && source /opt/ros/jazzy/setup.bash && cd ~/weeder_ws && COLCON_LOG_PATH=/tmp/colcon_log colcon build --build-base ~/weeder_build/build --install-base ~/weeder_build/install'

## VM 内でシミュレーション起動
vm-sim:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && export LIBGL_ALWAYS_SOFTWARE=1 && export DISPLAY=:0 && ros2 launch grass_chopper sim_launch.py'

## VM 内でシミュレーション起動 (ヘッドレス: GUI なし、センサーデータは取得可)
vm-sim-headless:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && export LIBGL_ALWAYS_SOFTWARE=1 && ros2 launch grass_chopper sim_launch.py headless:=true'

## Nav2 モードでシミュレーション起動 (weeder_node なし、slam_test.world)
vm-sim-nav2:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && export LIBGL_ALWAYS_SOFTWARE=1 && export DISPLAY=:0 && ros2 launch grass_chopper sim_launch.py world:=slam_test.world x:=0.0 y:=-4.0 nav2_mode:=true'

## Nav2 スタック起動 (vm-sim-nav2 が起動済みの状態で実行)
vm-nav2:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && ros2 launch grass_chopper nav2_launch.py'

## カバレッジテスト用シミュレーション起動 (coverage_test.world + Nav2 モード)
vm-sim-coverage:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && export LIBGL_ALWAYS_SOFTWARE=1 && export DISPLAY=:0 && ros2 launch grass_chopper sim_launch.py world:=coverage_test.world x:=-3.5 y:=-3.5 nav2_mode:=true'

## 障害物ありカバレッジテスト用シミュレーション起動 (coverage_obstacles.world + Nav2 モード)
vm-sim-obstacles:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && export LIBGL_ALWAYS_SOFTWARE=1 && export DISPLAY=:0 && ros2 launch grass_chopper sim_launch.py world:=coverage_obstacles.world x:=-3.5 y:=-3.5 nav2_mode:=true'

## カバレッジ走行実行 (vm-sim-coverage + vm-nav2 が起動済みの状態で実行)
vm-coverage:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && ros2 run grass_chopper coverage_commander_node --ros-args --params-file ~/weeder_build/install/grass_chopper/share/grass_chopper/config/coverage_params.yaml -p use_sim_time:=true'

## 障害物ありカバレッジ走行実行 (vm-sim-obstacles + vm-nav2 が起動済みの状態で実行)
vm-coverage-obstacles:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && ros2 run grass_chopper coverage_commander_node --ros-args --params-file ~/weeder_build/install/grass_chopper/share/grass_chopper/config/coverage_obstacles_params.yaml -p use_sim_time:=true'

## ミッション管理テスト用シミュレーション起動 (docking_test.world + Nav2 モード)
vm-sim-mission:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && export LIBGL_ALWAYS_SOFTWARE=1 && export DISPLAY=:0 && export GZ_SIM_RESOURCE_PATH=~/weeder_build/install/grass_chopper/share/grass_chopper/models:$${GZ_SIM_RESOURCE_PATH} && ros2 launch grass_chopper sim_launch.py world:=docking_test.world x:=-3.5 y:=-3.5 nav2_mode:=true'

## ミッション管理テスト用シミュレーション起動 (ヘッドレス: GUI なし)
vm-sim-mission-headless:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && export LIBGL_ALWAYS_SOFTWARE=1 && export GZ_SIM_RESOURCE_PATH=~/weeder_build/install/grass_chopper/share/grass_chopper/models:$${GZ_SIM_RESOURCE_PATH} && ros2 launch grass_chopper sim_launch.py world:=docking_test.world x:=-3.5 y:=-3.5 nav2_mode:=true headless:=true'

## バッテリーシミュレーションノード起動 (vm-sim-mission が起動済みの状態で実行)
vm-battery:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && ros2 run grass_chopper battery_sim_node --ros-args --params-file ~/weeder_build/install/grass_chopper/share/grass_chopper/config/battery_params.yaml -p use_sim_time:=true'

## ミッション管理ノード起動 (vm-sim-mission + vm-nav2 + vm-battery が起動済みの状態で実行)
vm-mission:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && ros2 run grass_chopper mission_tree_node --ros-args --params-file ~/weeder_build/install/grass_chopper/share/grass_chopper/config/mission_params.yaml -p use_sim_time:=true'

## ドッキングスタック起動 (vm-sim-mission + vm-nav2 が起動済みの状態で実行)
vm-docking:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && source ~/weeder_build/install/setup.bash && export GZ_SIM_RESOURCE_PATH=~/weeder_build/install/grass_chopper/share/grass_chopper/models:$${GZ_SIM_RESOURCE_PATH} && ros2 launch grass_chopper docking_launch.py'

## Nav2 NavigateToPose テスト (ゴール: (0.0, 4.0) へ移動)
vm-nav2-test:
	multipass exec ros2-vm -- bash -c 'source /opt/ros/jazzy/setup.bash && ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 4.0, z: 0.0}, orientation: {w: 1.0}}}}"'

## VM 内の ROS 2 / Gazebo 残留プロセスを一括停止
vm-kill:
	multipass exec ros2-vm -- bash -c 'killall -9 mission_tree_node battery_sim_node coverage_tracker_node coverage_commander_node docking_server apriltag_node parameter_bridge ruby gz sim ros2 robot_state_publisher slam_toolbox controller_server planner_server behavior_server bt_navigator waypoint_follower lifecycle_manager twist_mux weeder_node 2>/dev/null; sleep 1; echo "cleaned"'

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
