.PHONY: check-syntax test-pure

## 構文チェック: Python + YAML
check-syntax:
	python3 scripts/syntax_check.py

## 純粋ロジックテスト (Mac ホストで実行可能, rclpy 不要)
test-pure:
	cd weeder_ws/src/grass_chopper && python3 -m pytest test/test_obstacle_avoidance.py -v
