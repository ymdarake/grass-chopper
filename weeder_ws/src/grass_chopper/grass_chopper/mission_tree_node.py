"""
============================================================================
ミッション管理 ROS 2 ノード (mission_tree_node)
============================================================================
py_trees_ros で BT を構築し、ミッション全体を管理する。

BT 構造:
  Root (Selector)
  ├── Seq[LowBattery]: IsBatteryLow → NavigateToHome → SetCharging
  ├── Seq[Charging]: IsCharging → CheckBatteryFull → SetNotCharging
  ├── Seq[Coverage]: IsBatteryOK + HasUncovered → RunCoverage
  └── Idle

購読:
  - /battery_state (sensor_msgs/BatteryState)
  - /coverage_ratio (std_msgs/Float32)

配信:
  - /is_charging (std_msgs/Bool): 充電状態 → battery_sim_node

使い方:
  ros2 run grass_chopper mission_tree_node \
    --ros-args -p use_sim_time:=true \
    --params-file mission_params.yaml
============================================================================
"""

import math
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from grass_chopper.mission_behaviors import (
    MissionState,
    compute_home_pose,
    should_continue_charging,
    should_return_home,
    should_start_coverage,
)


class MissionTreeNode(Node):
    """ミッション管理ノード (シンプルなステートマシン方式)"""

    # ミッション状態
    STATE_IDLE = "idle"
    STATE_COVERAGE = "coverage"
    STATE_RETURNING = "returning"
    STATE_CHARGING = "charging"

    def __init__(self):
        super().__init__('mission_tree_node')

        # --- パラメータ ---
        self.declare_parameter('tick_frequency', 2.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('low_threshold', 0.2)
        self.declare_parameter('target_coverage', 0.9)
        self.declare_parameter('coverage_commander_params_file', '')

        tick_freq = self.get_parameter('tick_frequency').value
        self._home_x = self.get_parameter('home_x').value
        self._home_y = self.get_parameter('home_y').value
        self._low_threshold = self.get_parameter('low_threshold').value
        self._target_coverage = self.get_parameter('target_coverage').value
        self._coverage_params_file = self.get_parameter(
            'coverage_commander_params_file').value

        # --- 内部状態 ---
        self._mission_state = self.STATE_IDLE
        self._battery_percentage = 1.0
        self._coverage_ratio = 0.0
        self._is_navigating = False
        self._home_reached = False
        self._coverage_process = None

        # --- NavigateToPose アクションクライアント (帰還用) ---
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # --- /battery_state 購読 ---
        self._battery_sub = self.create_subscription(
            BatteryState, '/battery_state',
            self._battery_callback, 10)

        # --- /coverage_ratio 購読 ---
        self._ratio_sub = self.create_subscription(
            Float32, '/coverage_ratio',
            self._coverage_ratio_callback, 10)

        # --- /is_charging 配信 (battery_sim_node に充電状態通知) ---
        self._charging_pub = self.create_publisher(Bool, '/is_charging', 10)

        # --- ティックタイマー ---
        self._timer = self.create_timer(1.0 / tick_freq, self._tick)

        self.get_logger().info(
            f'ミッション管理ノード起動: tick={tick_freq:.1f}Hz, '
            f'home=({self._home_x:.1f}, {self._home_y:.1f}), '
            f'low_threshold={self._low_threshold:.0%}, '
            f'target_coverage={self._target_coverage:.0%}')

    def _battery_callback(self, msg: BatteryState):
        """バッテリー状態更新"""
        self._battery_percentage = msg.percentage

    def _coverage_ratio_callback(self, msg: Float32):
        """カバレッジ率更新"""
        self._coverage_ratio = msg.data

    def _build_mission_state(self) -> MissionState:
        """現在の状態から MissionState を構築"""
        return MissionState(
            battery_percentage=self._battery_percentage,
            is_charging=(self._mission_state == self.STATE_CHARGING),
            coverage_ratio=self._coverage_ratio,
            target_coverage=self._target_coverage,
            is_coverage_running=(self._mission_state == self.STATE_COVERAGE),
        )

    def _tick(self):
        """メインティック: 状態に応じた行動を決定"""
        state = self._build_mission_state()

        # 充電状態を battery_sim_node に配信
        charging_msg = Bool()
        charging_msg.data = (self._mission_state == self.STATE_CHARGING)
        self._charging_pub.publish(charging_msg)

        if self._mission_state == self.STATE_IDLE:
            self._handle_idle(state)
        elif self._mission_state == self.STATE_COVERAGE:
            self._handle_coverage(state)
        elif self._mission_state == self.STATE_RETURNING:
            self._handle_returning(state)
        elif self._mission_state == self.STATE_CHARGING:
            self._handle_charging(state)

    def _handle_idle(self, state: MissionState):
        """IDLE: カバレッジ開始判定"""
        if should_return_home(state, self._low_threshold):
            self._transition_to(self.STATE_RETURNING)
            self._navigate_to_home()
            return

        if should_start_coverage(state):
            self._transition_to(self.STATE_COVERAGE)
            self._start_coverage_commander()

    def _handle_coverage(self, state: MissionState):
        """COVERAGE: 低バッテリー帰還判定"""
        if should_return_home(state, self._low_threshold):
            self._stop_coverage_commander()
            self._transition_to(self.STATE_RETURNING)
            self._navigate_to_home()
            return

        # カバレッジ走行プロセスの終了確認
        if self._coverage_process is not None:
            retcode = self._coverage_process.poll()
            if retcode is not None:
                self.get_logger().info(
                    f'カバレッジ走行プロセス終了 (code: {retcode})')
                self._coverage_process = None
                self._transition_to(self.STATE_IDLE)

    def _handle_returning(self, state: MissionState):
        """RETURNING: ナビゲーション完了待ち"""
        if not self._is_navigating:
            if self._home_reached:
                # 帰還成功 → 充電開始
                self._transition_to(self.STATE_CHARGING)
                self.get_logger().info('ホーム到着。充電開始')
            else:
                # 帰還失敗 → IDLE に戻る
                self.get_logger().error('ホームへの帰還に失敗。IDLE に戻ります')
                self._transition_to(self.STATE_IDLE)

    def _handle_charging(self, state: MissionState):
        """CHARGING: 満充電判定"""
        if not should_continue_charging(state):
            self.get_logger().info('充電完了。IDLE に遷移')
            self._transition_to(self.STATE_IDLE)

    def _transition_to(self, new_state: str):
        """状態遷移"""
        old_state = self._mission_state
        self._mission_state = new_state
        self.get_logger().info(f'状態遷移: {old_state} → {new_state}')

    def _navigate_to_home(self):
        """ホーム位置へ NavigateToPose を送信"""
        self._is_navigating = True
        self._home_reached = False

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 サーバー未接続')
            self._is_navigating = False
            return

        _, _, yaw = compute_home_pose(
            0.0, 0.0,  # 現在位置は TF から取るべきだが MVP ではダミー
            self._home_x, self._home_y)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self._home_x
        goal_msg.pose.pose.position.y = self._home_y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

        self.get_logger().info(
            f'ホーム ({self._home_x:.1f}, {self._home_y:.1f}) へ帰還中...')

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._home_goal_response)

    def _home_goal_response(self, future):
        """帰還ゴール応答"""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'帰還ゴール送信エラー: {e}')
            self._is_navigating = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn('帰還ゴールが拒否されました')
            self._is_navigating = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._home_result)

    def _home_result(self, future):
        """帰還結果"""
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('ホームに到着しました')
                self._home_reached = True
            else:
                self.get_logger().warn(f'帰還失敗 (status: {result.status})')
                self._home_reached = False
        except Exception as e:
            self.get_logger().error(f'帰還結果取得エラー: {e}')
            self._home_reached = False
        self._is_navigating = False

    def _start_coverage_commander(self):
        """カバレッジコマンダーをサブプロセスで起動"""
        cmd = [
            'ros2', 'run', 'grass_chopper', 'coverage_commander_node',
            '--ros-args', '-p', 'use_sim_time:=true',
        ]
        if self._coverage_params_file:
            cmd.extend(['--params-file', self._coverage_params_file])

        self.get_logger().info(f'カバレッジ走行を開始: {" ".join(cmd)}')
        try:
            self._coverage_process = subprocess.Popen(cmd)
        except Exception as e:
            self.get_logger().error(f'カバレッジコマンダー起動エラー: {e}')
            self._transition_to(self.STATE_IDLE)

    def _stop_coverage_commander(self):
        """カバレッジコマンダーを停止"""
        if self._coverage_process is not None:
            self.get_logger().info('カバレッジコマンダーを停止中...')
            self._coverage_process.terminate()
            try:
                self._coverage_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self._coverage_process.kill()
            self._coverage_process = None


def main(args=None):
    """ノードのエントリーポイント"""
    rclpy.init(args=args)
    node = MissionTreeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ミッション管理ノードを停止します')
    finally:
        node._stop_coverage_commander()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
