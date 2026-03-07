"""
============================================================================
ミッション管理 ROS 2 ノード (mission_tree_node)
============================================================================
ステートマシン方式でミッション全体を管理する。

状態遷移:
  IDLE → COVERAGE → RETURNING → DOCKING → CHARGING → IDLE
  - RETURNING: NavigateToPose で接近ポーズへ帰還
  - DOCKING: DockRobot で精密ドッキング (opennav_docking)
  - enable_docking=false の場合: RETURNING → CHARGING (MVP 動作)

購読:
  - /battery_state (sensor_msgs/BatteryState)
  - /coverage_ratio (std_msgs/Float32)
  - /coverage_progress (std_msgs/Int32MultiArray): カバレッジ走行進捗 [completed_index, total_waypoints]

配信:
  - /is_charging (std_msgs/Bool): 充電状態 → battery_sim_node

使い方:
  ros2 run grass_chopper mission_tree_node \
    --ros-args -p use_sim_time:=true \
    --params-file mission_params.yaml
============================================================================
"""

import math
import os
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, Int32MultiArray
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from grass_chopper.docking_behavior import (
    DockingAction,
    DockingParams,
    compute_approach_pose,
    evaluate_docking_result,
)
from grass_chopper.mission_behaviors import (
    CoverageProgress,
    MissionState,
    compute_home_pose,
    get_resume_index,
    should_continue_charging,
    should_resume_coverage,
    should_return_home,
    should_start_coverage,
    update_progress,
)


class MissionTreeNode(Node):
    """ミッション管理ノード (シンプルなステートマシン方式)"""

    # ミッション状態
    STATE_IDLE = "idle"
    STATE_COVERAGE = "coverage"
    STATE_RETURNING = "returning"
    STATE_DOCKING = "docking"
    STATE_CHARGING = "charging"

    def __init__(self):
        super().__init__('mission_tree_node')

        # --- パラメータ ---
        self.declare_parameter('tick_frequency', 2.0)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)
        self.declare_parameter('low_threshold', 0.2)
        self.declare_parameter('target_coverage', 0.9)
        self.declare_parameter('coverage_commander_params_file', '')
        self.declare_parameter('enable_docking', False)
        self.declare_parameter('max_docking_retries', 3)
        self.declare_parameter('dock_id', 'charging_dock_1')

        tick_freq = self.get_parameter('tick_frequency').value
        self._home_x = self.get_parameter('home_x').value
        self._home_y = self.get_parameter('home_y').value
        self._home_yaw = self.get_parameter('home_yaw').value
        self._low_threshold = self.get_parameter('low_threshold').value
        self._target_coverage = self.get_parameter('target_coverage').value
        self._coverage_params_file = self.get_parameter(
            'coverage_commander_params_file').value
        self._enable_docking = self.get_parameter('enable_docking').value
        self._max_docking_retries = self.get_parameter(
            'max_docking_retries').value
        self._dock_id = self.get_parameter('dock_id').value

        # --- 内部状態 ---
        self._mission_state = self.STATE_IDLE
        self._battery_percentage = 1.0
        self._coverage_ratio = 0.0
        self._is_navigating = False
        self._home_reached = False
        self._coverage_process = None
        self._coverage_progress = CoverageProgress(
            total_waypoints=0, completed_index=-1)
        self._docking_attempts = 0
        self._is_docking = False
        self._docking_params = DockingParams(
            home_x=self._home_x,
            home_y=self._home_y,
            home_yaw=self._home_yaw,
        )

        # --- NavigateToPose アクションクライアント (帰還用) ---
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # --- DockRobot アクションクライアント (精密ドッキング用) ---
        self._dock_client = None
        if self._enable_docking:
            try:
                from nav2_msgs.action import DockRobot
                self._DockRobot = DockRobot
                self._dock_client = ActionClient(
                    self, DockRobot, 'dock_robot')
                self.get_logger().info('精密ドッキング有効 (opennav_docking)')
            except ImportError:
                self.get_logger().warn(
                    'nav2_msgs.action.DockRobot が見つかりません。'
                    'ドッキング機能を無効化します')
                self._enable_docking = False

        # --- /battery_state 購読 ---
        self._battery_sub = self.create_subscription(
            BatteryState, '/battery_state',
            self._battery_callback, 10)

        # --- /coverage_ratio 購読 ---
        self._ratio_sub = self.create_subscription(
            Float32, '/coverage_ratio',
            self._coverage_ratio_callback, 10)

        # --- /coverage_progress 購読 (coverage_commander_node から進捗通知) ---
        self._progress_sub = self.create_subscription(
            Int32MultiArray, '/coverage_progress',
            self._coverage_progress_callback, 10)

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

    def _coverage_progress_callback(self, msg: Int32MultiArray):
        """カバレッジ走行進捗の更新 (data = [completed_index, total_waypoints])"""
        if len(msg.data) >= 2:
            self._coverage_progress = update_progress(
                self._coverage_progress,
                completed_index=msg.data[0],
                total_waypoints=msg.data[1],
            )

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
        elif self._mission_state == self.STATE_DOCKING:
            self._handle_docking(state)
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
            # 中断再開判定: 残ウェイポイントがあれば中断地点から再開
            start_index = 0
            if should_resume_coverage(self._coverage_progress):
                start_index = get_resume_index(self._coverage_progress)
                self.get_logger().info(
                    f'カバレッジ再開: ウェイポイント {start_index} から')
            self._start_coverage_commander(start_index=start_index)

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
                if self._enable_docking:
                    # 接近ポーズ到着 → 精密ドッキング開始
                    self._transition_to(self.STATE_DOCKING)
                    self._docking_attempts = 0
                    self._start_docking()
                else:
                    # ドッキング無効 → そのまま充電開始 (MVP)
                    self._transition_to(self.STATE_CHARGING)
                    self.get_logger().info('ホーム到着。充電開始')
            else:
                # 帰還失敗 → IDLE に戻る
                self.get_logger().error('ホームへの帰還に失敗。IDLE に戻ります')
                self._transition_to(self.STATE_IDLE)

    def _handle_docking(self, state: MissionState):
        """DOCKING: DockRobot 完了待ち (非同期コールバックで処理)"""
        pass  # コールバックで状態遷移を行う

    def _handle_charging(self, state: MissionState):
        """CHARGING: 満充電判定"""
        if not should_continue_charging(state):
            self.get_logger().info('充電完了。IDLE に遷移')
            # 注: _coverage_progress はリセットしない
            # → IDLE 遷移後に should_resume_coverage で再開判定される
            self._transition_to(self.STATE_IDLE)

    def _transition_to(self, new_state: str):
        """状態遷移"""
        old_state = self._mission_state
        self._mission_state = new_state
        self.get_logger().info(f'状態遷移: {old_state} → {new_state}')

    def _navigate_to_home(self):
        """ホーム位置 (ドッキング有効時は接近ポーズ) へ NavigateToPose を送信"""
        self._is_navigating = True
        self._home_reached = False

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 サーバー未接続')
            self._is_navigating = False
            return

        if self._enable_docking:
            # ドッキング有効: 接近ポーズ (ドック手前) へナビゲーション
            target_x, target_y, yaw = compute_approach_pose(
                self._docking_params)
            self.get_logger().info(
                f'接近ポーズ ({target_x:.1f}, {target_y:.1f}) へ帰還中...')
        else:
            # ドッキング無効: ホーム直接
            target_x = self._home_x
            target_y = self._home_y
            _, _, yaw = compute_home_pose(
                0.0, 0.0, self._home_x, self._home_y)
            self.get_logger().info(
                f'ホーム ({target_x:.1f}, {target_y:.1f}) へ帰還中...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target_x
        goal_msg.pose.pose.position.y = target_y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)

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

    def _start_docking(self):
        """DockRobot アクションを送信"""
        if self._dock_client is None:
            self.get_logger().error('DockRobot クライアント未初期化')
            self._transition_to(self.STATE_CHARGING)
            return

        self._is_docking = True
        self._docking_attempts += 1

        if not self._dock_client.server_is_ready():
            self.get_logger().error('DockRobot サーバー未接続')
            self._is_docking = False
            self._transition_to(self.STATE_CHARGING)
            return

        goal_msg = self._DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.dock_id = self._dock_id
        goal_msg.navigate_to_staging_pose = False  # 既に接近ポーズにいる

        self.get_logger().info(
            f'精密ドッキング開始 (試行 {self._docking_attempts}/'
            f'{self._max_docking_retries}): dock_id={self._dock_id}')

        future = self._dock_client.send_goal_async(goal_msg)
        future.add_done_callback(self._dock_goal_response)

    def _dock_goal_response(self, future):
        """DockRobot ゴール応答"""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'DockRobot ゴール送信エラー: {e}')
            self._handle_dock_failure()
            return

        if not goal_handle.accepted:
            self.get_logger().warn('DockRobot ゴールが拒否されました')
            self._handle_dock_failure()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._dock_result)

    def _dock_result(self, future):
        """DockRobot 結果"""
        succeeded = False
        try:
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                succeeded = True
                self.get_logger().info('精密ドッキング成功!')
            else:
                self.get_logger().warn(
                    f'精密ドッキング失敗 (status: {result.status})')
        except Exception as e:
            self.get_logger().error(f'DockRobot 結果取得エラー: {e}')

        self._is_docking = False
        action = evaluate_docking_result(
            self._docking_attempts, self._max_docking_retries, succeeded)

        if action == DockingAction.COMPLETE:
            self._transition_to(self.STATE_CHARGING)
            self.get_logger().info('ドッキング完了。充電開始')
        elif action == DockingAction.RETRY:
            self.get_logger().info('ドッキングリトライ...')
            self._start_docking()
        else:  # FALLBACK
            self.get_logger().warn(
                'ドッキング最大リトライ到達。フォールバック: 充電開始')
            self._transition_to(self.STATE_CHARGING)

    def _handle_dock_failure(self):
        """ドッキング失敗時の共通処理"""
        self._is_docking = False
        action = evaluate_docking_result(
            self._docking_attempts, self._max_docking_retries, False)
        if action == DockingAction.RETRY:
            self.get_logger().info('ドッキングリトライ...')
            self._start_docking()
        else:
            self.get_logger().warn('フォールバック: 充電開始')
            self._transition_to(self.STATE_CHARGING)

    def _start_coverage_commander(self, start_index: int = 0):
        """カバレッジコマンダーをサブプロセスで起動"""
        cmd = [
            'ros2', 'run', 'grass_chopper', 'coverage_commander_node',
            '--ros-args', '-p', 'use_sim_time:=true',
            '-p', f'start_index:={start_index}',
        ]
        if self._coverage_params_file:
            expanded = os.path.expanduser(self._coverage_params_file)
            cmd.extend(['--params-file', expanded])

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
