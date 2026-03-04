"""
============================================================================
カバレッジ走行 コマンダーノード (coverage_commander_node)
============================================================================
NavigateToPose アクションを使ってウェイポイントを1つずつ順次送信し、
カバレッジ走行を実行する ROS 2 アダプターノード。

計算ロジックは coverage_planner モジュールに委譲し、
このノードは ROS 2 接続 (アクションクライアント/パラメータ) のみ担当する。

使い方:
  ros2 run grass_chopper coverage_commander_node \
    --ros-args -p use_sim_time:=true

パラメータ:
  - region (double[]): カバレッジ領域 [min_x, min_y, max_x, max_y]
  - swath_width (double): 作業幅 [m]
  - margin (double): 境界マージン [m]
  - direction (double): 走行方向 [rad]
============================================================================
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from shapely.geometry import Polygon

from grass_chopper.coverage_planner import (
    CoverageParams,
    generate_boustrophedon_waypoints,
    estimate_coverage_ratio,
)


class CoverageCommanderNode(Node):
    """カバレッジ走行を指示するノード (NavigateToPose 逐次方式)"""

    def __init__(self):
        super().__init__('coverage_commander_node')

        # --- ROS 2 パラメータ ---
        self.declare_parameter(
            'region', [-3.0, -3.0, 3.0, 3.0],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                description='カバレッジ領域 [min_x, min_y, max_x, max_y]'
            )
        )
        self.declare_parameter(
            'swath_width', 0.18,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='作業幅 [m]'
            )
        )
        self.declare_parameter(
            'margin', 0.3,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='境界マージン [m]'
            )
        )
        self.declare_parameter(
            'direction', 0.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='走行方向 [rad]'
            )
        )

        # --- NavigateToPose アクションクライアント ---
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # ウェイポイントキューと進捗管理
        self._waypoints = []
        self._current_index = 0
        self._total_waypoints = 0

        self.get_logger().info('カバレッジコマンダーノードを起動しました')

        # 起動後すぐにカバレッジ走行を開始
        self._timer = self.create_timer(1.0, self._start_coverage)

    def _start_coverage(self):
        """カバレッジ走行を開始する (タイマーコールバック、1回だけ実行)"""
        self._timer.cancel()

        # パラメータ取得
        region = self.get_parameter('region').value
        swath_width = self.get_parameter('swath_width').value
        margin = self.get_parameter('margin').value
        direction = self.get_parameter('direction').value

        if len(region) != 4:
            self.get_logger().error(f'region パラメータは [min_x, min_y, max_x, max_y] の4要素必要: {region}')
            return

        min_x, min_y, max_x, max_y = region

        # 矩形ポリゴン生成
        polygon = Polygon([
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
        ])

        params = CoverageParams(
            swath_width=swath_width,
            margin=margin,
            direction=direction,
        )

        # ウェイポイント生成
        self._waypoints = generate_boustrophedon_waypoints(polygon, params)
        if not self._waypoints:
            self.get_logger().error('ウェイポイントを生成できませんでした')
            return

        self._total_waypoints = len(self._waypoints)
        self._current_index = 0

        # カバレッジ率推定
        ratio = estimate_coverage_ratio(self._waypoints, polygon, swath_width)
        self.get_logger().info(
            f'カバレッジ走行計画: {self._total_waypoints} ウェイポイント, '
            f'推定カバレッジ率: {ratio:.1%}, '
            f'領域: [{min_x}, {min_y}] ~ [{max_x}, {max_y}]'
        )

        # アクションサーバー接続待ち
        self.get_logger().info('Nav2 アクションサーバーの接続を待機中...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 アクションサーバーに接続できませんでした')
            return

        # 最初のウェイポイントへナビゲーション開始
        self._navigate_to_next()

    def _navigate_to_next(self):
        """次のウェイポイントへ NavigateToPose を送信"""
        if self._current_index >= self._total_waypoints:
            self.get_logger().info('カバレッジ走行完了!')
            return

        wp = self._waypoints[self._current_index]
        remaining = self._total_waypoints - self._current_index

        self.get_logger().info(
            f'ウェイポイント {self._current_index + 1}/{self._total_waypoints} '
            f'({wp.x:.2f}, {wp.y:.2f}) へ移動中... (残り: {remaining})'
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp.x
        goal_msg.pose.pose.position.y = wp.y
        goal_msg.pose.pose.position.z = 0.0
        # yaw → quaternion (z 軸回転のみ)
        goal_msg.pose.pose.orientation.z = math.sin(wp.yaw / 2)
        goal_msg.pose.pose.orientation.w = math.cos(wp.yaw / 2)

        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """ゴール受付結果のコールバック"""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'ゴール送信中に例外発生: {e}')
            self._current_index += 1
            self._navigate_to_next()
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                f'ウェイポイント {self._current_index + 1} が拒否されました。スキップします')
            self._current_index += 1
            self._navigate_to_next()
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """フィードバックコールバック (NavigateToPose)"""
        # NavigateToPose のフィードバックには距離情報等がある
        pass

    def _result_callback(self, future):
        """結果コールバック — 成功/失敗に関わらず次のウェイポイントへ進む"""
        wp = self._waypoints[self._current_index]

        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().error(f'アクション結果取得中に例外発生: {e}')
            status = GoalStatus.STATUS_UNKNOWN

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'ウェイポイント {self._current_index + 1}/{self._total_waypoints} 到達 '
                f'({wp.x:.2f}, {wp.y:.2f})')
        else:
            self.get_logger().warn(
                f'ウェイポイント {self._current_index + 1} 失敗 (status: {status})。'
                f'次へ進みます')

        self._current_index += 1
        self._navigate_to_next()


def main(args=None):
    """ノードのエントリーポイント"""
    rclpy.init(args=args)
    node = CoverageCommanderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('カバレッジコマンダーを停止します')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
