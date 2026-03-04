"""
============================================================================
カバレッジ走行 コマンダーノード (coverage_commander_node)
============================================================================
NavigateThroughPoses アクションを使ってカバレッジ走行を実行する
ROS 2 アダプターノード。

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
from nav2_msgs.action import NavigateThroughPoses
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from shapely.geometry import Polygon

from grass_chopper.coverage_planner import (
    CoverageParams,
    generate_boustrophedon_waypoints,
    estimate_coverage_ratio,
)


class CoverageCommanderNode(Node):
    """カバレッジ走行を指示するノード"""

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

        # --- NavigateThroughPoses アクションクライアント ---
        self._action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')

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
        waypoints = generate_boustrophedon_waypoints(polygon, params)
        if not waypoints:
            self.get_logger().error('ウェイポイントを生成できませんでした')
            return

        # カバレッジ率推定
        ratio = estimate_coverage_ratio(waypoints, polygon, swath_width)
        self.get_logger().info(
            f'カバレッジ走行計画: {len(waypoints)} ウェイポイント, '
            f'推定カバレッジ率: {ratio:.1%}, '
            f'領域: [{min_x}, {min_y}] ~ [{max_x}, {max_y}]'
        )

        # NavigateThroughPoses アクションを送信
        self._send_through_poses(waypoints)

    def _send_through_poses(self, waypoints):
        """NavigateThroughPoses アクションにウェイポイントを送信"""
        self.get_logger().info('Nav2 アクションサーバーの接続を待機中...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 アクションサーバーに接続できませんでした')
            return

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []

        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp.x
            pose.pose.position.y = wp.y
            pose.pose.position.z = 0.0
            # yaw → quaternion (z 軸回転のみ)
            pose.pose.orientation.z = math.sin(wp.yaw / 2)
            pose.pose.orientation.w = math.cos(wp.yaw / 2)
            goal_msg.poses.append(pose)

        self.get_logger().info(f'{len(goal_msg.poses)} ウェイポイントを Nav2 に送信中...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """ゴール受付結果のコールバック"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('カバレッジ走行ゴールが拒否されました')
            return

        self.get_logger().info('カバレッジ走行ゴールが受理されました')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, feedback_msg):
        """フィードバックコールバック"""
        remaining = feedback_msg.feedback.number_of_poses_remaining
        self.get_logger().info(
            f'残りウェイポイント: {remaining}',
            throttle_duration_sec=5.0,
        )

    def _result_callback(self, future):
        """結果コールバック"""
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('カバレッジ走行完了!')
        else:
            self.get_logger().warn(f'カバレッジ走行終了 (status: {result.status})')


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
