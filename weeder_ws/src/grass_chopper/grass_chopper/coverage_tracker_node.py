"""
============================================================================
カバレッジ追跡 ROS 2 アダプターノード (coverage_tracker_node)
============================================================================
/map (OccupancyGrid) と /tf (map→base_link) を購読し、
CoverageTracker 純粋ロジックでカバレッジ率をリアルタイム計算する。

結果は以下のトピックに配信:
  - /coverage_grid (OccupancyGrid): カバレッジグリッド
  - /coverage_ratio (Float32): カバレッジ率 (0.0 ~ 1.0)

使い方:
  ros2 run grass_chopper coverage_tracker_node \
    --ros-args -p use_sim_time:=true -p tool_radius:=0.5

パラメータ:
  - tool_radius (double): 作業幅/2 [m] (default: 0.5)
  - update_frequency (double): 更新頻度 [Hz] (default: 5.0)
============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import tf2_ros
import numpy as np

from grass_chopper.coverage_tracker import CoverageTracker, GridConfig


class CoverageTrackerNode(Node):
    """カバレッジ追跡ノード: TF + /map → /coverage_grid + /coverage_ratio"""

    def __init__(self):
        super().__init__('coverage_tracker_node')

        # --- パラメータ ---
        self.declare_parameter(
            'tool_radius', 0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='作業幅/2 [m]'
            )
        )
        self.declare_parameter(
            'update_frequency', 5.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='更新頻度 [Hz]'
            )
        )

        self._tool_radius = self.get_parameter('tool_radius').value
        update_freq = self.get_parameter('update_frequency').value

        # --- TF リスナー ---
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # --- /map 購読 (latched QoS) ---
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos)

        # --- パブリッシャー ---
        self._grid_pub = self.create_publisher(
            OccupancyGrid, '/coverage_grid', 10)
        self._ratio_pub = self.create_publisher(
            Float32, '/coverage_ratio', 10)

        # --- 内部状態 ---
        self._tracker = None
        self._map_msg = None  # 配信用メタデータ保持

        # --- タイマー (TF 取得 + mark_covered) ---
        timer_period = 1.0 / update_freq
        self._timer = self.create_timer(timer_period, self._update_callback)

        self.get_logger().info(
            f'カバレッジ追跡ノード起動: tool_radius={self._tool_radius:.2f}m, '
            f'freq={update_freq:.1f}Hz')

    def _map_callback(self, msg: OccupancyGrid):
        """初回 /map 受信時に CoverageTracker を初期化"""
        if self._tracker is not None:
            return  # 初回のみ

        config = GridConfig(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
        )
        self._tracker = CoverageTracker(config)
        self._map_msg = msg

        self.get_logger().info(
            f'/map 受信: {config.width}x{config.height}, '
            f'resolution={config.resolution:.3f}m, '
            f'origin=({config.origin_x:.2f}, {config.origin_y:.2f})')

    def _update_callback(self):
        """タイマーコールバック: TF 取得 → mark_covered → 配信"""
        if self._tracker is None:
            return

        # map → base_link の TF を取得
        try:
            transform = self._tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        # カバレッジグリッド更新
        self._tracker.mark_covered(x, y, self._tool_radius)

        # カバレッジ率配信
        ratio = self._tracker.get_coverage_ratio()
        ratio_msg = Float32()
        ratio_msg.data = float(ratio)
        self._ratio_pub.publish(ratio_msg)

        # カバレッジグリッド配信
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        grid_msg.info = self._map_msg.info
        grid_data = self._tracker.to_occupancy_grid()
        grid_msg.data = grid_data.flatten().tolist()
        self._grid_pub.publish(grid_msg)


def main(args=None):
    """ノードのエントリーポイント"""
    rclpy.init(args=args)
    node = CoverageTrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('カバレッジ追跡ノードを停止します')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
