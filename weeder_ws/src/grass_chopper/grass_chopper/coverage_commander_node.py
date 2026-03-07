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
  - obstacles (double[][]): 障害物頂点座標リスト [[x1,y1,x2,y2,...], ...]
  - enable_remow (bool): 刈り残し再走行を有効化 (default: false)
  - remow_min_area (double): 再走行対象の最小面積 [m²] (default: 0.5)
  - remow_max_iterations (int): 再走行の最大回数 (default: 1)
  - auto_detect_region (bool): /map から走行領域を自動検出 (default: false)
  - auto_detect_robot_radius (double): 自動検出時の安全マージン [m] (default: 0.2)
  - start_index (int): 開始ウェイポイントのインデックス (default: 0)

配信:
  - /coverage_progress (std_msgs/Int32MultiArray): [completed_index, total_waypoints]
============================================================================
"""

import math

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from shapely.geometry import Polygon
from std_msgs.msg import Int32MultiArray

from grass_chopper.coverage_planner import (
    CoverageParams,
    generate_boustrophedon_waypoints,
    estimate_coverage_ratio,
)
from grass_chopper.coverage_tracker import (
    GridConfig,
    detect_uncovered_regions,
)
from grass_chopper.map_region_detector import (
    detect_obstacles_from_map,
    extract_free_regions,
    occupancy_grid_to_binary,
    select_largest_region,
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
        # 障害物: 各障害物は [x1, y1, x2, y2, ...] のフラット配列で表現
        # 複数障害物は obstacles_0, obstacles_1, ... で指定
        self.declare_parameter(
            'num_obstacles', 0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='障害物の数'
            )
        )

        # --- 再走行パラメータ ---
        self.declare_parameter(
            'enable_remow', False,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='刈り残し再走行を有効化'
            )
        )
        self.declare_parameter(
            'remow_min_area', 0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='再走行対象の最小面積 [m²]'
            )
        )
        self.declare_parameter(
            'remow_max_iterations', 1,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='再走行の最大回数'
            )
        )

        # --- 再開パラメータ ---
        self.declare_parameter(
            'start_index', 0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='開始ウェイポイントのインデックス (中断再開用)'
            )
        )

        # --- 自動領域検出パラメータ ---
        self.declare_parameter(
            'auto_detect_region', False,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='/map から走行領域・障害物を自動検出'
            )
        )
        self.declare_parameter(
            'auto_detect_robot_radius', 0.2,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='自動検出時の安全マージン [m]'
            )
        )

        # --- NavigateToPose アクションクライアント ---
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # --- /coverage_progress 配信 (mission_tree_node が購読) ---
        self._progress_pub = self.create_publisher(
            Int32MultiArray, '/coverage_progress', 10)

        # ウェイポイントキューと進捗管理
        self._waypoints = []
        self._current_index = 0
        self._total_waypoints = 0
        self._remow_iteration = 0
        self._coverage_params = None  # _start_coverage で設定
        self._target_polygon = None   # カバレッジ対象ポリゴン (再走行フィルタ用)
        self._obstacles = []          # 障害物リスト (再走行時にも使用)
        self._coverage_grid_msg = None  # 最新の /coverage_grid メッセージ

        # /coverage_grid 購読 (再走行用)
        self._coverage_grid_sub = self.create_subscription(
            OccupancyGrid, '/coverage_grid',
            self._coverage_grid_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))

        # /map 購読 (自動領域検出用)
        self._map_msg = None
        if self.get_parameter('auto_detect_region').value:
            from rclpy.qos import DurabilityPolicy
            map_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._map_sub = self.create_subscription(
                OccupancyGrid, '/map', self._map_callback, map_qos)
            self.get_logger().info('自動領域検出モード: /map を待機中...')

        self.get_logger().info('カバレッジコマンダーノードを起動しました')

        # 起動後すぐにカバレッジ走行を開始
        self._timer = self.create_timer(1.0, self._start_coverage)

    def _map_callback(self, msg: OccupancyGrid):
        """自動領域検出用: /map 受信"""
        if self._map_msg is not None:
            return  # 初回のみ
        self._map_msg = msg
        self.get_logger().info(
            f'/map 受信: {msg.info.width}x{msg.info.height}, '
            f'resolution={msg.info.resolution:.3f}m')

    def _detect_region_from_map(self):
        """
        /map から走行領域と障害物を自動検出する

        Returns:
            (polygon, obstacles) or (None, []) if detection failed
        """
        if self._map_msg is None:
            self.get_logger().warn('自動検出: /map 未受信')
            return None, []

        msg = self._map_msg
        config = GridConfig(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
        )
        grid_data = np.array(msg.data, dtype=np.int8)
        robot_radius = self.get_parameter('auto_detect_robot_radius').value

        # 二値化 → 走行領域抽出
        binary = occupancy_grid_to_binary(grid_data, config)
        regions = extract_free_regions(
            binary, config, min_area=1.0, robot_radius=robot_radius)
        polygon = select_largest_region(regions)

        if polygon is None:
            self.get_logger().error('自動検出: 走行可能領域が見つかりません')
            return None, []

        self.get_logger().info(
            f'自動検出: 走行領域 {polygon.area:.1f} m² '
            f'({len(regions)} 候補中最大)')

        # 障害物抽出
        obstacles = detect_obstacles_from_map(grid_data, config, min_area=0.1)
        if obstacles:
            self.get_logger().info(f'自動検出: 障害物 {len(obstacles)} 個')

        return polygon, obstacles

    def _start_coverage(self):
        """カバレッジ走行を開始する (タイマーコールバック、1回だけ実行)"""
        self._timer.cancel()

        auto_detect = self.get_parameter('auto_detect_region').value

        # パラメータ取得
        swath_width = self.get_parameter('swath_width').value
        margin = self.get_parameter('margin').value
        direction = self.get_parameter('direction').value

        if auto_detect:
            # /map から自動検出
            if self._map_msg is None:
                # /map がまだ来ていない → タイマー再開して待機
                self.get_logger().info('自動検出: /map 待機中... (5秒後に再試行)')
                self._timer = self.create_timer(5.0, self._start_coverage)
                return

            polygon, obstacles = self._detect_region_from_map()
            if polygon is None:
                return
        else:
            region = self.get_parameter('region').value
            if len(region) != 4:
                self.get_logger().error(
                    f'region パラメータは [min_x, min_y, max_x, max_y] の4要素必要: {region}')
                return

            min_x, min_y, max_x, max_y = region
            polygon = Polygon([
                (min_x, min_y),
                (max_x, min_y),
                (max_x, max_y),
                (min_x, max_y),
            ])

            # 障害物パラメータ取得
            obstacles = self._load_obstacles()

        params = CoverageParams(
            swath_width=swath_width,
            margin=margin,
            direction=direction,
        )
        self._coverage_params = params
        self._target_polygon = polygon
        self._obstacles = obstacles
        if obstacles:
            self.get_logger().info(f'障害物: {len(obstacles)} 個')

        # ウェイポイント生成
        self._waypoints = generate_boustrophedon_waypoints(
            polygon, params, obstacles=obstacles or None)
        if not self._waypoints:
            self.get_logger().error('ウェイポイントを生成できませんでした')
            return

        self._total_waypoints = len(self._waypoints)

        # start_index パラメータで中断地点から再開
        start_index = self.get_parameter('start_index').value
        if start_index > 0 and start_index < self._total_waypoints:
            self._current_index = start_index
            self.get_logger().info(
                f'中断地点から再開: ウェイポイント {start_index}/{self._total_waypoints}')
        else:
            self._current_index = 0

        # カバレッジ率推定
        ratio = estimate_coverage_ratio(self._waypoints, polygon, swath_width)
        bounds = polygon.bounds  # (minx, miny, maxx, maxy)
        self.get_logger().info(
            f'カバレッジ走行計画: {self._total_waypoints} ウェイポイント, '
            f'推定カバレッジ率: {ratio:.1%}, '
            f'領域: [{bounds[0]:.1f}, {bounds[1]:.1f}] ~ [{bounds[2]:.1f}, {bounds[3]:.1f}]'
        )

        # アクションサーバー接続待ち
        self.get_logger().info('Nav2 アクションサーバーの接続を待機中...')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 アクションサーバーに接続できませんでした')
            return

        # コストマップ準備のため待機 (SLAM + Nav2 の初期化完了待ち)
        self.get_logger().info('コストマップ準備待ち (10秒)...')
        import time
        time.sleep(10.0)

        # 最初のウェイポイントへナビゲーション開始
        self._navigate_to_next()

    def _load_obstacles(self) -> list:
        """障害物パラメータを Shapely Polygon リストに変換"""
        num = self.get_parameter('num_obstacles').value
        if num <= 0:
            return []

        obstacles = []
        for i in range(num):
            param_name = f'obstacle_{i}'
            # 動的パラメータ宣言 (未宣言の場合)
            if not self.has_parameter(param_name):
                self.declare_parameter(
                    param_name, [],
                    ParameterDescriptor(
                        type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                        description=f'障害物 {i} の頂点座標 [x1,y1,x2,y2,...]'
                    )
                )
            coords = self.get_parameter(param_name).value
            if len(coords) < 6:  # 最低 3 頂点 (6 座標) 必要
                self.get_logger().warn(
                    f'obstacle_{i}: 頂点数不足 ({len(coords)} 座標)、スキップ')
                continue
            # フラット配列を (x, y) ペアに変換
            points = [(coords[j], coords[j + 1])
                      for j in range(0, len(coords) - 1, 2)]
            try:
                poly = Polygon(points)
                if poly.is_valid and not poly.is_empty:
                    obstacles.append(poly)
                else:
                    self.get_logger().warn(
                        f'obstacle_{i}: 無効なポリゴン、スキップ')
            except Exception as e:
                self.get_logger().warn(
                    f'obstacle_{i}: ポリゴン生成エラー: {e}')

        return obstacles

    def _coverage_grid_callback(self, msg: OccupancyGrid):
        """最新の /coverage_grid を保持"""
        self._coverage_grid_msg = msg

    def _on_coverage_complete(self):
        """カバレッジ走行完了時の処理 (再走行判定)"""
        enable_remow = self.get_parameter('enable_remow').value
        max_iter = self.get_parameter('remow_max_iterations').value

        if not enable_remow or self._remow_iteration >= max_iter:
            self.get_logger().info('カバレッジ走行完了!')
            raise SystemExit(0)

        if self._coverage_grid_msg is None:
            self.get_logger().warn(
                '再走行: /coverage_grid が未受信のためスキップ')
            self.get_logger().info('カバレッジ走行完了!')
            raise SystemExit(0)

        # /coverage_grid から未カバー領域を検出
        min_area = self.get_parameter('remow_min_area').value
        msg = self._coverage_grid_msg
        grid_config = GridConfig(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
        )
        grid_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))

        uncovered = detect_uncovered_regions(grid_data, grid_config,
                                             min_area=min_area)

        # ターゲット領域外の未カバー領域をフィルタリング
        if self._target_polygon and uncovered:
            valid = []
            for region in uncovered:
                inter = region.intersection(self._target_polygon)
                if inter.is_empty:
                    continue
                geoms = inter.geoms if hasattr(inter, 'geoms') else [inter]
                for geom in geoms:
                    if isinstance(geom, Polygon) and geom.area >= min_area:
                        valid.append(geom)
            uncovered = valid

        if not uncovered:
            self.get_logger().info(
                '再走行: 刈り残し領域なし。カバレッジ走行完了!')
            raise SystemExit(0)

        self._remow_iteration += 1
        total_area = sum(r.area for r in uncovered)
        self.get_logger().info(
            f'再走行 {self._remow_iteration}/{max_iter}: '
            f'{len(uncovered)} 領域, 合計面積 {total_area:.2f} m²')

        # 各未カバー領域のウェイポイントを生成
        all_waypoints = []
        params = self._coverage_params
        for region in uncovered:
            wps = generate_boustrophedon_waypoints(
                region, CoverageParams(
                    swath_width=params.swath_width,
                    margin=0.0,  # 再走行ではマージン不要
                    direction=params.direction,
                ),
                obstacles=self._obstacles or None)
            all_waypoints.extend(wps)

        if not all_waypoints:
            self.get_logger().info(
                '再走行: ウェイポイント生成不可。カバレッジ走行完了!')
            raise SystemExit(0)

        self._waypoints = all_waypoints
        self._total_waypoints = len(all_waypoints)
        self._current_index = 0

        self.get_logger().info(
            f'再走行: {self._total_waypoints} ウェイポイントで開始')
        self._navigate_to_next()

    def _navigate_to_next(self):
        """次のウェイポイントへ NavigateToPose を送信"""
        if self._current_index >= self._total_waypoints:
            self._on_coverage_complete()
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

        # 進捗を配信 (mission_tree_node が購読して記憶)
        # data = [completed_index, total_waypoints]
        progress_msg = Int32MultiArray()
        progress_msg.data = [self._current_index, self._total_waypoints]
        self._progress_pub.publish(progress_msg)

        self._current_index += 1
        self._navigate_to_next()


def main(args=None):
    """ノードのエントリーポイント"""
    rclpy.init(args=args)
    node = CoverageCommanderNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        node.get_logger().info('カバレッジコマンダーを停止します')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
