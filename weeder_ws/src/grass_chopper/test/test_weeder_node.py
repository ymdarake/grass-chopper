"""
============================================================================
WeederNode ユニットテスト
============================================================================
Phase 1 の既存ロジック（前方スキャン + 左回転回避）のテスト。
TDD のリファクタリング安全ネットとして機能する。

テスト方針:
  - rclpy.init/shutdown は module スコープの fixture で管理
  - cmd_pub.publish を MagicMock でモック化して出力を検証
  - LaserScan メッセージはヘルパー関数で生成
============================================================================
"""

import math
from unittest.mock import MagicMock

import pytest
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from grass_chopper.weeder_node import WeederNode


# ============================================================================
# Fixtures
# ============================================================================

@pytest.fixture(scope='module')
def rclpy_init():
    """rclpy のライフサイクルをモジュールスコープで管理"""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node(rclpy_init):
    """テスト用の WeederNode インスタンスを生成・破棄"""
    n = WeederNode()
    n.cmd_pub.publish = MagicMock()
    yield n
    n.destroy_node()


# ============================================================================
# ヘルパー関数
# ============================================================================

def create_laser_scan(
    distances=2.0,
    num_samples=360,
    angle_min=-math.pi,
    angle_max=math.pi,
    range_min=0.12,
    range_max=10.0,
):
    """
    LaserScan メッセージを生成するヘルパー

    Args:
        distances: float の場合は全方位同一距離、list の場合はそのまま使用
        num_samples: サンプル数（デフォルト360）
        angle_min: スキャン開始角度 [rad]
        angle_max: スキャン終了角度 [rad]
        range_min: 有効最小距離 [m]
        range_max: 有効最大距離 [m]

    Returns:
        LaserScan メッセージ
    """
    msg = LaserScan()
    msg.angle_min = angle_min
    msg.angle_max = angle_max
    msg.angle_increment = (angle_max - angle_min) / num_samples
    msg.range_min = range_min
    msg.range_max = range_max

    if isinstance(distances, (int, float)):
        msg.ranges = [float(distances)] * num_samples
    else:
        msg.ranges = [float(d) for d in distances]

    return msg


def create_laser_scan_with_zones(
    front=2.0,
    left=2.0,
    right=2.0,
    back=2.0,
    num_samples=360,
    range_min=0.12,
    range_max=10.0,
):
    """
    ゾーンごとに距離を設定した LaserScan を生成

    ゾーン定義（angle_min=-pi の場合のインデックス）:
      - index 0 = -180度（後方）
      - index 90 = -90度（右）
      - index 180 = 0度（正面）
      - index 270 = +90度（左）

    Args:
        front: 正面 (150-210) の距離
        left: 左側 (240-300) の距離
        right: 右側 (60-120) の距離
        back: 後方 (0-59, 301-359) の距離
    """
    distances = [float(back)] * num_samples

    # 右側: index 60-120 (-120度 ~ -60度)
    for i in range(60, 121):
        distances[i] = float(right)

    # 正面: index 150-210 (-30度 ~ +30度)
    for i in range(150, 211):
        distances[i] = float(front)

    # 左側: index 240-300 (+60度 ~ +120度)
    for i in range(240, 301):
        distances[i] = float(left)

    return create_laser_scan(
        distances=distances,
        num_samples=num_samples,
        range_min=range_min,
        range_max=range_max,
    )


def get_published_twist(node) -> Twist:
    """最後にパブリッシュされた Twist メッセージを取得"""
    assert node.cmd_pub.publish.called, 'publish が呼ばれていません'
    return node.cmd_pub.publish.call_args[0][0]


# ============================================================================
# Phase 1 ロジックのテスト
# ============================================================================

class TestPhase1BasicBehavior:
    """Phase 1 の基本動作テスト"""

    def test_forward_when_all_clear(self, node):
        """全方位が安全距離以上 → 直進"""
        msg = create_laser_scan(distances=2.0)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.2)
        assert twist.angular.z == pytest.approx(0.0)

    def test_turn_left_when_obstacle_ahead(self, node):
        """前方に障害物 (0.3m) → 停止して左回転"""
        msg = create_laser_scan_with_zones(front=0.3)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(0.5)

    def test_forward_when_obstacle_only_behind(self, node):
        """後方のみ障害物、前方クリア → 直進"""
        msg = create_laser_scan_with_zones(front=2.0, back=0.3)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.2)
        assert twist.angular.z == pytest.approx(0.0)

    def test_forward_when_obstacle_only_sides(self, node):
        """左右のみ障害物、前方クリア → 直進"""
        msg = create_laser_scan_with_zones(front=2.0, left=0.3, right=0.3)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.2)
        assert twist.angular.z == pytest.approx(0.0)

    def test_turn_when_obstacle_at_safe_distance_boundary(self, node):
        """障害物がちょうど安全距離 (0.5m) → 安全とみなす（距離 < safe_distance で回避）"""
        msg = create_laser_scan_with_zones(front=0.5)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        # 0.5 は safe_distance と同値なので < ではない → 直進
        assert twist.linear.x == pytest.approx(0.2)
        assert twist.angular.z == pytest.approx(0.0)

    def test_turn_when_obstacle_just_below_safe_distance(self, node):
        """障害物が安全距離のすぐ内側 (0.49m) → 回転回避"""
        msg = create_laser_scan_with_zones(front=0.49)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(0.5)


class TestPhase1EdgeCases:
    """Phase 1 のエッジケーステスト"""

    def test_empty_ranges(self, node):
        """空の ranges → パブリッシュしない（早期リターン）"""
        msg = create_laser_scan(distances=[])
        msg.ranges = []
        node.cmd_pub.publish.reset_mock()
        node.scan_callback(msg)

        # 空のrangesの場合、num_samples==0 で早期リターン
        assert not node.cmd_pub.publish.called

    def test_all_inf_ranges(self, node):
        """全て inf → 安全とみなして直進"""
        msg = create_laser_scan(distances=float('inf'))
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.2)
        assert twist.angular.z == pytest.approx(0.0)

    def test_all_nan_ranges(self, node):
        """全て NaN → 安全とみなして直進"""
        msg = create_laser_scan(distances=float('nan'))
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.2)
        assert twist.angular.z == pytest.approx(0.0)

    def test_mixed_inf_nan_with_valid(self, node):
        """inf/NaN が混在するが、前方に有効な近距離データ → 回避"""
        distances = [2.0] * 360
        # 前方ゾーン (150-210) に inf/NaN と有効な近距離データを混在
        for i in range(150, 180):
            distances[i] = float('inf')
        for i in range(180, 195):
            distances[i] = float('nan')
        distances[195] = 0.3  # 有効な近距離データ
        for i in range(196, 211):
            distances[i] = 2.0

        msg = create_laser_scan(distances=distances)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(0.5)

    def test_ranges_below_range_min_filtered(self, node):
        """range_min (0.12m) 以下のデータはフィルタリングされる"""
        distances = [2.0] * 360
        # 前方ゾーンに range_min 以下のデータを設定
        for i in range(150, 211):
            distances[i] = 0.05  # range_min (0.12) より小さい
        msg = create_laser_scan(distances=distances, range_min=0.12)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        # range_min 以下はフィルタリングされるため、有効データなし → 安全 → 直進
        assert twist.linear.x == pytest.approx(0.2)
        assert twist.angular.z == pytest.approx(0.0)

    def test_ranges_at_range_max_filtered(self, node):
        """range_max 以上のデータはフィルタリングされる"""
        distances = [2.0] * 360
        # 前方ゾーンに range_max 以上のデータを設定
        for i in range(150, 211):
            distances[i] = 10.0  # range_max と同値
        msg = create_laser_scan(distances=distances, range_max=10.0)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        # range_max 以上はフィルタリングされるため、有効データなし → 安全 → 直進
        assert twist.linear.x == pytest.approx(0.2)
        assert twist.angular.z == pytest.approx(0.0)


class TestPhase1Parameters:
    """Phase 1 のパラメータ初期値テスト"""

    def test_default_safe_distance(self, node):
        """安全距離のデフォルト値が 0.5m"""
        assert node.safe_distance == pytest.approx(0.5)

    def test_default_forward_speed(self, node):
        """前進速度のデフォルト値が 0.2 m/s"""
        assert node.forward_speed == pytest.approx(0.2)

    def test_default_turn_speed(self, node):
        """回転速度のデフォルト値が 0.5 rad/s"""
        assert node.turn_speed == pytest.approx(0.5)


# ============================================================================
# Step 2: ROS 2 パラメータ化のテスト
# ============================================================================

class TestRos2Parameters:
    """ROS 2 declare_parameter によるパラメータ管理テスト"""

    def test_safe_distance_declared_as_parameter(self, node):
        """safe_distance が ROS 2 パラメータとして宣言されている"""
        param = node.get_parameter('safe_distance')
        assert param.value == pytest.approx(0.5)

    def test_forward_speed_declared_as_parameter(self, node):
        """forward_speed が ROS 2 パラメータとして宣言されている"""
        param = node.get_parameter('forward_speed')
        assert param.value == pytest.approx(0.2)

    def test_turn_speed_declared_as_parameter(self, node):
        """turn_speed が ROS 2 パラメータとして宣言されている"""
        param = node.get_parameter('turn_speed')
        assert param.value == pytest.approx(0.5)

    def test_parameter_change_affects_behavior(self, node):
        """パラメータ変更が行動に反映される"""
        from rclpy.parameter import Parameter

        # safe_distance を 1.0m に変更
        node.set_parameters([
            Parameter('safe_distance', Parameter.Type.DOUBLE, 1.0)
        ])

        # 前方 0.8m（元の 0.5m より大きいが、新しい 1.0m より小さい）
        msg = create_laser_scan_with_zones(front=0.8)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        # safe_distance=1.0 なので 0.8 < 1.0 → 回避
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z > 0.0

        # 元に戻す
        node.set_parameters([
            Parameter('safe_distance', Parameter.Type.DOUBLE, 0.5)
        ])

    def test_dynamic_forward_speed_change(self, node):
        """forward_speed の動的変更が直進時に反映される"""
        from rclpy.parameter import Parameter

        node.set_parameters([
            Parameter('forward_speed', Parameter.Type.DOUBLE, 0.3)
        ])

        msg = create_laser_scan(distances=2.0)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.3)

        # 元に戻す
        node.set_parameters([
            Parameter('forward_speed', Parameter.Type.DOUBLE, 0.2)
        ])

    def test_dynamic_turn_speed_change(self, node):
        """turn_speed の動的変更が回避時に反映される"""
        from rclpy.parameter import Parameter

        node.set_parameters([
            Parameter('turn_speed', Parameter.Type.DOUBLE, 0.8)
        ])

        msg = create_laser_scan_with_zones(front=0.3)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.angular.z == pytest.approx(0.8)

        # 元に戻す
        node.set_parameters([
            Parameter('turn_speed', Parameter.Type.DOUBLE, 0.5)
        ])


# ============================================================================
# Step 3: LiDAR ゾーン解析のテスト
# ============================================================================

class TestDegToIndex:
    """_deg_to_index の変換テスト"""

    def test_front_0_degrees(self, node):
        """0度（正面）のインデックス変換"""
        idx = node._deg_to_index(0.0, -math.pi, math.pi / 180)
        assert idx == 180

    def test_right_minus_90_degrees(self, node):
        """-90度（右）のインデックス変換"""
        idx = node._deg_to_index(-90.0, -math.pi, math.pi / 180)
        assert idx == 90

    def test_left_plus_90_degrees(self, node):
        """+90度（左）のインデックス変換"""
        idx = node._deg_to_index(90.0, -math.pi, math.pi / 180)
        assert idx == 270

    def test_back_minus_180_degrees(self, node):
        """-180度（後方）のインデックス変換"""
        idx = node._deg_to_index(-180.0, -math.pi, math.pi / 180)
        assert idx == 0


class TestGetZoneStats:
    """get_zone_stats のテスト"""

    def test_front_zone_min_and_avg(self, node):
        """正面ゾーンの最小・平均距離が正しく計算される"""
        distances = [5.0] * 360
        # 正面ゾーン (-30 ~ +30) に値を設定
        for i in range(150, 211):
            distances[i] = 1.0
        distances[180] = 0.5  # 正面ど真ん中に近い値

        min_d, avg_d = node.get_zone_stats(
            distances, -30.0, 30.0, 0.12, 10.0
        )
        assert min_d == pytest.approx(0.5)
        assert avg_d < 1.0  # 平均は 0.5 と 1.0 の間

    def test_left_zone(self, node):
        """左ゾーン (+60 ~ +120) の計算"""
        distances = [5.0] * 360
        for i in range(240, 301):
            distances[i] = 2.0

        min_d, avg_d = node.get_zone_stats(
            distances, 60.0, 120.0, 0.12, 10.0
        )
        assert min_d == pytest.approx(2.0)
        assert avg_d == pytest.approx(2.0)

    def test_right_zone(self, node):
        """右ゾーン (-120 ~ -60) の計算"""
        distances = [5.0] * 360
        for i in range(60, 121):
            distances[i] = 3.0

        min_d, avg_d = node.get_zone_stats(
            distances, -120.0, -60.0, 0.12, 10.0
        )
        assert min_d == pytest.approx(3.0)
        assert avg_d == pytest.approx(3.0)

    def test_inf_nan_filtered(self, node):
        """inf / NaN はフィルタリングされる"""
        distances = [5.0] * 360
        for i in range(150, 180):
            distances[i] = float('inf')
        for i in range(180, 200):
            distances[i] = float('nan')
        distances[200] = 1.5
        for i in range(201, 211):
            distances[i] = 2.0

        min_d, avg_d = node.get_zone_stats(
            distances, -30.0, 30.0, 0.12, 10.0
        )
        assert min_d == pytest.approx(1.5)
        assert not math.isinf(avg_d)
        assert not math.isnan(avg_d)

    def test_no_valid_data_returns_inf(self, node):
        """有効データなしの場合は (inf, inf) を返す"""
        distances = [float('inf')] * 360

        min_d, avg_d = node.get_zone_stats(
            distances, -30.0, 30.0, 0.12, 10.0
        )
        assert math.isinf(min_d)
        assert math.isinf(avg_d)

    def test_range_min_max_filtering(self, node):
        """range_min/range_max 外のデータはフィルタリングされる"""
        distances = [5.0] * 360
        for i in range(150, 211):
            distances[i] = 0.05  # range_min (0.12) 以下

        min_d, avg_d = node.get_zone_stats(
            distances, -30.0, 30.0, 0.12, 10.0
        )
        assert math.isinf(min_d)
        assert math.isinf(avg_d)


# ============================================================================
# Step 4: 状態遷移マシンのテスト
# ============================================================================

class TestStateMachine:
    """状態遷移マシンのテスト"""

    def test_initial_state_is_forward(self, node):
        """初期状態は FORWARD"""
        from grass_chopper.weeder_node import RobotState
        assert node.state == RobotState.FORWARD

    def test_forward_when_all_clear(self, node):
        """全方位クリア → FORWARD 状態を維持"""
        from grass_chopper.weeder_node import RobotState

        msg = create_laser_scan(distances=2.0)
        node.scan_callback(msg)

        assert node.state == RobotState.FORWARD
        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.2)
        assert twist.angular.z == pytest.approx(0.0)

    def test_avoid_left_when_left_is_clearer(self, node):
        """前方に障害物、左が空いている → AVOID_LEFT"""
        from grass_chopper.weeder_node import RobotState

        # 前方近い、左が遠い、右が近い
        msg = create_laser_scan_with_zones(front=0.3, left=3.0, right=0.5)
        node.scan_callback(msg)

        assert node.state == RobotState.AVOID_LEFT
        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z > 0  # 正 = 左回転

    def test_avoid_right_when_right_is_clearer(self, node):
        """前方に障害物、右が空いている → AVOID_RIGHT"""
        from grass_chopper.weeder_node import RobotState

        # 前方近い、右が遠い、左が近い
        msg = create_laser_scan_with_zones(front=0.3, left=0.5, right=3.0)
        node.scan_callback(msg)

        assert node.state == RobotState.AVOID_RIGHT
        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z < 0  # 負 = 右回転

    def test_return_to_forward_after_avoidance(self, node):
        """回避後、前方がクリアになったら FORWARD に復帰"""
        from grass_chopper.weeder_node import RobotState

        # まず障害物を検知して回避状態にする
        msg = create_laser_scan_with_zones(front=0.3, left=3.0, right=0.5)
        node.scan_callback(msg)
        assert node.state == RobotState.AVOID_LEFT

        # 前方がクリアになった（safe_distance * 1.2 = 0.6 以上）
        msg = create_laser_scan(distances=2.0)
        node.scan_callback(msg)
        assert node.state == RobotState.FORWARD

    def test_compute_twist_forward(self, node):
        """FORWARD 状態の Twist が正しい"""
        from grass_chopper.weeder_node import RobotState

        msg = create_laser_scan(distances=2.0)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(node.forward_speed)
        assert twist.angular.z == pytest.approx(0.0)

    def test_compute_twist_avoid_left(self, node):
        """AVOID_LEFT 状態の Twist が正しい"""
        msg = create_laser_scan_with_zones(front=0.3, left=3.0, right=0.5)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(node.turn_speed)

    def test_compute_twist_avoid_right(self, node):
        """AVOID_RIGHT 状態の Twist が正しい"""
        msg = create_laser_scan_with_zones(front=0.3, left=0.5, right=3.0)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(-node.turn_speed)

    def test_equal_sides_defaults_to_right(self, node):
        """左右が同距離の場合 → AVOID_RIGHT（デフォルト）"""
        from grass_chopper.weeder_node import RobotState

        msg = create_laser_scan_with_zones(front=0.3, left=2.0, right=2.0)
        node.scan_callback(msg)

        # 左右同距離の場合は右回避（left_avg > right_avg でなければ右）
        assert node.state == RobotState.AVOID_RIGHT


# ============================================================================
# Step 5: 壁沿い走行 (Wall Following) のテスト
# ============================================================================

class TestWallFollowing:
    """壁沿い走行 (PD制御) のテスト"""

    def test_wall_follow_parameters_declared(self, node):
        """壁沿い走行パラメータが宣言されている"""
        assert node.get_parameter('wall_target_distance').value == pytest.approx(0.5)
        assert node.get_parameter('wall_follow_kp').value == pytest.approx(1.5)
        assert node.get_parameter('wall_follow_kd').value == pytest.approx(0.3)
        assert node.get_parameter('wall_follow_speed').value == pytest.approx(0.15)
        assert node.get_parameter('angular_z_limit').value == pytest.approx(0.8)

    def test_forward_to_wall_follow_transition(self, node):
        """FORWARD 状態で右側に壁 → WALL_FOLLOW に遷移"""
        from grass_chopper.weeder_node import RobotState

        # まず FORWARD 状態にする
        msg = create_laser_scan(distances=2.0)
        node.scan_callback(msg)
        assert node.state == RobotState.FORWARD

        # 右側に壁（wall_target_distance * 1.5 = 0.75 未満）、前方クリア
        msg = create_laser_scan_with_zones(front=2.0, right=0.6, left=5.0)
        node.scan_callback(msg)
        assert node.state == RobotState.WALL_FOLLOW

    def test_wall_follow_to_forward_when_wall_gone(self, node):
        """WALL_FOLLOW 状態で壁がなくなった → FORWARD に遷移"""
        from grass_chopper.weeder_node import RobotState

        # まず WALL_FOLLOW 状態にする
        msg = create_laser_scan_with_zones(front=2.0, right=0.6, left=5.0)
        node.scan_callback(msg)
        # FORWARD -> WALL_FOLLOW にするため再度送信
        node.state = RobotState.WALL_FOLLOW

        # 右側の壁がなくなった（wall_target_distance * 2.0 = 1.0 以上）
        msg = create_laser_scan_with_zones(front=2.0, right=3.0, left=5.0)
        node.scan_callback(msg)
        assert node.state == RobotState.FORWARD

    def test_wall_follow_to_avoid_when_obstacle_ahead(self, node):
        """WALL_FOLLOW 中に前方障害物 → AVOID に遷移"""
        from grass_chopper.weeder_node import RobotState

        node.state = RobotState.WALL_FOLLOW

        # 前方に障害物、左が空いている
        msg = create_laser_scan_with_zones(front=0.3, left=3.0, right=0.5)
        node.scan_callback(msg)
        assert node.state == RobotState.AVOID_LEFT

    def test_pd_control_wall_at_target_distance(self, node):
        """壁がちょうど目標距離 → angular.z がほぼ 0"""
        from grass_chopper.weeder_node import RobotState

        node.state = RobotState.WALL_FOLLOW
        node.prev_wall_error = 0.0

        # 右側が目標距離 (0.5m)、前方クリア
        msg = create_laser_scan_with_zones(front=2.0, right=0.5, left=5.0)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.15)
        assert abs(twist.angular.z) < 0.1  # ほぼ 0

    def test_pd_control_wall_too_close(self, node):
        """壁に近すぎる → angular.z > 0（左に曲がって壁から離れる）"""
        from grass_chopper.weeder_node import RobotState

        node.state = RobotState.WALL_FOLLOW
        node.prev_wall_error = 0.0

        # 右側が近い (0.3m)、目標 0.5m
        msg = create_laser_scan_with_zones(front=2.0, right=0.3, left=5.0)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.angular.z > 0  # 左へ（壁から離れる）

    def test_pd_control_wall_too_far(self, node):
        """壁から遠すぎる → angular.z < 0（右に曲がって壁に近づく）"""
        from grass_chopper.weeder_node import RobotState

        node.state = RobotState.WALL_FOLLOW
        node.prev_wall_error = 0.0

        # 右側が遠い (0.7m)、目標 0.5m、ただし WALL_FOLLOW 脱出しないレベル
        msg = create_laser_scan_with_zones(front=2.0, right=0.7, left=5.0)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.angular.z < 0  # 右へ（壁に近づく）

    def test_angular_z_limit_applied(self, node):
        """角速度リミットが正しく適用される"""
        from grass_chopper.weeder_node import RobotState

        node.state = RobotState.WALL_FOLLOW
        node.prev_wall_error = 0.0

        # 極端に壁に近い → 大きな error → リミットで制限される
        msg = create_laser_scan_with_zones(front=2.0, right=0.15, left=5.0)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert abs(twist.angular.z) <= 0.8 + 0.01  # angular_z_limit


# ============================================================================
# Step 6: 行き止まり検知と U ターンのテスト
# ============================================================================

class TestUTurn:
    """行き止まり検知と U ターンのテスト"""

    def test_u_turn_parameter_declared(self, node):
        """u_turn_speed パラメータが宣言されている"""
        assert node.get_parameter('u_turn_speed').value == pytest.approx(1.0)

    def test_u_turn_when_three_sides_blocked(self, node):
        """三方が塞がれている → U_TURN に遷移"""
        from grass_chopper.weeder_node import RobotState

        msg = create_laser_scan_with_zones(front=0.3, left=0.3, right=0.3)
        node.scan_callback(msg)

        assert node.state == RobotState.U_TURN

    def test_u_turn_twist(self, node):
        """U_TURN 状態の Twist が正しい（停止して高速回転）"""
        from grass_chopper.weeder_node import RobotState

        msg = create_laser_scan_with_zones(front=0.3, left=0.3, right=0.3)
        node.scan_callback(msg)

        twist = get_published_twist(node)
        assert twist.linear.x == pytest.approx(0.0)
        assert twist.angular.z == pytest.approx(1.0)  # u_turn_speed

    def test_u_turn_to_forward_when_front_clears(self, node):
        """U_TURN 中に前方が開けた → FORWARD に復帰"""
        from grass_chopper.weeder_node import RobotState

        # まず U_TURN 状態にする
        msg = create_laser_scan_with_zones(front=0.3, left=0.3, right=0.3)
        node.scan_callback(msg)
        assert node.state == RobotState.U_TURN

        # 前方が十分に開けた（safe_distance * 1.5 = 0.75 以上）
        msg = create_laser_scan(distances=2.0)
        node.scan_callback(msg)
        assert node.state == RobotState.FORWARD

    def test_not_u_turn_when_only_two_sides_blocked(self, node):
        """2方向のみ塞がれている → U_TURN にならず AVOID"""
        from grass_chopper.weeder_node import RobotState

        # 前方と右が塞がれ、左は空いている
        msg = create_laser_scan_with_zones(front=0.3, left=3.0, right=0.3)
        node.scan_callback(msg)

        assert node.state == RobotState.AVOID_LEFT
        assert node.state != RobotState.U_TURN

    def test_u_turn_has_highest_priority(self, node):
        """U_TURN は他の全ての状態より優先度が高い"""
        from grass_chopper.weeder_node import RobotState

        # WALL_FOLLOW 状態から開始
        node.state = RobotState.WALL_FOLLOW

        # 三方が塞がれた → U_TURN に遷移（WALL_FOLLOW より優先）
        msg = create_laser_scan_with_zones(front=0.3, left=0.3, right=0.3)
        node.scan_callback(msg)

        assert node.state == RobotState.U_TURN
