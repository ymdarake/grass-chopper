"""
============================================================================
obstacle_avoidance 純粋ロジック ユニットテスト
============================================================================
rclpy 不要。Mac ホスト上で pytest のみで実行可能。
============================================================================
"""

import math

import pytest

from grass_chopper.obstacle_avoidance import (
    AvoidanceParams,
    RobotState,
    ScanConfig,
    analyze_zones,
    compute_twist,
    compute_wall_follow_twist,
    deg_to_index,
    get_zone_stats,
    update_state,
)


# ============================================================================
# ヘルパー
# ============================================================================

# 標準的な LiDAR 設定: -pi ~ +pi, 360サンプル → 1度/サンプル
DEFAULT_ANGLE_MIN = -math.pi
DEFAULT_ANGLE_INCREMENT = math.pi / 180  # 1度刻み
DEFAULT_RANGE_MIN = 0.12
DEFAULT_RANGE_MAX = 10.0
DEFAULT_SCAN_CONFIG = ScanConfig()


def make_ranges(front=2.0, left=2.0, right=2.0, back=2.0, num_samples=360):
    """ゾーンごとに距離を設定した ranges リストを生成 (LaserScan 不要)"""
    distances = [float(back)] * num_samples
    for i in range(60, 121):      # 右 (-120 ~ -60度)
        distances[i] = float(right)
    for i in range(150, 211):     # 正面 (-30 ~ +30度)
        distances[i] = float(front)
    for i in range(240, 301):     # 左 (+60 ~ +120度)
        distances[i] = float(left)
    return distances


def make_zones(front=2.0, left=2.0, right=2.0):
    """ゾーン辞書を直接生成 (analyze_zones を介さない)"""
    return {
        'front_min': front, 'front_avg': front,
        'left_min': left, 'left_avg': left,
        'right_min': right, 'right_avg': right,
    }


@pytest.fixture
def default_params():
    return AvoidanceParams()


# ============================================================================
# TestDegToIndex
# ============================================================================

class TestDegToIndex:
    """deg_to_index の変換テスト"""

    def test_front_0_degrees(self):
        """0度（正面）→ index 180"""
        idx = deg_to_index(0.0, DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_INCREMENT)
        assert idx == 180

    def test_right_minus_90_degrees(self):
        """-90度（右）→ index 90"""
        idx = deg_to_index(-90.0, DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_INCREMENT)
        assert idx == 90

    def test_left_plus_90_degrees(self):
        """+90度（左）→ index 270"""
        idx = deg_to_index(90.0, DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_INCREMENT)
        assert idx == 270

    def test_back_minus_180_degrees(self):
        """-180度（後方）→ index 0"""
        idx = deg_to_index(-180.0, DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_INCREMENT)
        assert idx == 0


# ============================================================================
# TestGetZoneStats
# ============================================================================

class TestGetZoneStats:
    """get_zone_stats のテスト"""

    def test_front_zone_min_and_avg(self):
        """正面ゾーンの最小・平均距離が正しく計算される"""
        distances = [5.0] * 360
        for i in range(150, 211):
            distances[i] = 1.0
        distances[180] = 0.5  # 正面ど真ん中

        min_d, avg_d = get_zone_stats(
            distances, -30.0, 30.0,
            DEFAULT_RANGE_MIN, DEFAULT_RANGE_MAX,
            DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_INCREMENT,
        )
        assert min_d == pytest.approx(0.5)
        assert avg_d < 1.0

    def test_left_zone(self):
        """左ゾーン (+60 ~ +120) の計算"""
        distances = [5.0] * 360
        for i in range(240, 301):
            distances[i] = 2.0

        min_d, avg_d = get_zone_stats(
            distances, 60.0, 120.0,
            DEFAULT_RANGE_MIN, DEFAULT_RANGE_MAX,
            DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_INCREMENT,
        )
        assert min_d == pytest.approx(2.0)
        assert avg_d == pytest.approx(2.0)

    def test_right_zone(self):
        """右ゾーン (-120 ~ -60) の計算"""
        distances = [5.0] * 360
        for i in range(60, 121):
            distances[i] = 3.0

        min_d, avg_d = get_zone_stats(
            distances, -120.0, -60.0,
            DEFAULT_RANGE_MIN, DEFAULT_RANGE_MAX,
            DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_INCREMENT,
        )
        assert min_d == pytest.approx(3.0)
        assert avg_d == pytest.approx(3.0)

    def test_inf_nan_filtered(self):
        """inf / NaN はフィルタリングされる"""
        distances = [5.0] * 360
        for i in range(150, 180):
            distances[i] = float('inf')
        for i in range(180, 200):
            distances[i] = float('nan')
        distances[200] = 1.5
        for i in range(201, 211):
            distances[i] = 2.0

        min_d, avg_d = get_zone_stats(
            distances, -30.0, 30.0,
            DEFAULT_RANGE_MIN, DEFAULT_RANGE_MAX,
            DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_INCREMENT,
        )
        assert min_d == pytest.approx(1.5)
        assert not math.isinf(avg_d)
        assert not math.isnan(avg_d)

    def test_no_valid_data_returns_inf(self):
        """有効データなし → (inf, inf)"""
        distances = [float('inf')] * 360

        min_d, avg_d = get_zone_stats(
            distances, -30.0, 30.0,
            DEFAULT_RANGE_MIN, DEFAULT_RANGE_MAX,
            DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_INCREMENT,
        )
        assert math.isinf(min_d)
        assert math.isinf(avg_d)

    def test_empty_ranges_returns_inf(self):
        """空の ranges → (inf, inf)"""
        min_d, avg_d = get_zone_stats(
            [], -30.0, 30.0,
            DEFAULT_RANGE_MIN, DEFAULT_RANGE_MAX,
            DEFAULT_ANGLE_MIN, DEFAULT_ANGLE_INCREMENT,
        )
        assert math.isinf(min_d)
        assert math.isinf(avg_d)


# ============================================================================
# TestAnalyzeZones
# ============================================================================

class TestAnalyzeZones:
    """analyze_zones の統合テスト"""

    def test_all_clear(self):
        """全方位 2.0m → 全ゾーンが 2.0m"""
        ranges = make_ranges(front=2.0, left=2.0, right=2.0)
        zones = analyze_zones(ranges, DEFAULT_SCAN_CONFIG)
        assert zones['front_min'] == pytest.approx(2.0)
        assert zones['left_min'] == pytest.approx(2.0)
        assert zones['right_min'] == pytest.approx(2.0)

    def test_obstacle_front_only(self):
        """前方のみ障害物"""
        ranges = make_ranges(front=0.3, left=2.0, right=2.0)
        zones = analyze_zones(ranges, DEFAULT_SCAN_CONFIG)
        assert zones['front_min'] == pytest.approx(0.3)
        assert zones['left_min'] == pytest.approx(2.0)
        assert zones['right_min'] == pytest.approx(2.0)

    def test_all_zones_different(self):
        """各ゾーンが異なる距離"""
        ranges = make_ranges(front=1.0, left=3.0, right=0.5)
        zones = analyze_zones(ranges, DEFAULT_SCAN_CONFIG)
        assert zones['front_min'] == pytest.approx(1.0)
        assert zones['left_min'] == pytest.approx(3.0)
        assert zones['right_min'] == pytest.approx(0.5)


# ============================================================================
# TestStateMachine
# ============================================================================

class TestStateMachine:
    """update_state の状態遷移テスト (純粋関数)"""

    def test_forward_stays_forward_when_clear(self, default_params):
        """全方位クリア → FORWARD 維持"""
        zones = make_zones(front=2.0, left=2.0, right=2.0)
        new_state, reset = update_state(RobotState.FORWARD, zones, default_params)
        assert new_state == RobotState.FORWARD

    def test_forward_to_avoid_left(self, default_params):
        """前方障害物 + 左が広い → AVOID_LEFT"""
        zones = make_zones(front=0.3, left=3.0, right=0.5)
        new_state, reset = update_state(RobotState.FORWARD, zones, default_params)
        assert new_state == RobotState.AVOID_LEFT

    def test_forward_to_avoid_right(self, default_params):
        """前方障害物 + 右が広い → AVOID_RIGHT"""
        zones = make_zones(front=0.3, left=0.5, right=3.0)
        new_state, reset = update_state(RobotState.FORWARD, zones, default_params)
        assert new_state == RobotState.AVOID_RIGHT

    def test_equal_sides_defaults_to_right(self, default_params):
        """左右同距離 → AVOID_RIGHT (左が広くないので右)"""
        zones = make_zones(front=0.3, left=2.0, right=2.0)
        new_state, reset = update_state(RobotState.FORWARD, zones, default_params)
        assert new_state == RobotState.AVOID_RIGHT

    def test_avoid_to_forward_when_clear(self, default_params):
        """回避中に前方がクリア (safe_distance * 1.2 以上) → FORWARD"""
        zones = make_zones(front=2.0, left=2.0, right=2.0)
        new_state, _ = update_state(RobotState.AVOID_LEFT, zones, default_params)
        assert new_state == RobotState.FORWARD

    def test_u_turn_when_three_sides_blocked(self, default_params):
        """三方塞がり → U_TURN"""
        zones = make_zones(front=0.3, left=0.3, right=0.3)
        new_state, _ = update_state(RobotState.FORWARD, zones, default_params)
        assert new_state == RobotState.U_TURN

    def test_forward_to_wall_follow(self, default_params):
        """FORWARD + 右側に壁が近い → WALL_FOLLOW"""
        zones = make_zones(front=2.0, left=5.0, right=0.6)
        new_state, reset = update_state(RobotState.FORWARD, zones, default_params)
        assert new_state == RobotState.WALL_FOLLOW
        assert reset is True  # prev_wall_error リセット

    def test_wall_follow_continues(self, default_params):
        """WALL_FOLLOW + 右側に壁あり → WALL_FOLLOW 継続"""
        zones = make_zones(front=2.0, left=5.0, right=0.5)
        new_state, _ = update_state(RobotState.WALL_FOLLOW, zones, default_params)
        assert new_state == RobotState.WALL_FOLLOW

    def test_wall_follow_to_forward_when_wall_gone(self, default_params):
        """WALL_FOLLOW + 右壁消失 (wall_target * 2.0 以上) → FORWARD"""
        zones = make_zones(front=2.0, left=5.0, right=3.0)
        new_state, reset = update_state(RobotState.WALL_FOLLOW, zones, default_params)
        assert new_state == RobotState.FORWARD
        assert reset is True

    def test_wall_follow_to_avoid_resets_error(self, default_params):
        """WALL_FOLLOW → AVOID 遷移時に wall_error_reset が True"""
        zones = make_zones(front=0.3, left=3.0, right=0.5)
        new_state, reset = update_state(RobotState.WALL_FOLLOW, zones, default_params)
        assert new_state == RobotState.AVOID_LEFT
        assert reset is True

    def test_avoid_direction_maintained_while_obstacle(self, default_params):
        """回避中に左右の距離が変動しても回避方向を維持 (チャタリング防止)"""
        # 最初は左回避
        zones = make_zones(front=0.3, left=3.0, right=2.5)
        new_state, _ = update_state(RobotState.FORWARD, zones, default_params)
        assert new_state == RobotState.AVOID_LEFT

        # 右の方が広くなっても AVOID_LEFT を維持
        zones2 = make_zones(front=0.3, left=2.5, right=3.0)
        new_state2, _ = update_state(RobotState.AVOID_LEFT, zones2, default_params)
        assert new_state2 == RobotState.AVOID_LEFT


# ============================================================================
# TestWallFollowing (compute_wall_follow_twist)
# ============================================================================

class TestWallFollowing:
    """壁沿い走行 PD 制御のテスト (純粋関数)"""

    def test_wall_at_target_distance(self, default_params):
        """壁が目標距離 → angular_z がほぼ 0"""
        cmd, new_error = compute_wall_follow_twist(
            right_min=0.5, prev_error=0.0, params=default_params)
        assert cmd.linear_x == pytest.approx(0.15)
        assert abs(cmd.angular_z) < 0.1

    def test_wall_too_close(self, default_params):
        """壁に近すぎる → angular_z > 0 (左へ離れる)"""
        cmd, new_error = compute_wall_follow_twist(
            right_min=0.3, prev_error=0.0, params=default_params)
        assert cmd.angular_z > 0

    def test_wall_too_far(self, default_params):
        """壁から遠すぎる → angular_z < 0 (右へ近づく)"""
        cmd, new_error = compute_wall_follow_twist(
            right_min=0.7, prev_error=0.0, params=default_params)
        assert cmd.angular_z < 0

    def test_angular_z_limit_positive(self, default_params):
        """極端に近い → angular_z_limit でクリップ"""
        cmd, _ = compute_wall_follow_twist(
            right_min=0.15, prev_error=0.0, params=default_params)
        assert cmd.angular_z <= default_params.angular_z_limit + 0.01

    def test_angular_z_limit_negative(self, default_params):
        """極端に遠い → -angular_z_limit でクリップ"""
        cmd, _ = compute_wall_follow_twist(
            right_min=2.0, prev_error=0.0, params=default_params)
        assert cmd.angular_z >= -default_params.angular_z_limit - 0.01

    def test_prev_error_propagated(self, default_params):
        """new_error が正しく返る"""
        _, new_error = compute_wall_follow_twist(
            right_min=0.3, prev_error=0.0, params=default_params)
        # error = wall_target(0.5) - right_min(0.3) = 0.2
        assert new_error == pytest.approx(0.2)

    def test_d_term_effect(self, default_params):
        """微分項: prev_error と current_error の差が angular_z に影響"""
        cmd1, _ = compute_wall_follow_twist(
            right_min=0.3, prev_error=0.0, params=default_params)
        cmd2, _ = compute_wall_follow_twist(
            right_min=0.3, prev_error=0.2, params=default_params)
        # prev_error=0.2 の場合 d_term=0 で、prev_error=0.0 の場合 d_term>0
        # → cmd1 の angular_z が cmd2 より大きいはず
        assert cmd1.angular_z > cmd2.angular_z


# ============================================================================
# TestComputeTwist
# ============================================================================

class TestComputeTwist:
    """compute_twist のテスト (純粋関数)"""

    def test_forward(self, default_params):
        """FORWARD → 前進、回転なし"""
        zones = make_zones(front=2.0)
        cmd, new_error = compute_twist(
            RobotState.FORWARD, zones, 0.0, default_params)
        assert cmd.linear_x == pytest.approx(0.2)
        assert cmd.angular_z == pytest.approx(0.0)

    def test_avoid_left(self, default_params):
        """AVOID_LEFT → 停止、左回転"""
        zones = make_zones(front=0.3, left=3.0, right=0.5)
        cmd, _ = compute_twist(
            RobotState.AVOID_LEFT, zones, 0.0, default_params)
        assert cmd.linear_x == pytest.approx(0.0)
        assert cmd.angular_z == pytest.approx(0.5)

    def test_avoid_right(self, default_params):
        """AVOID_RIGHT → 停止、右回転"""
        zones = make_zones(front=0.3, left=0.5, right=3.0)
        cmd, _ = compute_twist(
            RobotState.AVOID_RIGHT, zones, 0.0, default_params)
        assert cmd.linear_x == pytest.approx(0.0)
        assert cmd.angular_z == pytest.approx(-0.5)

    def test_u_turn(self, default_params):
        """U_TURN → 停止、高速回転"""
        zones = make_zones(front=0.3, left=0.3, right=0.3)
        cmd, _ = compute_twist(
            RobotState.U_TURN, zones, 0.0, default_params)
        assert cmd.linear_x == pytest.approx(0.0)
        assert cmd.angular_z == pytest.approx(1.0)

    def test_wall_follow_delegates(self, default_params):
        """WALL_FOLLOW → compute_wall_follow_twist に委譲"""
        zones = make_zones(front=2.0, right=0.5)
        cmd, new_error = compute_twist(
            RobotState.WALL_FOLLOW, zones, 0.0, default_params)
        assert cmd.linear_x == pytest.approx(0.15)
        # wall_target(0.5) - right_min(0.5) = 0.0 → ほぼゼロ
        assert abs(cmd.angular_z) < 0.1


# ============================================================================
# TestUTurn (統合的な遷移 + Twist 検証)
# ============================================================================

class TestUTurn:
    """U ターン関連の統合テスト"""

    def test_u_turn_highest_priority(self, default_params):
        """WALL_FOLLOW からでも三方塞がり → U_TURN"""
        zones = make_zones(front=0.3, left=0.3, right=0.3)
        new_state, _ = update_state(RobotState.WALL_FOLLOW, zones, default_params)
        assert new_state == RobotState.U_TURN

    def test_u_turn_to_forward_when_clear(self, default_params):
        """U_TURN 中に前方クリア → FORWARD"""
        zones = make_zones(front=2.0, left=2.0, right=2.0)
        new_state, _ = update_state(RobotState.U_TURN, zones, default_params)
        assert new_state == RobotState.FORWARD

    def test_not_u_turn_when_two_sides_blocked(self, default_params):
        """2方向のみ塞がり → U_TURN ではなく AVOID"""
        zones = make_zones(front=0.3, left=3.0, right=0.3)
        new_state, _ = update_state(RobotState.FORWARD, zones, default_params)
        assert new_state == RobotState.AVOID_LEFT

    def test_u_turn_twist_values(self, default_params):
        """U_TURN Twist: 停止 + u_turn_speed 回転"""
        zones = make_zones(front=0.3, left=0.3, right=0.3)
        cmd, _ = compute_twist(RobotState.U_TURN, zones, 0.0, default_params)
        assert cmd.linear_x == pytest.approx(0.0)
        assert cmd.angular_z == pytest.approx(1.0)

    def test_u_turn_stays_when_still_blocked(self, default_params):
        """U_TURN 中にまだ塞がっている → U_TURN 維持"""
        zones = make_zones(front=0.3, left=0.3, right=0.3)
        new_state, _ = update_state(RobotState.U_TURN, zones, default_params)
        assert new_state == RobotState.U_TURN
