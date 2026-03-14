"""
============================================================================
シリアルブリッジ 純粋ロジック ユニットテスト
============================================================================
rclpy 不要。Mac ホスト上で pytest のみで実行可能。

Pi 5 ↔ Pico 間の通信状態管理・オドメトリ計算をテストする。
============================================================================
"""

import math

import pytest

from grass_chopper.serial_bridge import (
    BridgeState,
    compute_odometry_delta,
    WatchdogTimer,
    OdometryAccumulator,
)


# ============================================================================
# TestBridgeState
# ============================================================================

class TestBridgeState:
    """BridgeState: 通信状態管理"""

    def test_initial_state(self):
        """初期状態は DISCONNECTED"""
        state = BridgeState()
        assert state.is_connected is False
        assert state.last_recv_time is None

    def test_on_receive(self):
        """データ受信で CONNECTED に遷移"""
        state = BridgeState()
        state.on_receive(timestamp=1.0)
        assert state.is_connected is True
        assert state.last_recv_time == 1.0

    def test_on_receive_updates_time(self):
        """連続受信でタイムスタンプが更新される"""
        state = BridgeState()
        state.on_receive(timestamp=1.0)
        state.on_receive(timestamp=2.5)
        assert state.last_recv_time == 2.5

    def test_on_timeout(self):
        """タイムアウトで DISCONNECTED に遷移"""
        state = BridgeState()
        state.on_receive(timestamp=1.0)
        state.on_timeout()
        assert state.is_connected is False


# ============================================================================
# TestWatchdogTimer
# ============================================================================

class TestWatchdogTimer:
    """WatchdogTimer: 通信途絶検出"""

    def test_timeout_before_first_feed(self):
        """一度も feed されていない → タイムアウト (安全側)"""
        wd = WatchdogTimer(timeout_sec=0.5)
        assert wd.is_timed_out(current_time=0.0) is True

    def test_no_timeout_within_period(self):
        """期間内ならタイムアウトしない"""
        wd = WatchdogTimer(timeout_sec=0.5)
        wd.feed(timestamp=1.0)
        assert wd.is_timed_out(current_time=1.3) is False

    def test_timeout_after_period(self):
        """期間を超えたらタイムアウト"""
        wd = WatchdogTimer(timeout_sec=0.5)
        wd.feed(timestamp=1.0)
        assert wd.is_timed_out(current_time=1.6) is True

    def test_feed_resets(self):
        """feed でタイマーがリセットされる"""
        wd = WatchdogTimer(timeout_sec=0.5)
        wd.feed(timestamp=1.0)
        wd.feed(timestamp=1.4)  # リセット
        assert wd.is_timed_out(current_time=1.8) is False

    def test_exact_boundary(self):
        """ちょうど境界値 → タイムアウトしない"""
        wd = WatchdogTimer(timeout_sec=0.5)
        wd.feed(timestamp=1.0)
        assert wd.is_timed_out(current_time=1.5) is False


# ============================================================================
# TestComputeOdometryDelta
# ============================================================================

class TestComputeOdometryDelta:
    """compute_odometry_delta: エンコーダ差分からオドメトリ増分を計算"""

    def test_straight_forward(self):
        """直進: 左右同速度"""
        dx, dy, dtheta = compute_odometry_delta(
            left_vel=1.0, right_vel=1.0,
            wheel_separation=0.24, dt=1.0)
        assert dx > 0  # 前進
        assert dy == pytest.approx(0.0, abs=1e-6)
        assert dtheta == pytest.approx(0.0, abs=1e-6)

    def test_turn_in_place(self):
        """その場旋回: 左右逆速度"""
        dx, dy, dtheta = compute_odometry_delta(
            left_vel=-1.0, right_vel=1.0,
            wheel_separation=0.24, dt=1.0)
        assert abs(dx) < 0.01  # ほぼ移動しない
        assert dtheta > 0  # 左旋回 (反時計回り)

    def test_zero_velocity(self):
        """停止"""
        dx, dy, dtheta = compute_odometry_delta(
            left_vel=0.0, right_vel=0.0,
            wheel_separation=0.24, dt=1.0)
        assert dx == pytest.approx(0.0)
        assert dy == pytest.approx(0.0)
        assert dtheta == pytest.approx(0.0)

    def test_zero_dt(self):
        """dt=0 でもクラッシュしない"""
        dx, dy, dtheta = compute_odometry_delta(
            left_vel=1.0, right_vel=1.0,
            wheel_separation=0.24, dt=0.0)
        assert dx == pytest.approx(0.0)
        assert dy == pytest.approx(0.0)
        assert dtheta == pytest.approx(0.0)

    def test_curve(self):
        """カーブ: 右の速度が大きい → 左に曲がる"""
        dx, dy, dtheta = compute_odometry_delta(
            left_vel=0.5, right_vel=1.0,
            wheel_separation=0.24, dt=0.1)
        assert dx > 0  # 前進要素あり
        assert dtheta > 0  # 左旋回


# ============================================================================
# TestOdometryAccumulator
# ============================================================================

class TestOdometryAccumulator:
    """OdometryAccumulator: オドメトリの累積計算"""

    def test_initial_pose(self):
        """初期位置は原点"""
        odom = OdometryAccumulator(wheel_separation=0.24)
        assert odom.x == pytest.approx(0.0)
        assert odom.y == pytest.approx(0.0)
        assert odom.theta == pytest.approx(0.0)

    def test_straight_forward(self):
        """直進で X が増加"""
        odom = OdometryAccumulator(wheel_separation=0.24)
        # left_vel/right_vel は車輪の線速度 [m/s]
        odom.update(left_vel=1.0, right_vel=1.0, dt=1.0)
        assert odom.x > 0
        assert odom.y == pytest.approx(0.0, abs=1e-6)

    def test_accumulation(self):
        """複数回の更新が累積される"""
        odom = OdometryAccumulator(wheel_separation=0.24)
        odom.update(left_vel=1.0, right_vel=1.0, dt=0.1)
        x1 = odom.x
        odom.update(left_vel=1.0, right_vel=1.0, dt=0.1)
        assert odom.x > x1

    def test_reset(self):
        """リセットで原点に戻る"""
        odom = OdometryAccumulator(wheel_separation=0.24)
        odom.update(left_vel=1.0, right_vel=1.0, dt=1.0)
        odom.reset()
        assert odom.x == pytest.approx(0.0)
        assert odom.y == pytest.approx(0.0)
        assert odom.theta == pytest.approx(0.0)

    def test_90_degree_turn(self):
        """約 90° 旋回"""
        odom = OdometryAccumulator(wheel_separation=0.24)
        # 左右逆 → その場旋回。角速度 = (right - left) / separation
        # 旋回角速度 = (1.0 - (-1.0)) / 0.24 = 8.33 rad/s
        # 90° = π/2 ≈ 1.57 rad → dt = 1.57 / 8.33 ≈ 0.188 s
        dt = (math.pi / 2) / (2.0 / 0.24)
        odom.update(left_vel=-1.0, right_vel=1.0, dt=dt)
        assert odom.theta == pytest.approx(math.pi / 2, abs=0.01)

    def test_theta_normalized(self):
        """theta は [-π, π] に正規化される"""
        odom = OdometryAccumulator(wheel_separation=0.24)
        # 大きな回転を累積 → 正規化されるか確認
        for _ in range(100):
            odom.update(left_vel=-1.0, right_vel=1.0, dt=0.1)
        assert -math.pi <= odom.theta <= math.pi


# ============================================================================
# TestComputeOdometryDeltaEdgeCases
# ============================================================================

class TestComputeOdometryDeltaEdgeCases:
    """compute_odometry_delta: エッジケース"""

    def test_zero_wheel_separation(self):
        """wheel_separation=0 でゼロ除算しない"""
        dx, dy, dtheta = compute_odometry_delta(
            left_vel=1.0, right_vel=1.0,
            wheel_separation=0.0, dt=1.0)
        assert dx == pytest.approx(0.0)
        assert dy == pytest.approx(0.0)
        assert dtheta == pytest.approx(0.0)
