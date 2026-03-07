"""
============================================================================
MissionBehaviors 純粋ロジック ユニットテスト
============================================================================
rclpy 不要。Mac ホスト上で pytest のみで実行可能。
py_trees にも依存しない純粋関数のみをテスト。
============================================================================
"""

import math

import pytest

from grass_chopper.mission_behaviors import (
    MissionState,
    compute_home_pose,
    should_continue_charging,
    should_return_home,
    should_start_coverage,
)


# ============================================================================
# TestMissionState
# ============================================================================

class TestMissionState:
    """MissionState データクラステスト"""

    def test_creation(self):
        """正常に生成できる"""
        state = MissionState(
            battery_percentage=0.8,
            is_charging=False,
            coverage_ratio=0.5,
            target_coverage=0.9,
            is_coverage_running=False,
        )
        assert state.battery_percentage == pytest.approx(0.8)
        assert state.is_charging is False
        assert state.coverage_ratio == pytest.approx(0.5)

    def test_frozen(self):
        """frozen=True で変更不可"""
        state = MissionState(
            battery_percentage=0.8, is_charging=False,
            coverage_ratio=0.5, target_coverage=0.9,
            is_coverage_running=False,
        )
        with pytest.raises(AttributeError):
            state.battery_percentage = 0.5


# ============================================================================
# TestShouldReturnHome
# ============================================================================

class TestShouldReturnHome:
    """should_return_home: 帰還判定"""

    def test_low_battery_returns_true(self):
        """バッテリー低 + 未充電 → 帰還"""
        state = MissionState(
            battery_percentage=0.15,
            is_charging=False,
            coverage_ratio=0.5,
            target_coverage=0.9,
            is_coverage_running=True,
        )
        assert should_return_home(state, low_threshold=0.2) is True

    def test_high_battery_returns_false(self):
        """バッテリー十分 → 帰還不要"""
        state = MissionState(
            battery_percentage=0.8,
            is_charging=False,
            coverage_ratio=0.5,
            target_coverage=0.9,
            is_coverage_running=True,
        )
        assert should_return_home(state, low_threshold=0.2) is False

    def test_already_charging_returns_false(self):
        """充電中 → 帰還不要"""
        state = MissionState(
            battery_percentage=0.1,
            is_charging=True,
            coverage_ratio=0.5,
            target_coverage=0.9,
            is_coverage_running=False,
        )
        assert should_return_home(state, low_threshold=0.2) is False


# ============================================================================
# TestShouldStartCoverage
# ============================================================================

class TestShouldStartCoverage:
    """should_start_coverage: カバレッジ開始判定"""

    def test_battery_ok_and_uncovered_returns_true(self):
        """バッテリー十分 + 未完了 + 非実行中 → 開始"""
        state = MissionState(
            battery_percentage=0.8,
            is_charging=False,
            coverage_ratio=0.5,
            target_coverage=0.9,
            is_coverage_running=False,
        )
        assert should_start_coverage(state) is True

    def test_already_running_returns_false(self):
        """既に走行中 → 開始不要"""
        state = MissionState(
            battery_percentage=0.8,
            is_charging=False,
            coverage_ratio=0.5,
            target_coverage=0.9,
            is_coverage_running=True,
        )
        assert should_start_coverage(state) is False

    def test_coverage_complete_returns_false(self):
        """カバレッジ完了 → 開始不要"""
        state = MissionState(
            battery_percentage=0.8,
            is_charging=False,
            coverage_ratio=0.95,
            target_coverage=0.9,
            is_coverage_running=False,
        )
        assert should_start_coverage(state) is False

    def test_charging_returns_false(self):
        """充電中 → 開始不要"""
        state = MissionState(
            battery_percentage=0.8,
            is_charging=True,
            coverage_ratio=0.5,
            target_coverage=0.9,
            is_coverage_running=False,
        )
        assert should_start_coverage(state) is False


# ============================================================================
# TestShouldContinueCharging
# ============================================================================

class TestShouldContinueCharging:
    """should_continue_charging: 充電継続判定"""

    def test_not_full_returns_true(self):
        """満充電でない → 充電継続"""
        state = MissionState(
            battery_percentage=0.5,
            is_charging=True,
            coverage_ratio=0.0,
            target_coverage=0.9,
            is_coverage_running=False,
        )
        assert should_continue_charging(state) is True

    def test_full_returns_false(self):
        """満充電 → 充電完了"""
        state = MissionState(
            battery_percentage=1.0,
            is_charging=True,
            coverage_ratio=0.0,
            target_coverage=0.9,
            is_coverage_running=False,
        )
        assert should_continue_charging(state) is False

    def test_not_charging_returns_false(self):
        """充電中でない → False"""
        state = MissionState(
            battery_percentage=0.5,
            is_charging=False,
            coverage_ratio=0.0,
            target_coverage=0.9,
            is_coverage_running=False,
        )
        assert should_continue_charging(state) is False


# ============================================================================
# TestComputeHomePose
# ============================================================================

class TestComputeHomePose:
    """compute_home_pose: ホームポーズ計算"""

    def test_basic_computation(self):
        """ホーム座標が正しく返る"""
        x, y, yaw = compute_home_pose(
            current_x=5.0, current_y=3.0,
            home_x=0.0, home_y=0.0,
        )
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(0.0)
        # yaw はホーム方向を向く
        expected_yaw = math.atan2(0.0 - 3.0, 0.0 - 5.0)
        assert yaw == pytest.approx(expected_yaw)

    def test_same_position(self):
        """現在位置 = ホーム位置 → yaw=0"""
        x, y, yaw = compute_home_pose(
            current_x=0.0, current_y=0.0,
            home_x=0.0, home_y=0.0,
        )
        assert x == pytest.approx(0.0)
        assert y == pytest.approx(0.0)
        assert yaw == pytest.approx(0.0)
