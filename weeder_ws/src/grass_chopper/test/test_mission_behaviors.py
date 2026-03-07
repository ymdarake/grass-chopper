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


# ============================================================================
# TestCoverageProgress
# ============================================================================

class TestCoverageProgress:
    """CoverageProgress データクラステスト"""

    def test_creation(self):
        """正常に生成できる"""
        progress = CoverageProgress(
            total_waypoints=10,
            completed_index=-1,
        )
        assert progress.total_waypoints == 10
        assert progress.completed_index == -1

    def test_frozen(self):
        """frozen=True で変更不可"""
        progress = CoverageProgress(total_waypoints=10, completed_index=-1)
        with pytest.raises(AttributeError):
            progress.total_waypoints = 5


class TestUpdateProgress:
    """update_progress: 進捗更新"""

    def test_basic_update(self):
        """インデックスが正しく更新される"""
        progress = CoverageProgress(total_waypoints=10, completed_index=-1)
        updated = update_progress(progress, completed_index=3)
        assert updated.completed_index == 3
        assert updated.total_waypoints == 10

    def test_update_preserves_total(self):
        """total_waypoints を省略すると元の値が維持される"""
        progress = CoverageProgress(total_waypoints=20, completed_index=5)
        updated = update_progress(progress, completed_index=10)
        assert updated.total_waypoints == 20
        assert updated.completed_index == 10

    def test_update_with_total_waypoints(self):
        """total_waypoints も同時に更新できる"""
        progress = CoverageProgress(total_waypoints=0, completed_index=-1)
        updated = update_progress(progress, completed_index=3, total_waypoints=10)
        assert updated.total_waypoints == 10
        assert updated.completed_index == 3

    def test_update_total_waypoints_overrides(self):
        """total_waypoints を指定すると上書きされる"""
        progress = CoverageProgress(total_waypoints=5, completed_index=2)
        updated = update_progress(progress, completed_index=2, total_waypoints=15)
        assert updated.total_waypoints == 15


class TestGetResumeIndex:
    """get_resume_index: 再開インデックスの取得"""

    def test_resume_from_middle(self):
        """中断地点の手前 (completed_index) から再開"""
        progress = CoverageProgress(total_waypoints=10, completed_index=4)
        # completed_index=4 → ウェイポイント0~4完了 → 5から再開
        assert get_resume_index(progress) == 5

    def test_resume_from_beginning(self):
        """未開始 (completed_index=-1) → 0から開始"""
        progress = CoverageProgress(total_waypoints=10, completed_index=-1)
        assert get_resume_index(progress) == 0

    def test_all_complete_returns_zero(self):
        """全完了 → 0 (最初からやり直し)"""
        progress = CoverageProgress(total_waypoints=10, completed_index=9)
        assert get_resume_index(progress) == 0


class TestShouldResumeCoverage:
    """should_resume_coverage: 再開すべきかの判定"""

    def test_has_remaining_returns_true(self):
        """中断して残ウェイポイントあり → 再開する"""
        progress = CoverageProgress(total_waypoints=10, completed_index=4)
        assert should_resume_coverage(progress) is True

    def test_all_complete_returns_false(self):
        """全完了 → 再開しない (新しいサイクル)"""
        progress = CoverageProgress(total_waypoints=10, completed_index=9)
        assert should_resume_coverage(progress) is False

    def test_not_started_returns_false(self):
        """未開始 → 再開ではなく新規開始"""
        progress = CoverageProgress(total_waypoints=10, completed_index=-1)
        assert should_resume_coverage(progress) is False

    def test_zero_waypoints_returns_false(self):
        """ウェイポイントなし → 再開しない"""
        progress = CoverageProgress(total_waypoints=0, completed_index=-1)
        assert should_resume_coverage(progress) is False
