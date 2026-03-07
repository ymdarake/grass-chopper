"""
============================================================================
DockingBehavior 純粋ロジック ユニットテスト
============================================================================
rclpy 不要。Mac ホスト上で pytest のみで実行可能。
============================================================================
"""

import math

import pytest

from grass_chopper.docking_behavior import (
    DockingAction,
    DockingParams,
    compute_approach_pose,
    compute_dock_alignment_twist,
    evaluate_docking_result,
    is_at_dock,
)


# ============================================================================
# TestDockingParams
# ============================================================================

class TestDockingParams:
    """DockingParams データクラステスト"""

    def test_default_values(self):
        """デフォルト値が正しい"""
        params = DockingParams()
        assert params.home_x == pytest.approx(0.0)
        assert params.home_y == pytest.approx(0.0)
        assert params.home_yaw == pytest.approx(0.0)
        assert params.approach_distance == pytest.approx(0.5)
        assert params.dock_tolerance == pytest.approx(0.1)

    def test_frozen(self):
        """frozen=True で変更不可"""
        params = DockingParams()
        with pytest.raises(AttributeError):
            params.home_x = 1.0


# ============================================================================
# TestComputeApproachPose
# ============================================================================

class TestComputeApproachPose:
    """compute_approach_pose: 接近ポーズ計算"""

    def test_approach_from_east(self):
        """home_yaw=0 (東向き) → 接近ポイントは西側"""
        params = DockingParams(home_x=0.0, home_y=0.0, home_yaw=0.0,
                               approach_distance=0.5)
        x, y, yaw = compute_approach_pose(params)
        # ドックの前方 (yaw=0=東) から接近 → 接近点は西側 (-0.5, 0)
        assert x == pytest.approx(-0.5)
        assert y == pytest.approx(0.0, abs=1e-6)
        # 接近方向はドックを向く (東向き)
        assert yaw == pytest.approx(0.0)

    def test_approach_from_north(self):
        """home_yaw=π/2 (北向き) → 接近ポイントは南側"""
        params = DockingParams(home_x=1.0, home_y=2.0,
                               home_yaw=math.pi / 2,
                               approach_distance=1.0)
        x, y, yaw = compute_approach_pose(params)
        assert x == pytest.approx(1.0, abs=1e-6)
        assert y == pytest.approx(1.0, abs=1e-6)
        assert yaw == pytest.approx(math.pi / 2, abs=1e-6)


# ============================================================================
# TestIsAtDock
# ============================================================================

class TestIsAtDock:
    """is_at_dock: ドック到達判定"""

    def test_at_dock(self):
        """ドック位置にいる → True"""
        params = DockingParams(home_x=0.0, home_y=0.0,
                               dock_tolerance=0.1)
        assert is_at_dock(0.05, 0.05, params) is True

    def test_not_at_dock(self):
        """ドック位置から離れている → False"""
        params = DockingParams(home_x=0.0, home_y=0.0,
                               dock_tolerance=0.1)
        assert is_at_dock(1.0, 1.0, params) is False


# ============================================================================
# TestComputeDockAlignmentTwist
# ============================================================================

class TestComputeDockAlignmentTwist:
    """compute_dock_alignment_twist: アライメント速度計算"""

    def test_aligned_gives_zero(self):
        """ドックに正対 → 速度ゼロ"""
        params = DockingParams(home_x=0.0, home_y=0.0, home_yaw=0.0)
        linear, angular = compute_dock_alignment_twist(
            current_x=0.0, current_y=0.0, current_yaw=0.0, params=params)
        assert linear == pytest.approx(0.0, abs=0.05)
        assert angular == pytest.approx(0.0, abs=0.05)

    def test_offset_gives_correction(self):
        """ドックからずれている → 補正速度"""
        params = DockingParams(home_x=0.0, home_y=0.0, home_yaw=0.0)
        linear, angular = compute_dock_alignment_twist(
            current_x=1.0, current_y=0.0, current_yaw=math.pi, params=params)
        # ドックに向かうため前進方向の速度が正
        assert linear != 0.0 or angular != 0.0


# ============================================================================
# TestEvaluateDockingResult
# ============================================================================

class TestEvaluateDockingResult:
    """evaluate_docking_result: ドッキング結果の評価"""

    def test_success(self):
        """ドッキング成功 → COMPLETE"""
        result = evaluate_docking_result(
            attempt_count=1, max_retries=3, succeeded=True)
        assert result == DockingAction.COMPLETE

    def test_retry_on_failure(self):
        """失敗でリトライ回数残り → RETRY"""
        result = evaluate_docking_result(
            attempt_count=1, max_retries=3, succeeded=False)
        assert result == DockingAction.RETRY

    def test_fallback_on_max_retries(self):
        """最大リトライ到達 → FALLBACK"""
        result = evaluate_docking_result(
            attempt_count=3, max_retries=3, succeeded=False)
        assert result == DockingAction.FALLBACK

    def test_first_attempt_success(self):
        """初回成功 → COMPLETE"""
        result = evaluate_docking_result(
            attempt_count=0, max_retries=3, succeeded=True)
        assert result == DockingAction.COMPLETE
