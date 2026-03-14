"""
============================================================================
傾斜検知 純粋ロジック ユニットテスト
============================================================================
rclpy 不要。Mac ホスト上で pytest のみで実行可能。
============================================================================
"""

import math

import pytest

from grass_chopper.incline_monitor import (
    InclineLevel,
    quaternion_to_rpy,
    evaluate_incline,
    should_stop,
)


# ============================================================================
# TestQuaternionToRpy
# ============================================================================

class TestQuaternionToRpy:
    """quaternion_to_rpy: 四元数 → ロール・ピッチ・ヨー変換"""

    def test_identity_quaternion(self):
        """単位四元数 → (0, 0, 0)"""
        roll, pitch, yaw = quaternion_to_rpy(0.0, 0.0, 0.0, 1.0)
        assert roll == pytest.approx(0.0, abs=1e-6)
        assert pitch == pytest.approx(0.0, abs=1e-6)
        assert yaw == pytest.approx(0.0, abs=1e-6)

    def test_pitch_15_degrees(self):
        """ピッチ 15° の四元数"""
        angle = math.radians(15.0)
        # Y 軸回転の四元数: (0, sin(θ/2), 0, cos(θ/2))
        qy = math.sin(angle / 2)
        qw = math.cos(angle / 2)
        roll, pitch, yaw = quaternion_to_rpy(0.0, qy, 0.0, qw)
        assert roll == pytest.approx(0.0, abs=1e-4)
        assert pitch == pytest.approx(angle, abs=1e-4)
        assert yaw == pytest.approx(0.0, abs=1e-4)

    def test_roll_30_degrees(self):
        """ロール 30° の四元数"""
        angle = math.radians(30.0)
        # X 軸回転の四元数: (sin(θ/2), 0, 0, cos(θ/2))
        qx = math.sin(angle / 2)
        qw = math.cos(angle / 2)
        roll, pitch, yaw = quaternion_to_rpy(qx, 0.0, 0.0, qw)
        assert roll == pytest.approx(angle, abs=1e-4)
        assert pitch == pytest.approx(0.0, abs=1e-4)

    def test_yaw_90_degrees(self):
        """ヨー 90° の四元数"""
        angle = math.radians(90.0)
        # Z 軸回転の四元数: (0, 0, sin(θ/2), cos(θ/2))
        qz = math.sin(angle / 2)
        qw = math.cos(angle / 2)
        roll, pitch, yaw = quaternion_to_rpy(0.0, 0.0, qz, qw)
        assert yaw == pytest.approx(angle, abs=1e-4)


# ============================================================================
# TestEvaluateIncline
# ============================================================================

class TestEvaluateIncline:
    """evaluate_incline: 傾斜レベル判定"""

    def test_flat_surface_is_safe(self):
        """平坦な面 → SAFE"""
        level = evaluate_incline(
            roll=0.0, pitch=0.0,
            warning_deg=15.0, emergency_deg=25.0)
        assert level == InclineLevel.SAFE

    def test_small_tilt_is_safe(self):
        """小さな傾き (5°) → SAFE"""
        level = evaluate_incline(
            roll=math.radians(3.0), pitch=math.radians(4.0),
            warning_deg=15.0, emergency_deg=25.0)
        assert level == InclineLevel.SAFE

    def test_warning_pitch(self):
        """ピッチ 18° → WARNING"""
        level = evaluate_incline(
            roll=0.0, pitch=math.radians(18.0),
            warning_deg=15.0, emergency_deg=25.0)
        assert level == InclineLevel.WARNING

    def test_warning_roll(self):
        """ロール 16° → WARNING"""
        level = evaluate_incline(
            roll=math.radians(16.0), pitch=0.0,
            warning_deg=15.0, emergency_deg=25.0)
        assert level == InclineLevel.WARNING

    def test_emergency_pitch(self):
        """ピッチ 30° → EMERGENCY"""
        level = evaluate_incline(
            roll=0.0, pitch=math.radians(30.0),
            warning_deg=15.0, emergency_deg=25.0)
        assert level == InclineLevel.EMERGENCY

    def test_emergency_roll(self):
        """ロール 26° → EMERGENCY"""
        level = evaluate_incline(
            roll=math.radians(26.0), pitch=0.0,
            warning_deg=15.0, emergency_deg=25.0)
        assert level == InclineLevel.EMERGENCY

    def test_negative_angles(self):
        """負の角度でも絶対値で判定"""
        level = evaluate_incline(
            roll=math.radians(-20.0), pitch=0.0,
            warning_deg=15.0, emergency_deg=25.0)
        assert level == InclineLevel.WARNING

    def test_boundary_warning(self):
        """閾値をわずかに超える → WARNING"""
        level = evaluate_incline(
            roll=0.0, pitch=math.radians(15.1),
            warning_deg=15.0, emergency_deg=25.0)
        assert level == InclineLevel.WARNING

    def test_boundary_just_below_warning(self):
        """閾値をわずかに下回る → SAFE"""
        level = evaluate_incline(
            roll=0.0, pitch=math.radians(14.9),
            warning_deg=15.0, emergency_deg=25.0)
        assert level == InclineLevel.SAFE

    def test_boundary_emergency(self):
        """緊急閾値をわずかに超える → EMERGENCY"""
        level = evaluate_incline(
            roll=0.0, pitch=math.radians(25.1),
            warning_deg=15.0, emergency_deg=25.0)
        assert level == InclineLevel.EMERGENCY


# ============================================================================
# TestShouldStop
# ============================================================================

class TestShouldStop:
    """should_stop: 緊急停止判定"""

    def test_safe_no_stop(self):
        """SAFE → 停止しない"""
        assert should_stop(InclineLevel.SAFE) is False

    def test_warning_no_stop(self):
        """WARNING → 停止しない (減速のみ)"""
        assert should_stop(InclineLevel.WARNING) is False

    def test_emergency_stop(self):
        """EMERGENCY → 停止する"""
        assert should_stop(InclineLevel.EMERGENCY) is True
