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
    LowPassFilter,
    evaluate_incline_with_hysteresis,
    HysteresisState,
    calibrate_imu_offset,
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


# ============================================================================
# TestLowPassFilter
# ============================================================================

class TestLowPassFilter:
    """LowPassFilter: 高周波ノイズ除去"""

    def test_initial_value(self):
        """初期値は最初の入力をそのまま返す"""
        lpf = LowPassFilter(alpha=0.2)
        assert lpf.update(10.0) == pytest.approx(10.0)

    def test_smoothing(self):
        """ノイズの多い入力を平滑化する"""
        lpf = LowPassFilter(alpha=0.2)
        lpf.update(0.0)
        # 急激な変化を抑制
        result = lpf.update(10.0)
        assert result < 10.0  # 10.0 より小さい (平滑化される)
        assert result > 0.0   # 0.0 より大きい

    def test_convergence(self):
        """同じ値を入れ続けるとその値に収束する"""
        lpf = LowPassFilter(alpha=0.3)
        lpf.update(0.0)
        for _ in range(50):
            result = lpf.update(5.0)
        assert result == pytest.approx(5.0, abs=0.01)

    def test_alpha_1_no_filtering(self):
        """alpha=1.0 ではフィルタリングなし (入力がそのまま出る)"""
        lpf = LowPassFilter(alpha=1.0)
        lpf.update(0.0)
        assert lpf.update(10.0) == pytest.approx(10.0)

    def test_alpha_near_0_heavy_filtering(self):
        """alpha が小さいほど変化が遅い"""
        lpf = LowPassFilter(alpha=0.05)
        lpf.update(0.0)
        result = lpf.update(10.0)
        assert result < 1.0  # 非常に遅い追従

    def test_reset(self):
        """reset で初期状態に戻る"""
        lpf = LowPassFilter(alpha=0.3)
        lpf.update(100.0)
        lpf.update(100.0)
        lpf.reset()
        assert lpf.update(0.0) == pytest.approx(0.0)

    def test_alpha_zero_raises(self):
        """alpha=0 は ValueError"""
        with pytest.raises(ValueError):
            LowPassFilter(alpha=0.0)

    def test_alpha_negative_raises(self):
        """alpha < 0 は ValueError"""
        with pytest.raises(ValueError):
            LowPassFilter(alpha=-0.5)

    def test_alpha_over_1_raises(self):
        """alpha > 1 は ValueError"""
        with pytest.raises(ValueError):
            LowPassFilter(alpha=1.5)


# ============================================================================
# TestHysteresis
# ============================================================================

class TestHysteresis:
    """evaluate_incline_with_hysteresis: チャタリング防止"""

    def test_safe_to_warning(self):
        """SAFE → WARNING: 上昇閾値 (warning_deg) を超えたら遷移"""
        state = HysteresisState()
        level, state = evaluate_incline_with_hysteresis(
            roll_deg=0.0, pitch_deg=16.0,
            warning_deg=15.0, emergency_deg=25.0,
            hysteresis_deg=2.0, state=state)
        assert level == InclineLevel.WARNING

    def test_warning_to_safe_needs_hysteresis(self):
        """WARNING → SAFE: warning_deg - hysteresis_deg を下回る必要がある"""
        state = HysteresisState(current_level=InclineLevel.WARNING)
        # 14.0° は warning(15) - hysteresis(2) = 13° より上なのでまだ WARNING
        level, state = evaluate_incline_with_hysteresis(
            roll_deg=0.0, pitch_deg=14.0,
            warning_deg=15.0, emergency_deg=25.0,
            hysteresis_deg=2.0, state=state)
        assert level == InclineLevel.WARNING

    def test_warning_to_safe_below_hysteresis(self):
        """WARNING → SAFE: warning_deg - hysteresis_deg を下回ったら遷移"""
        state = HysteresisState(current_level=InclineLevel.WARNING)
        # 12.0° は warning(15) - hysteresis(2) = 13° より下なので SAFE
        level, state = evaluate_incline_with_hysteresis(
            roll_deg=0.0, pitch_deg=12.0,
            warning_deg=15.0, emergency_deg=25.0,
            hysteresis_deg=2.0, state=state)
        assert level == InclineLevel.SAFE

    def test_warning_to_emergency(self):
        """WARNING → EMERGENCY: emergency_deg を超えたら遷移"""
        state = HysteresisState(current_level=InclineLevel.WARNING)
        level, state = evaluate_incline_with_hysteresis(
            roll_deg=0.0, pitch_deg=26.0,
            warning_deg=15.0, emergency_deg=25.0,
            hysteresis_deg=2.0, state=state)
        assert level == InclineLevel.EMERGENCY

    def test_emergency_to_warning_needs_hysteresis(self):
        """EMERGENCY → WARNING: emergency_deg - hysteresis_deg を下回る必要"""
        state = HysteresisState(current_level=InclineLevel.EMERGENCY)
        # 24.0° は emergency(25) - hysteresis(2) = 23° より上なのでまだ EMERGENCY
        level, state = evaluate_incline_with_hysteresis(
            roll_deg=0.0, pitch_deg=24.0,
            warning_deg=15.0, emergency_deg=25.0,
            hysteresis_deg=2.0, state=state)
        assert level == InclineLevel.EMERGENCY

    def test_emergency_to_warning_below_hysteresis(self):
        """EMERGENCY → WARNING: emergency_deg - hysteresis_deg を下回ったら遷移"""
        state = HysteresisState(current_level=InclineLevel.EMERGENCY)
        # 22.0° は emergency(25) - hysteresis(2) = 23° より下なので WARNING
        level, state = evaluate_incline_with_hysteresis(
            roll_deg=0.0, pitch_deg=22.0,
            warning_deg=15.0, emergency_deg=25.0,
            hysteresis_deg=2.0, state=state)
        assert level == InclineLevel.WARNING

    def test_emergency_to_safe_directly(self):
        """EMERGENCY → SAFE: warning 未満なら WARNING を経由せず直接 SAFE"""
        state = HysteresisState(current_level=InclineLevel.EMERGENCY)
        level, state = evaluate_incline_with_hysteresis(
            roll_deg=0.0, pitch_deg=5.0,
            warning_deg=15.0, emergency_deg=25.0,
            hysteresis_deg=2.0, state=state)
        assert level == InclineLevel.SAFE

    def test_initial_state_is_safe(self):
        """初期状態は SAFE"""
        state = HysteresisState()
        assert state.current_level == InclineLevel.SAFE

    def test_uses_max_of_roll_and_pitch(self):
        """roll と pitch の大きい方で判定"""
        state = HysteresisState()
        level, _ = evaluate_incline_with_hysteresis(
            roll_deg=20.0, pitch_deg=5.0,
            warning_deg=15.0, emergency_deg=25.0,
            hysteresis_deg=2.0, state=state)
        assert level == InclineLevel.WARNING


# ============================================================================
# TestCalibrateImuOffset
# ============================================================================

class TestCalibrateImuOffset:
    """calibrate_imu_offset: IMU キャリブレーション"""

    def test_zero_samples(self):
        """サンプルがない場合は (0, 0) を返す"""
        offset_roll, offset_pitch = calibrate_imu_offset([])
        assert offset_roll == pytest.approx(0.0)
        assert offset_pitch == pytest.approx(0.0)

    def test_flat_surface(self):
        """水平面のサンプル → オフセットはほぼゼロ"""
        samples = [(0.0, 0.0)] * 10
        offset_roll, offset_pitch = calibrate_imu_offset(samples)
        assert offset_roll == pytest.approx(0.0, abs=1e-6)
        assert offset_pitch == pytest.approx(0.0, abs=1e-6)

    def test_tilted_mount(self):
        """傾いた取り付け → オフセットは平均値"""
        # roll=2°, pitch=3° で取り付けられている
        samples = [(2.0, 3.0)] * 10
        offset_roll, offset_pitch = calibrate_imu_offset(samples)
        assert offset_roll == pytest.approx(2.0)
        assert offset_pitch == pytest.approx(3.0)

    def test_noisy_samples(self):
        """ノイズありのサンプル → 平均でオフセットを推定"""
        # 真のオフセットは 1.0°, ノイズで ±0.5° 程度振れる
        samples = [(0.5, 0.5), (1.5, 1.5), (1.0, 1.0), (0.8, 0.8), (1.2, 1.2)]
        offset_roll, offset_pitch = calibrate_imu_offset(samples)
        assert offset_roll == pytest.approx(1.0, abs=0.01)
        assert offset_pitch == pytest.approx(1.0, abs=0.01)

    def test_single_sample(self):
        """サンプル 1 つでも動作する"""
        offset_roll, offset_pitch = calibrate_imu_offset([(5.0, -3.0)])
        assert offset_roll == pytest.approx(5.0)
        assert offset_pitch == pytest.approx(-3.0)
