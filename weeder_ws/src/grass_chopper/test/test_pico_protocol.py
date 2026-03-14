"""
============================================================================
Pico 通信プロトコル・モーター制御 ユニットテスト
============================================================================
rclpy 不要。Mac ホスト上で pytest のみで実行可能。

Pi 5 ↔ Pico 間の UART 通信プロトコル:
  送信 (Pi → Pico): "V,{left_vel},{right_vel}\n"  [rad/s]
  受信 (Pico → Pi): "E,{left_enc},{right_enc},{left_vel},{right_vel}\n"  [ticks, rad/s]
============================================================================
"""

import math

import pytest

from grass_chopper.pico_protocol import (
    encode_velocity_command,
    decode_velocity_command,
    encode_encoder_feedback,
    decode_encoder_feedback,
    validate_checksum,
    append_checksum,
    PidController,
    ticks_to_radians,
    radians_to_ticks,
)


# ============================================================================
# TestEncodeVelocityCommand
# ============================================================================

class TestEncodeVelocityCommand:
    """Pi → Pico: 速度指令のエンコード"""

    def test_basic(self):
        """基本的な速度指令"""
        msg = encode_velocity_command(1.0, -1.0)
        assert msg == "V,1.000,-1.000\n"

    def test_zero(self):
        """停止指令"""
        msg = encode_velocity_command(0.0, 0.0)
        assert msg == "V,0.000,0.000\n"

    def test_precision(self):
        """小数点以下 3 桁"""
        msg = encode_velocity_command(1.23456, -0.789)
        assert msg == "V,1.235,-0.789\n"


# ============================================================================
# TestDecodeVelocityCommand
# ============================================================================

class TestDecodeVelocityCommand:
    """Pico 側: 受信した速度指令のデコード"""

    def test_basic(self):
        """基本的なデコード"""
        left, right = decode_velocity_command("V,1.000,-1.000\n")
        assert left == pytest.approx(1.0)
        assert right == pytest.approx(-1.0)

    def test_zero(self):
        """停止指令"""
        left, right = decode_velocity_command("V,0.000,0.000\n")
        assert left == pytest.approx(0.0)
        assert right == pytest.approx(0.0)

    def test_invalid_prefix(self):
        """不正なプレフィックス → None"""
        result = decode_velocity_command("X,1.0,2.0\n")
        assert result is None

    def test_invalid_format(self):
        """不正なフォーマット → None"""
        result = decode_velocity_command("V,abc\n")
        assert result is None

    def test_empty(self):
        """空文字列 → None"""
        result = decode_velocity_command("")
        assert result is None

    def test_missing_field(self):
        """フィールド不足 → None"""
        result = decode_velocity_command("V,1.0\n")
        assert result is None

    def test_with_checksum(self):
        """チェックサム付きメッセージをデコードできる"""
        msg = append_checksum("V,1.000,-1.000")
        left, right = decode_velocity_command(msg)
        assert left == pytest.approx(1.0)
        assert right == pytest.approx(-1.0)


# ============================================================================
# TestEncodeEncoderFeedback
# ============================================================================

class TestEncodeEncoderFeedback:
    """Pico → Pi: エンコーダフィードバックのエンコード"""

    def test_basic(self):
        """基本的なフィードバック"""
        msg = encode_encoder_feedback(1000, -500, 2.5, -1.2)
        assert msg == "E,1000,-500,2.500,-1.200\n"

    def test_zero(self):
        """停止状態"""
        msg = encode_encoder_feedback(0, 0, 0.0, 0.0)
        assert msg == "E,0,0,0.000,0.000\n"


# ============================================================================
# TestDecodeEncoderFeedback
# ============================================================================

class TestDecodeEncoderFeedback:
    """Pi 側: エンコーダフィードバックのデコード"""

    def test_basic(self):
        """基本的なデコード"""
        result = decode_encoder_feedback("E,1000,-500,2.500,-1.200\n")
        assert result is not None
        left_ticks, right_ticks, left_vel, right_vel = result
        assert left_ticks == 1000
        assert right_ticks == -500
        assert left_vel == pytest.approx(2.5)
        assert right_vel == pytest.approx(-1.2)

    def test_invalid_prefix(self):
        """不正なプレフィックス → None"""
        result = decode_encoder_feedback("X,1000,-500,2.5,-1.2\n")
        assert result is None

    def test_invalid_format(self):
        """不正なフォーマット → None"""
        result = decode_encoder_feedback("E,abc\n")
        assert result is None

    def test_empty(self):
        """空文字列 → None"""
        result = decode_encoder_feedback("")
        assert result is None

    def test_with_checksum(self):
        """チェックサム付きメッセージをデコードできる"""
        msg = append_checksum("E,1000,-500,2.500,-1.200")
        result = decode_encoder_feedback(msg)
        assert result is not None
        left_ticks, right_ticks, left_vel, right_vel = result
        assert left_ticks == 1000
        assert right_ticks == -500
        assert left_vel == pytest.approx(2.5)
        assert right_vel == pytest.approx(-1.2)


# ============================================================================
# TestChecksum
# ============================================================================

class TestChecksum:
    """チェックサム: 通信エラー検出"""

    def test_append_and_validate(self):
        """チェックサム付与 → 検証が通る"""
        msg = "V,1.000,-1.000"
        with_cs = append_checksum(msg)
        assert validate_checksum(with_cs) is True

    def test_corrupted(self):
        """データが壊れたら検証失敗"""
        msg = "V,1.000,-1.000"
        with_cs = append_checksum(msg)
        corrupted = "V,1.000,-2.000" + with_cs[with_cs.rfind("*"):]
        assert validate_checksum(corrupted) is False

    def test_no_checksum(self):
        """チェックサムなし → 失敗"""
        assert validate_checksum("V,1.000,-1.000") is False

    def test_format(self):
        """チェックサムは *XX (2桁16進数) 形式"""
        msg = "V,1.000,-1.000"
        with_cs = append_checksum(msg)
        assert "*" in with_cs
        cs_part = with_cs.split("*")[1]
        assert len(cs_part) == 2
        int(cs_part, 16)  # 16進数としてパースできる


# ============================================================================
# TestPidController
# ============================================================================

class TestPidController:
    """PID 速度制御"""

    def test_zero_error(self):
        """誤差ゼロ → 出力ゼロ"""
        pid = PidController(kp=1.0, ki=0.0, kd=0.0)
        output = pid.update(target=1.0, current=1.0, dt=0.001)
        assert output == pytest.approx(0.0)

    def test_proportional(self):
        """P 制御: 誤差に比例した出力"""
        pid = PidController(kp=2.0, ki=0.0, kd=0.0)
        output = pid.update(target=5.0, current=3.0, dt=0.001)
        assert output == pytest.approx(4.0)  # 2.0 * (5-3)

    def test_integral(self):
        """I 制御: 誤差の累積"""
        pid = PidController(kp=0.0, ki=1.0, kd=0.0)
        pid.update(target=1.0, current=0.0, dt=1.0)  # error=1.0, integral=1.0
        output = pid.update(target=1.0, current=0.0, dt=1.0)  # integral=2.0
        assert output == pytest.approx(2.0)

    def test_derivative(self):
        """D 制御: 誤差の変化率"""
        pid = PidController(kp=0.0, ki=0.0, kd=1.0)
        pid.update(target=1.0, current=0.0, dt=1.0)  # error=1.0
        output = pid.update(target=2.0, current=0.0, dt=1.0)  # error=2.0, d_error=1.0
        assert output == pytest.approx(1.0)

    def test_output_clamp(self):
        """出力がクランプされる"""
        pid = PidController(kp=100.0, ki=0.0, kd=0.0, output_min=-1.0, output_max=1.0)
        output = pid.update(target=10.0, current=0.0, dt=0.001)
        assert output == pytest.approx(1.0)

    def test_output_clamp_negative(self):
        """負方向のクランプ"""
        pid = PidController(kp=100.0, ki=0.0, kd=0.0, output_min=-1.0, output_max=1.0)
        output = pid.update(target=-10.0, current=0.0, dt=0.001)
        assert output == pytest.approx(-1.0)

    def test_integral_windup_prevention(self):
        """積分ワインドアップ防止"""
        pid = PidController(kp=0.0, ki=1.0, kd=0.0, output_min=-1.0, output_max=1.0)
        for _ in range(100):
            pid.update(target=10.0, current=0.0, dt=1.0)
        # 積分が無限に大きくならず、出力がクランプされる
        output = pid.update(target=0.0, current=0.0, dt=1.0)
        assert output <= 1.0

    def test_reset(self):
        """リセットで内部状態がクリアされる"""
        pid = PidController(kp=0.0, ki=1.0, kd=0.0)
        pid.update(target=5.0, current=0.0, dt=1.0)
        pid.reset()
        output = pid.update(target=1.0, current=0.0, dt=1.0)
        assert output == pytest.approx(1.0)  # 積分がリセットされている


# ============================================================================
# TestTicksConversion
# ============================================================================

class TestTicksConversion:
    """エンコーダ ticks ↔ ラジアン変換"""

    def test_full_rotation(self):
        """1 回転 = 2π rad"""
        ticks_per_rev = 1440  # 一般的な値
        rad = ticks_to_radians(1440, ticks_per_rev)
        assert rad == pytest.approx(2 * math.pi)

    def test_half_rotation(self):
        """半回転 = π rad"""
        rad = ticks_to_radians(720, 1440)
        assert rad == pytest.approx(math.pi)

    def test_zero(self):
        """0 ticks = 0 rad"""
        assert ticks_to_radians(0, 1440) == pytest.approx(0.0)

    def test_negative(self):
        """負の ticks = 負の rad"""
        rad = ticks_to_radians(-1440, 1440)
        assert rad == pytest.approx(-2 * math.pi)

    def test_radians_to_ticks(self):
        """ラジアン → ticks の逆変換"""
        ticks = radians_to_ticks(2 * math.pi, 1440)
        assert ticks == pytest.approx(1440)

    def test_roundtrip(self):
        """ticks → rad → ticks の往復変換"""
        original = 500
        rad = ticks_to_radians(original, 1440)
        back = radians_to_ticks(rad, 1440)
        assert back == pytest.approx(original)

    def test_zero_ticks_per_rev(self):
        """ticks_per_rev=0 → 0.0 (ZeroDivisionError を防止)"""
        assert ticks_to_radians(100, 0) == pytest.approx(0.0)
        assert radians_to_ticks(1.0, 0) == pytest.approx(0.0)
