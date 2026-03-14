"""
============================================================================
Pico 通信プロトコル・モーター制御 純粋ロジック
============================================================================
ROS 2 / MicroPython に依存しない純粋関数モジュール。
Mac ホスト上で pytest のみでテスト実行可能。

Pi 5 ↔ Pico 間の UART 通信プロトコル:
  速度指令 (Pi → Pico): "V,{left_vel},{right_vel}\n"    [rad/s]
  フィードバック (Pico → Pi): "E,{left_enc},{right_enc},{left_vel},{right_vel}\n"
                                                          [ticks, rad/s]
  チェックサム: NMEA 風 XOR チェックサム "*XX" (2 桁 16 進数)

Pico 側ファームウェア / Pi 側 hardware_interface の両方から使用される。
============================================================================
"""

import math


# ============================================================================
# 速度指令 (Pi → Pico)
# ============================================================================

def encode_velocity_command(left_vel: float, right_vel: float) -> str:
    """
    速度指令をエンコードする

    Args:
        left_vel: 左モーター目標速度 [rad/s]
        right_vel: 右モーター目標速度 [rad/s]

    Returns:
        "V,{left},{right}\n" 形式の文字列
    """
    return f"V,{left_vel:.3f},{right_vel:.3f}\n"


def decode_velocity_command(msg: str) -> tuple[float, float] | None:
    """
    速度指令をデコードする

    Args:
        msg: "V,{left},{right}\n" 形式の文字列

    Returns:
        (left_vel, right_vel) or None (パース失敗時)
    """
    try:
        msg = msg.strip()
        if not msg.startswith("V,"):
            return None
        parts = msg[2:].split(",")
        if len(parts) != 2:
            return None
        return float(parts[0]), float(parts[1])
    except (ValueError, IndexError):
        return None


# ============================================================================
# エンコーダフィードバック (Pico → Pi)
# ============================================================================

def encode_encoder_feedback(
    left_ticks: int, right_ticks: int,
    left_vel: float, right_vel: float,
) -> str:
    """
    エンコーダフィードバックをエンコードする

    Args:
        left_ticks: 左エンコーダ累積 ticks
        right_ticks: 右エンコーダ累積 ticks
        left_vel: 左モーター現在速度 [rad/s]
        right_vel: 右モーター現在速度 [rad/s]

    Returns:
        "E,{lt},{rt},{lv},{rv}\n" 形式の文字列
    """
    return f"E,{left_ticks},{right_ticks},{left_vel:.3f},{right_vel:.3f}\n"


def decode_encoder_feedback(
    msg: str,
) -> tuple[int, int, float, float] | None:
    """
    エンコーダフィードバックをデコードする

    Args:
        msg: "E,{lt},{rt},{lv},{rv}\n" 形式の文字列

    Returns:
        (left_ticks, right_ticks, left_vel, right_vel) or None
    """
    try:
        msg = msg.strip()
        if not msg.startswith("E,"):
            return None
        parts = msg[2:].split(",")
        if len(parts) != 4:
            return None
        return int(parts[0]), int(parts[1]), float(parts[2]), float(parts[3])
    except (ValueError, IndexError):
        return None


# ============================================================================
# チェックサム (NMEA 風 XOR)
# ============================================================================

def _compute_xor_checksum(data: str) -> str:
    """XOR チェックサムを 2 桁 16 進数で返す"""
    cs = 0
    for ch in data:
        cs ^= ord(ch)
    return f"{cs:02X}"


def append_checksum(msg: str) -> str:
    """
    メッセージにチェックサムを付与する

    Args:
        msg: "V,1.000,-1.000" (改行なし)

    Returns:
        "V,1.000,-1.000*A3" 形式の文字列
    """
    cs = _compute_xor_checksum(msg)
    return f"{msg}*{cs}"


def validate_checksum(msg: str) -> bool:
    """
    チェックサム付きメッセージを検証する

    Args:
        msg: "V,1.000,-1.000*A3" 形式の文字列

    Returns:
        True (正常) / False (不正)
    """
    if "*" not in msg:
        return False
    data, cs_received = msg.rsplit("*", 1)
    cs_computed = _compute_xor_checksum(data)
    return cs_received.upper() == cs_computed.upper()


# ============================================================================
# PID 速度制御
# ============================================================================

class PidController:
    """
    PID コントローラー (積分ワインドアップ防止付き)

    Args:
        kp: 比例ゲイン
        ki: 積分ゲイン
        kd: 微分ゲイン
        output_min: 出力下限 (None で無制限)
        output_max: 出力上限 (None で無制限)
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_min: float | None = None,
        output_max: float | None = None,
    ):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._output_min = output_min
        self._output_max = output_max
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True

    def update(self, target: float, current: float, dt: float) -> float:
        """
        PID 制御ループ 1 ステップ

        Args:
            target: 目標値 [rad/s]
            current: 現在値 [rad/s]
            dt: ステップ時間 [s]

        Returns:
            制御出力 (PWM デューティ比等に使う)
        """
        error = target - current

        # 積分
        self._integral += error * dt

        # 微分
        if self._first:
            d_error = 0.0
            self._first = False
        else:
            d_error = (error - self._prev_error) / dt if dt > 0 else 0.0

        self._prev_error = error

        # PID 出力
        output = self._kp * error + self._ki * self._integral + self._kd * d_error

        # 出力クランプ + 積分ワインドアップ防止
        if self._output_max is not None and output > self._output_max:
            output = self._output_max
            # ワインドアップ防止: 出力が飽和していたら積分を巻き戻す
            self._integral -= error * dt
        elif self._output_min is not None and output < self._output_min:
            output = self._output_min
            self._integral -= error * dt

        return output

    def reset(self):
        """内部状態をクリアする"""
        self._integral = 0.0
        self._prev_error = 0.0
        self._first = True


# ============================================================================
# エンコーダ ticks ↔ ラジアン変換
# ============================================================================

def ticks_to_radians(ticks: int, ticks_per_rev: int) -> float:
    """
    エンコーダ ticks をラジアンに変換する

    Args:
        ticks: エンコーダ累積 ticks
        ticks_per_rev: 1 回転あたりの ticks 数

    Returns:
        ラジアン
    """
    return (ticks / ticks_per_rev) * 2.0 * math.pi


def radians_to_ticks(radians: float, ticks_per_rev: int) -> float:
    """
    ラジアンをエンコーダ ticks に変換する

    Args:
        radians: ラジアン
        ticks_per_rev: 1 回転あたりの ticks 数

    Returns:
        ticks (float)
    """
    return (radians / (2.0 * math.pi)) * ticks_per_rev
