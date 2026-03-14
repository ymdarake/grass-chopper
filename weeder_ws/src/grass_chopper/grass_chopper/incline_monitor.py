"""
============================================================================
傾斜検知 純粋計算ロジック (incline_monitor)
============================================================================
ROS 2 に依存しない純粋関数モジュール。
Mac ホスト上で pytest のみでテスト実行可能。

incline_monitor_node.py (ROS 2 アダプター) から使用される。
============================================================================
"""

import math
from enum import Enum


class InclineLevel(Enum):
    """傾斜レベル"""
    SAFE = "safe"           # 安全 (平坦)
    WARNING = "warning"     # 警告 (減速推奨)
    EMERGENCY = "emergency"  # 緊急 (即時停止)


def quaternion_to_rpy(
    x: float, y: float, z: float, w: float,
) -> tuple[float, float, float]:
    """
    四元数からロール・ピッチ・ヨー (RPY) を計算する

    Args:
        x, y, z, w: 四元数の各成分

    Returns:
        (roll, pitch, yaw): ラジアン単位の RPY
    """
    # Roll (X 軸回転)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y 軸回転) — ジンバルロック対策
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (Z 軸回転)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def evaluate_incline(
    roll: float,
    pitch: float,
    warning_deg: float = 15.0,
    emergency_deg: float = 25.0,
) -> InclineLevel:
    """
    ロール・ピッチ角から傾斜レベルを判定する

    Args:
        roll: ロール角 [rad]
        pitch: ピッチ角 [rad]
        warning_deg: 警告閾値 [度]
        emergency_deg: 緊急停止閾値 [度]

    Returns:
        InclineLevel: 傾斜レベル
    """
    roll_deg = abs(math.degrees(roll))
    pitch_deg = abs(math.degrees(pitch))
    max_deg = max(roll_deg, pitch_deg)

    if max_deg >= emergency_deg:
        return InclineLevel.EMERGENCY
    elif max_deg >= warning_deg:
        return InclineLevel.WARNING
    else:
        return InclineLevel.SAFE


class LowPassFilter:
    """
    一次ローパスフィルタ

    高周波ノイズを除去する。alpha が大きいほど追従が速い (フィルタが弱い)。
    alpha=1.0 でフィルタなし、alpha→0 で非常に強いフィルタ。

    Args:
        alpha: フィルタ係数 (0 < alpha ≤ 1)
    """

    def __init__(self, alpha: float = 0.2):
        if not (0.0 < alpha <= 1.0):
            raise ValueError(f"alpha must be in the range (0, 1], got {alpha}")
        self._alpha = alpha
        self._value: float | None = None

    def update(self, raw: float) -> float:
        """新しいサンプルを入力し、フィルタ済み値を返す"""
        if self._value is None:
            self._value = raw
        else:
            self._value = self._alpha * raw + (1.0 - self._alpha) * self._value
        return self._value

    def reset(self):
        """フィルタを初期状態に戻す"""
        self._value = None


class HysteresisState:
    """ヒステリシス判定の状態を保持するクラス"""

    def __init__(self, current_level: InclineLevel = InclineLevel.SAFE):
        self.current_level = current_level


def evaluate_incline_with_hysteresis(
    roll_deg: float,
    pitch_deg: float,
    warning_deg: float = 15.0,
    emergency_deg: float = 25.0,
    hysteresis_deg: float = 2.0,
    state: HysteresisState | None = None,
) -> tuple[InclineLevel, HysteresisState]:
    """
    ヒステリシス付き傾斜レベル判定

    レベルを上げる (SAFE→WARNING, WARNING→EMERGENCY) には上昇閾値を超える必要があり、
    レベルを下げるには (上昇閾値 - hysteresis_deg) を下回る必要がある。
    これにより閾値付近でのチャタリング (高速な状態切り替え) を防止する。

    Args:
        roll_deg: ロール角の絶対値 [度]
        pitch_deg: ピッチ角の絶対値 [度]
        warning_deg: 警告閾値 [度]
        emergency_deg: 緊急停止閾値 [度]
        hysteresis_deg: ヒステリシス幅 [度]
        state: 前回の判定状態 (None の場合は新規作成)

    Returns:
        (InclineLevel, HysteresisState): 判定結果と更新された状態
    """
    if state is None:
        state = HysteresisState()

    max_deg = max(abs(roll_deg), abs(pitch_deg))
    current = state.current_level

    if current == InclineLevel.SAFE:
        if max_deg >= emergency_deg:
            state.current_level = InclineLevel.EMERGENCY
        elif max_deg >= warning_deg:
            state.current_level = InclineLevel.WARNING

    elif current == InclineLevel.WARNING:
        if max_deg >= emergency_deg:
            state.current_level = InclineLevel.EMERGENCY
        elif max_deg < warning_deg - hysteresis_deg:
            state.current_level = InclineLevel.SAFE

    elif current == InclineLevel.EMERGENCY:
        if max_deg < warning_deg - hysteresis_deg:
            state.current_level = InclineLevel.SAFE
        elif max_deg < emergency_deg - hysteresis_deg:
            state.current_level = InclineLevel.WARNING

    return state.current_level, state


def calibrate_imu_offset(
    samples: list[tuple[float, float]],
) -> tuple[float, float]:
    """
    IMU キャリブレーション — 取り付けオフセットを推定する

    水平面で取得した (roll_deg, pitch_deg) サンプルの平均を
    取り付けオフセットとして返す。

    Args:
        samples: [(roll_deg, pitch_deg), ...] の計測サンプル列

    Returns:
        (offset_roll_deg, offset_pitch_deg): オフセット [度]
    """
    if not samples:
        return 0.0, 0.0

    n = len(samples)
    sum_roll = sum(s[0] for s in samples)
    sum_pitch = sum(s[1] for s in samples)
    return sum_roll / n, sum_pitch / n


def should_stop(level: InclineLevel) -> bool:
    """
    傾斜レベルに基づき緊急停止すべきか判定する

    EMERGENCY の場合のみ True を返す
    """
    return level == InclineLevel.EMERGENCY
