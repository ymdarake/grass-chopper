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


def should_stop(level: InclineLevel) -> bool:
    """
    傾斜レベルに基づき緊急停止すべきか判定する

    EMERGENCY の場合のみ True を返す
    """
    return level == InclineLevel.EMERGENCY
