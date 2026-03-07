"""
============================================================================
ドッキング/帰還行動 純粋計算ロジック (docking_behavior)
============================================================================
ROS 2 に依存しない純粋関数モジュール。
Mac ホスト上で pytest のみでテスト実行可能。

MVP: NavigateToPose でホーム位置へ帰還。到着 = 充電開始と単純化。
将来: opennav_docking で精密ドッキング (AprilTag)。
============================================================================
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class DockingParams:
    """ドッキングパラメータ"""
    home_x: float = 0.0              # ホーム (充電ステーション) x [m]
    home_y: float = 0.0              # ホーム y [m]
    home_yaw: float = 0.0            # ドック正面の方向 [rad]
    approach_distance: float = 0.5   # 接近距離 [m]
    dock_tolerance: float = 0.1      # ドック到達判定許容誤差 [m]


def compute_approach_pose(
    params: DockingParams,
) -> tuple[float, float, float]:
    """
    ドックへの接近ポーズを計算する

    ドック正面から approach_distance 手前の位置を返す。

    Args:
        params: ドッキングパラメータ
    Returns:
        (x, y, yaw): 接近ポーズ
    """
    # ドック正面方向の逆方向に approach_distance 分後退した位置
    ax = params.home_x - params.approach_distance * math.cos(params.home_yaw)
    ay = params.home_y - params.approach_distance * math.sin(params.home_yaw)
    return ax, ay, params.home_yaw


def is_at_dock(x: float, y: float, params: DockingParams) -> bool:
    """
    ドック位置に到達しているか判定する

    Args:
        x, y: 現在位置
        params: ドッキングパラメータ
    Returns:
        ドック許容誤差内なら True
    """
    dist = math.sqrt((x - params.home_x) ** 2 + (y - params.home_y) ** 2)
    return dist <= params.dock_tolerance


def compute_dock_alignment_twist(
    current_x: float,
    current_y: float,
    current_yaw: float,
    params: DockingParams,
) -> tuple[float, float]:
    """
    ドックへのアライメント補正速度を計算する (P 制御)

    Args:
        current_x, current_y: 現在位置
        current_yaw: 現在の向き [rad]
        params: ドッキングパラメータ
    Returns:
        (linear_x, angular_z): 速度指令
    """
    dx = params.home_x - current_x
    dy = params.home_y - current_y
    dist = math.sqrt(dx * dx + dy * dy)

    if dist < params.dock_tolerance:
        return 0.0, 0.0

    # 目標方向
    target_yaw = math.atan2(dy, dx)

    # 角度差 (-π ~ π)
    yaw_error = target_yaw - current_yaw
    while yaw_error > math.pi:
        yaw_error -= 2 * math.pi
    while yaw_error < -math.pi:
        yaw_error += 2 * math.pi

    # P 制御
    kp_linear = 0.5
    kp_angular = 1.0

    linear = min(kp_linear * dist, 0.2)   # 最大 0.2 m/s
    angular = kp_angular * yaw_error

    return linear, angular
