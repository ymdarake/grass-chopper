"""
============================================================================
ミッション管理 純粋計算ロジック (mission_behaviors)
============================================================================
ROS 2 / py_trees に依存しない純粋関数モジュール。
Mac ホスト上で pytest のみでテスト実行可能。

mission_tree_node.py (ROS 2 + py_trees_ros アダプター) から使用される。
============================================================================
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MissionState:
    """ミッション状態 (BT の Blackboard から構築)"""
    battery_percentage: float   # バッテリー残量 (0.0 ~ 1.0)
    is_charging: bool           # 充電中か
    coverage_ratio: float       # カバレッジ率 (0.0 ~ 1.0)
    target_coverage: float      # 目標カバレッジ率
    is_coverage_running: bool   # カバレッジ走行が実行中か


def should_return_home(state: MissionState, low_threshold: float) -> bool:
    """
    ホームに帰還すべきか判定する

    条件: バッテリーが低く、充電中でない
    """
    if state.is_charging:
        return False
    return state.battery_percentage <= low_threshold


def should_start_coverage(state: MissionState) -> bool:
    """
    カバレッジ走行を開始すべきか判定する

    条件: 充電中でなく、カバレッジ未完了で、走行中でない
    """
    if state.is_charging:
        return False
    if state.is_coverage_running:
        return False
    if state.coverage_ratio >= state.target_coverage:
        return False
    return True


def should_continue_charging(state: MissionState) -> bool:
    """
    充電を継続すべきか判定する

    条件: 充電中で、満充電でない
    """
    if not state.is_charging:
        return False
    return state.battery_percentage < 0.999


def compute_home_pose(
    current_x: float,
    current_y: float,
    home_x: float,
    home_y: float,
) -> tuple[float, float, float]:
    """
    ホーム位置への移動ポーズを計算する

    Args:
        current_x, current_y: 現在位置
        home_x, home_y: ホーム位置
    Returns:
        (x, y, yaw): ホーム位置の座標とそこへ向かう方向
    """
    dx = home_x - current_x
    dy = home_y - current_y
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        yaw = 0.0
    else:
        yaw = math.atan2(dy, dx)
    return home_x, home_y, yaw
