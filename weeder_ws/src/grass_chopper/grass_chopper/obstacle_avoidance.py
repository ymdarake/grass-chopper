"""
============================================================================
障害物回避 純粋計算ロジック (obstacle_avoidance)
============================================================================
ROS 2 に依存しない純粋関数モジュール。
Mac ホスト上で pytest のみでテスト実行可能。

WeederNode (weeder_node.py) はこのモジュールに計算処理を委譲する。
============================================================================
"""

import math
from dataclasses import dataclass
from enum import Enum, auto


# ============================================================================
# データ型
# ============================================================================

class RobotState(Enum):
    """ロボットの行動状態"""
    FORWARD = auto()      # 前方クリア: 直進
    AVOID_LEFT = auto()   # 左が空いている: 左回転回避
    AVOID_RIGHT = auto()  # 右が空いている: 右回転回避
    WALL_FOLLOW = auto()  # 壁沿い走行
    U_TURN = auto()       # 行き止まり: Uターン


@dataclass(frozen=True)
class TwistCommand:
    """ROS 2 非依存の速度指令"""
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class AvoidanceParams:
    """回避アルゴリズムの全パラメータを集約"""
    safe_distance: float = 0.5
    forward_speed: float = 0.2
    turn_speed: float = 0.5
    wall_target_distance: float = 0.5
    wall_follow_kp: float = 1.5
    wall_follow_kd: float = 0.3
    wall_follow_speed: float = 0.15
    angular_z_limit: float = 0.8
    u_turn_speed: float = 1.0


@dataclass(frozen=True)
class ScanConfig:
    """LaserScan メタデータ (ROS フリー)"""
    angle_min: float = -math.pi
    angle_increment: float = math.pi / 180
    range_min: float = 0.12
    range_max: float = 10.0


# ============================================================================
# 純粋関数
# ============================================================================

def deg_to_index(deg: float, angle_min: float, angle_increment: float) -> int:
    """
    角度[度]を ranges 配列のインデックスに変換

    Args:
        deg: 角度 [度] (-180 ~ 180)
        angle_min: スキャン開始角度 [rad]
        angle_increment: 角度刻み [rad]
    Returns:
        ranges 配列のインデックス
    """
    if angle_increment == 0.0:
        return 0
    rad = math.radians(deg)
    return round((rad - angle_min) / angle_increment)


def get_zone_stats(
    ranges: list,
    start_deg: float,
    end_deg: float,
    range_min: float,
    range_max: float,
    angle_min: float = -math.pi,
    angle_increment: float = 0.0,
) -> tuple:
    """
    指定角度範囲の最小距離と平均距離を返す

    Args:
        ranges: 距離データ配列
        start_deg: ゾーン開始角度 [度]
        end_deg: ゾーン終了角度 [度]
        range_min: 有効最小距離
        range_max: 有効最大距離
        angle_min: スキャン開始角度 [rad]
        angle_increment: 角度刻み [rad] (0の場合は自動計算)
    Returns:
        (min_distance, avg_distance)。有効データなしの場合は (inf, inf)
    """
    num_samples = len(ranges)
    if num_samples == 0:
        return (float('inf'), float('inf'))

    # angle_increment が指定されていない場合は自動計算（後方互換性）
    if angle_increment == 0.0:
        angle_increment = (2 * math.pi) / num_samples

    idx1 = deg_to_index(start_deg, angle_min, angle_increment)
    idx2 = deg_to_index(end_deg, angle_min, angle_increment)
    start_idx = max(0, min(idx1, idx2))
    end_idx = min(num_samples - 1, max(idx1, idx2))

    zone_ranges = ranges[start_idx:end_idx + 1]
    valid = [
        r for r in zone_ranges
        if not math.isinf(r) and not math.isnan(r)
        and r > range_min and r < range_max
    ]

    if not valid:
        return (float('inf'), float('inf'))

    return (min(valid), sum(valid) / len(valid))


def analyze_zones(ranges: list, scan_config: ScanConfig) -> dict:
    """
    距離データをゾーンごとに解析する

    Args:
        ranges: 距離データ配列
        scan_config: スキャン設定
    Returns:
        ゾーンごとの (min, avg) 辞書
    """
    r_min = scan_config.range_min
    r_max = scan_config.range_max
    a_min = scan_config.angle_min
    a_inc = scan_config.angle_increment

    front_min, front_avg = get_zone_stats(
        ranges, -30.0, 30.0, r_min, r_max, a_min, a_inc)
    left_min, left_avg = get_zone_stats(
        ranges, 60.0, 120.0, r_min, r_max, a_min, a_inc)
    right_min, right_avg = get_zone_stats(
        ranges, -120.0, -60.0, r_min, r_max, a_min, a_inc)

    return {
        'front_min': front_min, 'front_avg': front_avg,
        'left_min': left_min, 'left_avg': left_avg,
        'right_min': right_min, 'right_avg': right_avg,
    }


def update_state(
    current_state: RobotState,
    zones: dict,
    params: AvoidanceParams,
) -> tuple:
    """
    ゾーン情報に基づいて次の状態を決定する (純粋関数)

    Args:
        current_state: 現在の状態
        zones: ゾーン解析結果
        params: 回避パラメータ
    Returns:
        (new_state, wall_error_reset)
        wall_error_reset: True の場合 prev_wall_error をリセットすべき
    """
    front_min = zones['front_min']
    left_min = zones['left_min']
    right_min = zones['right_min']
    left_avg = zones['left_avg']
    right_avg = zones['right_avg']

    # WALL_FOLLOW から他の状態に遷移する場合は常にエラーリセット
    leaving_wall_follow = current_state == RobotState.WALL_FOLLOW

    # 行き止まり: 三方が塞がれている → U_TURN
    if (front_min < params.safe_distance
            and left_min < params.safe_distance
            and right_min < params.safe_distance):
        return (RobotState.U_TURN, leaving_wall_follow)

    # 前方に障害物 → 回避方向を選択
    if front_min < params.safe_distance:
        # 既に回避中なら方向を維持 (チャタリング防止)
        if current_state in (RobotState.AVOID_LEFT, RobotState.AVOID_RIGHT):
            return (current_state, False)
        if left_avg > right_avg:
            return (RobotState.AVOID_LEFT, leaving_wall_follow)
        else:
            return (RobotState.AVOID_RIGHT, leaving_wall_follow)

    # 回避中に前方がクリアになった → FORWARD に復帰
    if current_state in (RobotState.AVOID_LEFT, RobotState.AVOID_RIGHT,
                         RobotState.U_TURN):
        if front_min > params.safe_distance * 1.2:
            return (RobotState.FORWARD, False)

    # WALL_FOLLOW 状態での遷移判定
    if current_state == RobotState.WALL_FOLLOW:
        # 右側に壁がなくなった → FORWARD に復帰
        if right_min > params.wall_target_distance * 2.0:
            return (RobotState.FORWARD, True)
        # 壁沿い継続
        return (RobotState.WALL_FOLLOW, False)

    # FORWARD 状態で右側に壁が近い → WALL_FOLLOW に遷移
    if current_state == RobotState.FORWARD:
        if right_min < params.wall_target_distance * 1.5 and front_min >= params.safe_distance:
            return (RobotState.WALL_FOLLOW, True)

    return (current_state, False)


def compute_wall_follow_twist(
    right_min: float,
    prev_error: float,
    params: AvoidanceParams,
) -> tuple:
    """
    PD 制御で壁沿い走行の速度指令を生成 (純粋関数)

    error > 0: 壁に近すぎる → angular_z > 0 (左に曲がって離れる)
    error < 0: 壁から遠すぎる → angular_z < 0 (右に曲がって近づく)

    Args:
        right_min: 右側の最小距離
        prev_error: 前回のエラー値
        params: 回避パラメータ
    Returns:
        (TwistCommand, new_error)
    """
    error = params.wall_target_distance - right_min
    p_term = params.wall_follow_kp * error
    d_term = params.wall_follow_kd * (error - prev_error)

    angular_z = max(-params.angular_z_limit,
                    min(params.angular_z_limit, p_term + d_term))

    return (
        TwistCommand(linear_x=params.wall_follow_speed, angular_z=angular_z),
        error,
    )


def compute_twist(
    state: RobotState,
    zones: dict,
    prev_error: float,
    params: AvoidanceParams,
) -> tuple:
    """
    現在の状態に応じた速度指令を生成 (純粋関数)

    Args:
        state: 現在の状態
        zones: ゾーン解析結果
        prev_error: 前回の壁沿いエラー値
        params: 回避パラメータ
    Returns:
        (TwistCommand, new_prev_error)
    """
    if state == RobotState.FORWARD:
        return (TwistCommand(linear_x=params.forward_speed, angular_z=0.0), prev_error)
    elif state == RobotState.AVOID_LEFT:
        return (TwistCommand(linear_x=0.0, angular_z=params.turn_speed), prev_error)
    elif state == RobotState.AVOID_RIGHT:
        return (TwistCommand(linear_x=0.0, angular_z=-params.turn_speed), prev_error)
    elif state == RobotState.U_TURN:
        return (TwistCommand(linear_x=0.0, angular_z=params.u_turn_speed), prev_error)
    elif state == RobotState.WALL_FOLLOW:
        return compute_wall_follow_twist(zones['right_min'], prev_error, params)

    return (TwistCommand(), prev_error)
