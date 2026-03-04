"""
============================================================================
カバレッジ走行 純粋計算ロジック (coverage_planner)
============================================================================
ROS 2 に依存しない純粋関数モジュール。
Mac ホスト上で pytest のみでテスト実行可能。

CoverageCommanderNode (coverage_commander_node.py) はこのモジュールに
計算処理を委譲する。

Boustrophedon (牛の鋤跡) パターンでジグザグウェイポイントを生成し、
面塗りカバレッジ走行を実現する。
============================================================================
"""

import math
from dataclasses import dataclass

from shapely.affinity import rotate
from shapely.geometry import LineString, Point, Polygon


# ============================================================================
# データ型
# ============================================================================

@dataclass(frozen=True)
class CoverageParams:
    """カバレッジ走行のパラメータ"""
    swath_width: float = 0.18   # 作業幅 [m] (ロボットの草刈り幅)
    margin: float = 0.3         # 境界マージン [m] (壁からの安全距離)
    direction: float = 0.0      # 走行方向 [rad] (0=東西方向のストライプ)


@dataclass(frozen=True)
class Waypoint:
    """ウェイポイント (2D 座標 + 向き)"""
    x: float
    y: float
    yaw: float  # 進行方向 [rad]


# ============================================================================
# 純粋関数
# ============================================================================

def shrink_polygon(polygon: Polygon, margin: float) -> Polygon:
    """
    ポリゴンを内側に margin 分縮小する

    Args:
        polygon: 元のポリゴン
        margin: 縮小距離 [m]
    Returns:
        縮小されたポリゴン。margin が大きすぎて消滅した場合は空の Polygon
    """
    if polygon.is_empty or margin <= 0.0:
        return polygon

    shrunk = polygon.buffer(-margin)
    if shrunk.is_empty:
        return Polygon()
    if not isinstance(shrunk, Polygon):
        # 凹ポリゴンなどの縮小で MultiPolygon に分割された場合は最大面積を採用
        return max(shrunk.geoms, key=lambda p: p.area)
    return shrunk


def compute_yaw(p1: Waypoint, p2: Waypoint) -> float:
    """
    2点間の方向角を計算する

    Args:
        p1: 始点
        p2: 終点
    Returns:
        方向角 [rad] (-pi ~ pi)
    """
    dx = p2.x - p1.x
    dy = p2.y - p1.y
    if dx == 0.0 and dy == 0.0:
        return 0.0
    return math.atan2(dy, dx)


def generate_boustrophedon_waypoints(
    polygon: Polygon,
    params: CoverageParams,
) -> list:
    """
    Boustrophedon パターンでジグザグウェイポイントを生成する

    ポリゴンを margin 分縮小した後、swath_width 間隔で平行なストライプを引き、
    各ストライプとポリゴンの交差点をウェイポイントとして返す。

    Args:
        polygon: カバレッジ対象ポリゴン
        params: カバレッジパラメータ
    Returns:
        Waypoint のリスト (ジグザグ順)
    """
    if polygon.is_empty:
        return []

    # 1. マージン分縮小
    work_area = shrink_polygon(polygon, params.margin)
    if work_area.is_empty:
        return []

    # 回転の基点となる重心を保存しておく (逆回転復元時に再利用)
    centroid = work_area.centroid
    cx, cy = centroid.x, centroid.y

    # 2. direction が 0 でない場合、ポリゴンを逆回転して軸をそろえる
    if params.direction != 0.0:
        work_area = rotate(work_area, -math.degrees(params.direction), origin='centroid')

    # 3. バウンディングボックスを取得
    minx, miny, maxx, maxy = work_area.bounds

    # 4. y 方向にストライプを生成 (水平方向のスキャンライン)
    waypoints_raw = []
    y = miny + params.swath_width / 2
    row_index = 0

    while y <= maxy - params.swath_width / 2 + 1e-9:
        # スキャンラインとポリゴンの交差を計算
        scan_line = LineString([(minx - 1, y), (maxx + 1, y)])
        intersection = work_area.intersection(scan_line)

        if intersection.is_empty:
            y += params.swath_width
            row_index += 1
            continue

        # 交差結果から x 座標の範囲を取得 (bounds で安全に)
        bounds = intersection.bounds
        if not bounds:
            y += params.swath_width
            row_index += 1
            continue

        minx_int, _, maxx_int, _ = bounds

        # 点のみなど、交差幅が短すぎる場合はスキップ
        if maxx_int - minx_int < 1e-3:
            y += params.swath_width
            row_index += 1
            continue

        x_start = minx_int
        x_end = maxx_int

        # ジグザグ: 偶数行は左→右、奇数行は右→左
        if row_index % 2 == 0:
            waypoints_raw.append((x_start, y))
            waypoints_raw.append((x_end, y))
        else:
            waypoints_raw.append((x_end, y))
            waypoints_raw.append((x_start, y))

        y += params.swath_width
        row_index += 1

    if not waypoints_raw:
        return []

    # 5. direction が 0 でない場合、座標を元に戻す (正回転)
    if params.direction != 0.0:
        cos_d = math.cos(params.direction)
        sin_d = math.sin(params.direction)

        rotated = []
        for x, y in waypoints_raw:
            dx = x - cx
            dy = y - cy
            rx = cx + dx * cos_d - dy * sin_d
            ry = cy + dx * sin_d + dy * cos_d
            rotated.append((rx, ry))
        waypoints_raw = rotated

    # 6. yaw を計算して Waypoint リストに変換
    waypoints = []
    for i, (x, y) in enumerate(waypoints_raw):
        if i < len(waypoints_raw) - 1:
            nx, ny = waypoints_raw[i + 1]
            yaw = math.atan2(ny - y, nx - x)
        elif waypoints:
            yaw = waypoints[-1].yaw
        else:
            yaw = 0.0
        waypoints.append(Waypoint(x=x, y=y, yaw=yaw))

    return waypoints


def estimate_coverage_ratio(
    waypoints: list,
    polygon: Polygon,
    swath_width: float,
) -> float:
    """
    ウェイポイント列からカバレッジ率を推定する

    各連続するウェイポイント間の走行パスを swath_width でバッファリングし、
    ポリゴンとの重複面積からカバレッジ率を算出する。

    Args:
        waypoints: Waypoint のリスト
        polygon: カバレッジ対象ポリゴン
        swath_width: 作業幅 [m]
    Returns:
        カバレッジ率 (0.0 ~ 1.0)
    """
    if not waypoints or polygon.is_empty or polygon.area == 0:
        return 0.0

    # ウェイポイント間を結んだ LineString をバッファリング
    if len(waypoints) < 2:
        return 0.0

    coords = [(wp.x, wp.y) for wp in waypoints]
    path = LineString(coords)
    covered = path.buffer(swath_width / 2, cap_style='flat')
    intersection = covered.intersection(polygon)

    ratio = intersection.area / polygon.area
    return min(ratio, 1.0)
