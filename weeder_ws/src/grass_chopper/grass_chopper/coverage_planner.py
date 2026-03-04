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
from shapely.geometry import LineString, MultiPolygon, Point, Polygon
from shapely.ops import unary_union


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


def _extract_segments(intersection) -> list:
    """
    スキャンラインとポリゴンの交差結果から (x_start, x_end) セグメントを抽出

    LineString → 1セグメント、MultiLineString → 複数セグメント
    短すぎるセグメント (< 1e-3) はフィルタする。
    結果は x_start の昇順でソート済み。
    """
    from shapely.geometry import MultiLineString as MultiLS

    geoms = []
    if isinstance(intersection, LineString):
        geoms = [intersection]
    elif isinstance(intersection, MultiLS):
        geoms = list(intersection.geoms)
    elif hasattr(intersection, 'geoms'):
        # GeometryCollection 等
        geoms = [g for g in intersection.geoms if isinstance(g, LineString)]
    else:
        # Point など → スキップ
        return []

    segments = []
    for geom in geoms:
        if geom.is_empty:
            continue
        b = geom.bounds
        x_start, x_end = b[0], b[2]
        if x_end - x_start >= 1e-3:
            segments.append((x_start, x_end))

    segments.sort(key=lambda s: s[0])
    return segments


def _generate_single_polygon_waypoints(
    work_area: Polygon,
    params: CoverageParams,
) -> list:
    """
    単一ポリゴンに対する Boustrophedon ウェイポイント生成 (内部ヘルパー)

    マージン縮小済みのポリゴンに対して実行する。
    """
    if work_area.is_empty:
        return []

    # 回転の基点となる重心を保存しておく (逆回転復元時に再利用)
    centroid = work_area.centroid
    cx, cy = centroid.x, centroid.y

    # direction が 0 でない場合、ポリゴンを逆回転して軸をそろえる
    rotated_area = work_area
    if params.direction != 0.0:
        rotated_area = rotate(work_area, -math.degrees(params.direction),
                              origin='centroid')

    # バウンディングボックスを取得
    minx, miny, maxx, maxy = rotated_area.bounds

    # y 方向にストライプを生成 (水平方向のスキャンライン)
    waypoints_raw = []
    y = miny + params.swath_width / 2
    row_index = 0

    while y <= maxy - params.swath_width / 2 + 1e-9:
        scan_line = LineString([(minx - 1, y), (maxx + 1, y)])
        intersection = rotated_area.intersection(scan_line)

        if intersection.is_empty:
            y += params.swath_width
            row_index += 1
            continue

        # 交差結果からセグメントを抽出 (MultiLineString 対応)
        segments = _extract_segments(intersection)

        if not segments:
            y += params.swath_width
            row_index += 1
            continue

        # ジグザグ: 偶数行は左→右、奇数行は右→左
        if row_index % 2 == 1:
            segments.reverse()
            segments = [(x_end, x_start) for x_start, x_end in segments]

        for x_start, x_end in segments:
            waypoints_raw.append((x_start, y))
            waypoints_raw.append((x_end, y))

        y += params.swath_width
        row_index += 1

    if not waypoints_raw:
        return []

    # direction が 0 でない場合、座標を元に戻す (正回転)
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

    return waypoints_raw


def _raw_to_waypoints(waypoints_raw: list) -> list:
    """(x, y) タプルリストを Waypoint リストに変換 (yaw 計算付き)"""
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


def generate_boustrophedon_waypoints(
    polygon: Polygon,
    params: CoverageParams,
    obstacles: list | None = None,
) -> list:
    """
    Boustrophedon パターンでジグザグウェイポイントを生成する

    ポリゴンを margin 分縮小した後、swath_width 間隔で平行なストライプを引き、
    各ストライプとポリゴンの交差点をウェイポイントとして返す。

    obstacles が指定された場合、polygon.difference(union(obstacles)) で
    走行可能領域を計算し、MultiPolygon なら各サブポリゴンを貪欲法で
    近い順に走行する。

    Args:
        polygon: カバレッジ対象ポリゴン
        params: カバレッジパラメータ
        obstacles: 障害物ポリゴンのリスト (None=障害物なし)
    Returns:
        Waypoint のリスト (ジグザグ順)
    """
    if polygon.is_empty:
        return []

    # 1. マージン分縮小
    work_area = shrink_polygon(polygon, params.margin)
    if work_area.is_empty:
        return []

    # 2. 障害物があれば差分を計算
    if obstacles:
        obstacle_union = unary_union(obstacles)
        work_area = work_area.difference(obstacle_union)
        if work_area.is_empty:
            return []

    # 3. MultiPolygon の場合はサブポリゴンに分解して貪欲法で走行順序決定
    if isinstance(work_area, MultiPolygon):
        sub_polygons = list(work_area.geoms)
    else:
        sub_polygons = [work_area]

    # 面積が小さすぎるサブポリゴンをフィルタ
    sub_polygons = [p for p in sub_polygons
                    if not p.is_empty and p.area > params.swath_width ** 2]

    if not sub_polygons:
        return []

    # 貪欲法: 現在位置から最も近いサブポリゴンを次に選択
    all_raw = []
    current_pos = (0.0, 0.0)  # 初期位置 (ロボットの出発点想定)

    remaining = list(sub_polygons)
    while remaining:
        # 各サブポリゴンの重心距離で最近選択
        nearest_idx = min(
            range(len(remaining)),
            key=lambda i: (remaining[i].centroid.x - current_pos[0]) ** 2 +
                          (remaining[i].centroid.y - current_pos[1]) ** 2
        )
        sub = remaining.pop(nearest_idx)

        raw = _generate_single_polygon_waypoints(sub, params)
        if raw:
            all_raw.extend(raw)
            current_pos = raw[-1]  # 最後のウェイポイントを現在位置に更新

    if not all_raw:
        return []

    return _raw_to_waypoints(all_raw)


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
