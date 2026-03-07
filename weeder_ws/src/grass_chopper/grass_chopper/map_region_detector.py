"""
============================================================================
地図ベース自動領域検出 純粋計算ロジック (map_region_detector)
============================================================================
ROS 2 に依存しない純粋関数モジュール。
Mac ホスト上で pytest のみでテスト実行可能。

SLAM /map (OccupancyGrid) から走行領域・障害物を自動抽出する。
パイプライン: 二値化 → CLOSE → erode → findContours → pixel→world → Shapely
============================================================================
"""

import cv2
import numpy as np
from shapely.geometry import Polygon

from grass_chopper.coverage_tracker import GridConfig


def occupancy_grid_to_binary(
    grid_data: np.ndarray,
    config: GridConfig,
    free_threshold: int = 50,
) -> np.ndarray:
    """
    OccupancyGrid の 1D データを二値画像に変換する

    Args:
        grid_data: OccupancyGrid.data (1D int8, 0=free, 100=occupied, -1=unknown)
        config: グリッド設定 (width, height)
        free_threshold: この値未満かつ 0 以上のセルを free (255) とする
    Returns:
        二値画像 (uint8, 255=free, 0=occupied/unknown), shape=(height, width)
    """
    grid_2d = np.array(grid_data, dtype=np.int8).reshape((config.height, config.width))
    binary = np.where(
        (grid_2d >= 0) & (grid_2d < free_threshold),
        np.uint8(255),
        np.uint8(0),
    )
    return binary


def extract_free_regions(
    binary_mask: np.ndarray,
    config: GridConfig,
    min_area: float = 1.0,
    robot_radius: float = 0.2,
) -> list[Polygon]:
    """
    二値画像から走行可能領域をポリゴンとして抽出する

    Args:
        binary_mask: 二値画像 (255=free, 0=occupied), shape=(height, width)
        config: グリッド設定
        min_area: 最小面積 [m²] (これ以下は除外)
        robot_radius: 安全マージン [m] (erode でこの分だけ縮小)
    Returns:
        走行可能領域の Shapely Polygon リスト
    """
    processed = binary_mask.copy()

    # モルフォロジー CLOSE (穴埋め): DILATE → ERODE
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    processed = cv2.morphologyEx(processed, cv2.MORPH_CLOSE, kernel_close)

    # 安全マージン (erode)
    if robot_radius > 0:
        erode_pixels = max(1, int(robot_radius / config.resolution))
        kernel_erode = cv2.getStructuringElement(
            cv2.MORPH_RECT, (2 * erode_pixels + 1, 2 * erode_pixels + 1))
        processed = cv2.erode(processed, kernel_erode)

    # 輪郭抽出 (RETR_CCOMP: 穴あき対応)
    contours, hierarchy = cv2.findContours(
        processed, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

    if not contours or hierarchy is None:
        return []

    regions = []
    hierarchy = hierarchy[0]  # shape: (N, 4)

    for i, contour in enumerate(contours):
        # RETR_CCOMP: hierarchy[i][3] == -1 は外側輪郭
        if hierarchy[i][3] != -1:
            continue  # 内部輪郭 (穴) はスキップ

        if len(contour) < 3:
            continue

        # ピクセル座標 → ワールド座標
        exterior_points = _contour_to_world(contour, config)
        if len(exterior_points) < 3:
            continue

        # 穴 (子輪郭) を収集
        holes = []
        child_idx = hierarchy[i][2]  # 最初の子
        while child_idx != -1:
            child_contour = contours[child_idx]
            if len(child_contour) >= 3:
                hole_points = _contour_to_world(child_contour, config)
                if len(hole_points) >= 3:
                    holes.append(hole_points)
            child_idx = hierarchy[child_idx][0]  # 次の兄弟

        poly = Polygon(exterior_points, holes)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            continue

        poly = poly.simplify(config.resolution * 0.5)

        if poly.area >= min_area:
            regions.append(poly)

    return regions


def detect_obstacles_from_map(
    grid_data: np.ndarray,
    config: GridConfig,
    min_area: float = 0.1,
) -> list[Polygon]:
    """
    OccupancyGrid から障害物領域を抽出する

    free 領域の内部にある occupied ブロックを障害物として検出する。

    Args:
        grid_data: OccupancyGrid.data (1D int8)
        config: グリッド設定
        min_area: 最小面積 [m²]
    Returns:
        障害物の Shapely Polygon リスト
    """
    grid_2d = np.array(grid_data, dtype=np.int8).reshape((config.height, config.width))

    # occupied セル (>= 50) を白にした二値画像
    obstacle_mask = np.where(grid_2d >= 50, np.uint8(255), np.uint8(0))

    # ノイズ除去 (OPEN)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    cleaned = cv2.morphologyEx(obstacle_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(
        cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    obstacles = []
    for contour in contours:
        if len(contour) < 3:
            continue

        points = _contour_to_world(contour, config)
        if len(points) < 3:
            continue

        poly = Polygon(points)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            continue

        poly = poly.simplify(config.resolution * 0.5)

        if poly.area >= min_area:
            obstacles.append(poly)

    return obstacles


def select_largest_region(regions: list[Polygon]) -> Polygon | None:
    """
    最大面積のポリゴンを選択する

    Args:
        regions: Polygon のリスト
    Returns:
        最大面積の Polygon、空リストなら None
    """
    if not regions:
        return None
    return max(regions, key=lambda p: p.area)


def _contour_to_world(contour: np.ndarray, config: GridConfig) -> list[tuple[float, float]]:
    """OpenCV 輪郭のピクセル座標をワールド座標に変換"""
    points = []
    for pt in contour:
        col, row = pt[0]
        wx = config.origin_x + (col + 0.5) * config.resolution
        wy = config.origin_y + (row + 0.5) * config.resolution
        points.append((wx, wy))
    return points
