"""
============================================================================
カバレッジ追跡 純粋計算ロジック (coverage_tracker)
============================================================================
ROS 2 に依存しない純粋関数モジュール。
Mac ホスト上で pytest のみでテスト実行可能。

NumPy 2D グリッドでロボットの走行軌跡を記録し、
リアルタイムでカバレッジ率を計算する。
============================================================================
"""

from dataclasses import dataclass

import numpy as np
from shapely.geometry import Polygon


# ============================================================================
# データ型
# ============================================================================

@dataclass(frozen=True)
class GridConfig:
    """グリッド設定"""
    width: int          # グリッド幅 [cells]
    height: int         # グリッド高 [cells]
    resolution: float   # [m/cell]
    origin_x: float     # 左下 x [m]
    origin_y: float     # 左下 y [m]


# ============================================================================
# CoverageTracker
# ============================================================================

class CoverageTracker:
    """
    NumPy グリッドベースのカバレッジ追跡

    2D int8 配列 (0=未踏破, 100=踏破済み) でロボットの走行軌跡を記録し、
    リアルタイムでカバレッジ率を計算する。

    Args:
        config: グリッド設定
        target_polygon: カバレッジ対象ポリゴン (None=全セル対象)
    """

    def __init__(
        self,
        config: GridConfig,
        target_polygon: Polygon | None = None,
    ):
        self._config = config
        self._grid = np.zeros((config.height, config.width), dtype=np.int8)

        # ターゲットマスク: True = カバレッジ対象セル
        if target_polygon is not None and not target_polygon.is_empty:
            self._target_mask = self._build_polygon_mask(target_polygon)
        else:
            self._target_mask = np.ones(
                (config.height, config.width), dtype=bool)

    def _build_polygon_mask(self, polygon: Polygon) -> np.ndarray:
        """ポリゴン内のセルを True とするマスクを生成 (Shapely contains_xy)"""
        import shapely

        cfg = self._config
        cols = np.arange(cfg.width)
        rows = np.arange(cfg.height)
        col_grid, row_grid = np.meshgrid(cols, rows)

        wx = cfg.origin_x + (col_grid + 0.5) * cfg.resolution
        wy = cfg.origin_y + (row_grid + 0.5) * cfg.resolution

        return shapely.contains_xy(polygon, wx, wy)

    def mark_covered(self, x: float, y: float, tool_radius: float) -> None:
        """
        ロボット位置 (x, y) から tool_radius 内のセルを踏破済みにする

        バウンディングボックスで候補セルを絞り込み、円判定でマークする。
        グリッド外の座標は無視する。
        """
        cfg = self._config

        # ワールド座標 → セル座標の範囲 (バウンディングボックス)
        col_min = int((x - tool_radius - cfg.origin_x) / cfg.resolution)
        col_max = int((x + tool_radius - cfg.origin_x) / cfg.resolution) + 1
        row_min = int((y - tool_radius - cfg.origin_y) / cfg.resolution)
        row_max = int((y + tool_radius - cfg.origin_y) / cfg.resolution) + 1

        # グリッド範囲にクランプ
        col_min = max(0, col_min)
        col_max = min(cfg.width, col_max)
        row_min = max(0, row_min)
        row_max = min(cfg.height, row_max)

        if col_min >= col_max or row_min >= row_max:
            return

        # バウンディングボックス内で円判定 (NumPy ベクトル化)
        rows = np.arange(row_min, row_max)
        cols = np.arange(col_min, col_max)
        col_grid, row_grid = np.meshgrid(cols, rows)

        cx = cfg.origin_x + (col_grid + 0.5) * cfg.resolution
        cy = cfg.origin_y + (row_grid + 0.5) * cfg.resolution
        dist_sq = (cx - x) ** 2 + (cy - y) ** 2

        mask = dist_sq <= tool_radius * tool_radius
        self._grid[row_min:row_max, col_min:col_max] = np.where(
            mask, np.int8(100), self._grid[row_min:row_max, col_min:col_max])

    def get_coverage_ratio(self) -> float:
        """
        カバレッジ率を返す (0.0 ~ 1.0)

        target_polygon が指定されている場合、ポリゴン内セルのみで計算。
        """
        target_count = np.count_nonzero(self._target_mask)
        if target_count == 0:
            return 0.0

        covered_in_target = np.count_nonzero(
            (self._grid == 100) & self._target_mask)
        return covered_in_target / target_count

    def get_uncovered_cells(self) -> np.ndarray:
        """
        未踏破セルの bool 配列を返す

        True = 未踏破 (かつターゲット内), False = 踏破済みまたはターゲット外
        """
        return (self._grid == 0) & self._target_mask

    def to_occupancy_grid(self) -> np.ndarray:
        """
        ROS OccupancyGrid 配信用の int8 配列を返す

        0=未踏破, 100=踏破済み
        """
        return self._grid.copy()


# ============================================================================
# 刈り残し検出
# ============================================================================

def detect_uncovered_regions(
    coverage_grid: np.ndarray,
    config: GridConfig,
    min_area: float = 0.1,
) -> list:
    """
    未カバー領域をポリゴンのリストとして返す

    Args:
        coverage_grid: CoverageTracker の内部グリッド (int8, 0/100)
        config: グリッド設定
        min_area: 最小面積 [m²] (これ以下は無視)
    Returns:
        未カバー領域の Shapely Polygon リスト
    """
    import cv2

    # 未踏破セル (0) をバイナリマスクに変換 (255=未踏破, 0=踏破済み)
    binary = np.where(coverage_grid == 0, np.uint8(255), np.uint8(0))

    # ノイズ除去 (モルフォロジー OPEN = 侵食→膨張)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

    # 輪郭抽出
    contours, _ = cv2.findContours(
        cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    regions = []
    for contour in contours:
        if len(contour) < 3:
            continue

        # ピクセル座標 → ワールド座標に変換
        points = []
        for pt in contour:
            col, row = pt[0]
            wx = config.origin_x + (col + 0.5) * config.resolution
            wy = config.origin_y + (row + 0.5) * config.resolution
            points.append((wx, wy))

        if len(points) < 3:
            continue

        poly = Polygon(points)
        if not poly.is_valid:
            poly = poly.buffer(0)  # 自己交差修正
        if poly.is_empty:
            continue

        # simplify で頂点数を削減
        poly = poly.simplify(config.resolution * 0.5)

        if poly.area >= min_area:
            regions.append(poly)

    return regions
