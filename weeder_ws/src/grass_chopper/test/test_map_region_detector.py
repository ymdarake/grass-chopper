"""
============================================================================
MapRegionDetector 純粋ロジック ユニットテスト
============================================================================
rclpy 不要。Mac ホスト上で pytest のみで実行可能。
NumPy + Shapely + OpenCV が必要。
============================================================================
"""

import numpy as np
import pytest
from shapely.geometry import Polygon

from grass_chopper.coverage_tracker import GridConfig
from grass_chopper.map_region_detector import (
    detect_obstacles_from_map,
    extract_free_regions,
    occupancy_grid_to_binary,
    select_largest_region,
)


# ============================================================================
# ヘルパー
# ============================================================================

def make_config(width=20, height=20, resolution=0.5, origin_x=-5.0, origin_y=-5.0):
    """テスト用 GridConfig を生成 (10m x 10m)"""
    return GridConfig(
        width=width, height=height, resolution=resolution,
        origin_x=origin_x, origin_y=origin_y,
    )


# ============================================================================
# TestOccupancyGridToBinary
# ============================================================================

class TestOccupancyGridToBinary:
    """occupancy_grid_to_binary: OccupancyGrid → 二値画像変換"""

    def test_free_cells_become_white(self):
        """free セル (0) は 255 (白) になる"""
        config = make_config(4, 4, 1.0)
        grid = np.zeros(16, dtype=np.int8)  # 全て free (0)
        binary = occupancy_grid_to_binary(grid, config, free_threshold=50)
        assert binary.shape == (4, 4)
        assert np.all(binary == 255)

    def test_occupied_cells_become_black(self):
        """occupied セル (100) は 0 (黒) になる"""
        config = make_config(4, 4, 1.0)
        grid = np.full(16, 100, dtype=np.int8)  # 全て occupied
        binary = occupancy_grid_to_binary(grid, config, free_threshold=50)
        assert np.all(binary == 0)

    def test_unknown_cells_become_black(self):
        """unknown セル (-1) は 0 (黒) になる"""
        config = make_config(4, 4, 1.0)
        grid = np.full(16, -1, dtype=np.int8)  # 全て unknown
        binary = occupancy_grid_to_binary(grid, config, free_threshold=50)
        assert np.all(binary == 0)


# ============================================================================
# TestExtractFreeRegions
# ============================================================================

class TestExtractFreeRegions:
    """extract_free_regions: 二値画像から走行可能領域を抽出"""

    def test_square_room(self):
        """正方形の部屋 → 1 つのポリゴン"""
        # 20x20 グリッド、外周 2 セルが壁 (0)、内部が free (255)
        binary = np.zeros((20, 20), dtype=np.uint8)
        binary[2:18, 2:18] = 255  # 内部 16x16 が free
        config = make_config(20, 20, 0.5)  # 10m x 10m

        regions = extract_free_regions(binary, config, min_area=1.0, robot_radius=0.0)
        assert len(regions) >= 1
        # 最大領域の面積チェック (16x16 * 0.25 = 64 m²)
        largest = max(regions, key=lambda p: p.area)
        assert largest.area > 30.0  # モルフォロジーで少し縮小される

    def test_room_with_hole(self):
        """穴 (内部障害物) ありの部屋"""
        binary = np.zeros((20, 20), dtype=np.uint8)
        binary[2:18, 2:18] = 255  # 内部が free
        binary[8:12, 8:12] = 0    # 中央に穴 (4x4)
        config = make_config(20, 20, 0.5)

        regions = extract_free_regions(binary, config, min_area=1.0, robot_radius=0.0)
        assert len(regions) >= 1

    def test_multiple_rooms(self):
        """2 つの分離した部屋 → 2 つのポリゴン"""
        binary = np.zeros((20, 30), dtype=np.uint8)
        binary[2:18, 2:12] = 255   # 左の部屋
        binary[2:18, 18:28] = 255  # 右の部屋
        config = GridConfig(width=30, height=20, resolution=0.5,
                            origin_x=0.0, origin_y=0.0)

        regions = extract_free_regions(binary, config, min_area=1.0, robot_radius=0.0)
        assert len(regions) == 2

    def test_min_area_filter(self):
        """小さすぎる領域は min_area でフィルタ"""
        binary = np.zeros((20, 20), dtype=np.uint8)
        binary[5:7, 5:7] = 255  # 2x2 = 0.5 m² (resolution=0.5 → 4 cells * 0.25 = 1.0 m²)
        config = make_config(20, 20, 0.5)

        regions = extract_free_regions(binary, config, min_area=5.0, robot_radius=0.0)
        assert len(regions) == 0

    def test_erode_shrinks_region(self):
        """robot_radius > 0 で領域が縮小される"""
        # 大きなグリッドで erode 効果を明確にする
        binary = np.zeros((40, 40), dtype=np.uint8)
        binary[4:36, 4:36] = 255  # 内部 32x32 が free
        config = GridConfig(width=40, height=40, resolution=0.25,
                            origin_x=0.0, origin_y=0.0)

        regions_no_erode = extract_free_regions(
            binary, config, min_area=1.0, robot_radius=0.0)
        regions_erode = extract_free_regions(
            binary, config, min_area=1.0, robot_radius=0.5)

        assert len(regions_no_erode) >= 1
        assert len(regions_erode) >= 1
        area_no = max(r.area for r in regions_no_erode)
        area_yes = max(r.area for r in regions_erode)
        assert area_yes < area_no

    def test_empty_binary_returns_empty(self):
        """全て壁 (0) → 空リスト"""
        binary = np.zeros((20, 20), dtype=np.uint8)
        config = make_config(20, 20, 0.5)

        regions = extract_free_regions(binary, config, min_area=1.0, robot_radius=0.0)
        assert regions == []


# ============================================================================
# TestDetectObstaclesFromMap
# ============================================================================

class TestDetectObstaclesFromMap:
    """detect_obstacles_from_map: 地図から障害物領域を抽出"""

    def test_obstacles_detected(self):
        """free 領域内の occupied ブロック → 障害物ポリゴン"""
        # free の中に occupied ブロックを配置
        grid = np.zeros(400, dtype=np.int8)  # 20x20 全て free
        config = make_config(20, 20, 0.5)
        # 中央 4x4 セルを occupied に
        grid_2d = grid.reshape((20, 20))
        grid_2d[8:12, 8:12] = 100
        grid_flat = grid_2d.flatten()

        obstacles = detect_obstacles_from_map(grid_flat, config, min_area=0.1)
        assert len(obstacles) >= 1
        for obs in obstacles:
            assert isinstance(obs, Polygon)
            assert obs.area > 0.1

    def test_no_obstacles(self):
        """全て free → 障害物なし"""
        grid = np.zeros(400, dtype=np.int8)
        config = make_config(20, 20, 0.5)

        obstacles = detect_obstacles_from_map(grid, config, min_area=0.1)
        assert obstacles == []


# ============================================================================
# TestSelectLargestRegion
# ============================================================================

class TestSelectLargestRegion:
    """select_largest_region: 最大面積のポリゴンを選択"""

    def test_selects_largest(self):
        """複数ポリゴンから最大面積を選択"""
        small = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])   # 1 m²
        large = Polygon([(0, 0), (5, 0), (5, 5), (0, 5)])   # 25 m²
        medium = Polygon([(0, 0), (3, 0), (3, 3), (0, 3)])  # 9 m²

        result = select_largest_region([small, large, medium])
        assert result is not None
        assert result.area == pytest.approx(25.0)

    def test_empty_list_returns_none(self):
        """空リスト → None"""
        result = select_largest_region([])
        assert result is None
