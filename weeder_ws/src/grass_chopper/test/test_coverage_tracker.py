"""
============================================================================
CoverageTracker 純粋ロジック ユニットテスト
============================================================================
rclpy 不要。Mac ホスト上で pytest のみで実行可能。
NumPy + Shapely が必要: pip3 install numpy shapely
============================================================================
"""

import math

import numpy as np
import pytest
from shapely.geometry import Polygon

from grass_chopper.coverage_tracker import (
    CoverageTracker,
    GridConfig,
    detect_uncovered_regions,
)


# ============================================================================
# ヘルパー
# ============================================================================

def make_config(width=10, height=10, resolution=1.0, origin_x=0.0, origin_y=0.0):
    """テスト用 GridConfig を生成"""
    return GridConfig(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
    )


# ============================================================================
# TestGridConfig
# ============================================================================

class TestGridConfig:
    """GridConfig のデータクラステスト"""

    def test_creation(self):
        """正常に生成できる"""
        config = GridConfig(width=20, height=15, resolution=0.1,
                            origin_x=-1.0, origin_y=-1.5)
        assert config.width == 20
        assert config.height == 15
        assert config.resolution == pytest.approx(0.1)
        assert config.origin_x == pytest.approx(-1.0)
        assert config.origin_y == pytest.approx(-1.5)

    def test_frozen(self):
        """frozen=True で変更不可"""
        config = make_config()
        with pytest.raises(AttributeError):
            config.width = 99


# ============================================================================
# TestCoverageTrackerInit
# ============================================================================

class TestCoverageTrackerInit:
    """CoverageTracker 初期化テスト"""

    def test_initial_grid_all_zero(self):
        """初期状態ではグリッドが全て 0 (未踏破)"""
        tracker = CoverageTracker(make_config(5, 5, 1.0))
        grid = tracker.to_occupancy_grid()
        assert grid.shape == (5, 5)
        assert np.all(grid == 0)

    def test_initial_coverage_ratio_zero(self):
        """初期状態のカバレッジ率は 0.0"""
        tracker = CoverageTracker(make_config(10, 10, 0.5))
        assert tracker.get_coverage_ratio() == pytest.approx(0.0)

    def test_with_target_polygon(self):
        """target_polygon 指定時、ポリゴン外はカバレッジ対象外"""
        config = make_config(10, 10, 1.0, origin_x=0.0, origin_y=0.0)
        # 5m×5m の正方形 (グリッドの左下半分)
        polygon = Polygon([(0, 0), (5, 0), (5, 5), (0, 5)])
        tracker = CoverageTracker(config, target_polygon=polygon)
        # ポリゴン内のセルのみがターゲット
        assert tracker.get_coverage_ratio() == pytest.approx(0.0)


# ============================================================================
# TestMarkCovered
# ============================================================================

class TestMarkCovered:
    """mark_covered のテスト"""

    def test_single_mark(self):
        """1点をマークするとカバレッジ率が 0 より大きくなる"""
        config = make_config(10, 10, 1.0)
        tracker = CoverageTracker(config)
        tracker.mark_covered(5.0, 5.0, tool_radius=1.0)
        assert tracker.get_coverage_ratio() > 0.0

    def test_mark_center_of_grid(self):
        """グリッド中央をマークすると該当セルが 100 になる"""
        config = make_config(10, 10, 1.0)
        tracker = CoverageTracker(config)
        tracker.mark_covered(5.5, 5.5, tool_radius=0.4)
        grid = tracker.to_occupancy_grid()
        # セル (5, 5) が 100 (踏破済み) になっている
        assert grid[5, 5] == 100

    def test_mark_outside_grid_ignored(self):
        """グリッド外の座標をマークしてもエラーにならない"""
        config = make_config(10, 10, 1.0)
        tracker = CoverageTracker(config)
        # グリッド外
        tracker.mark_covered(-100.0, -100.0, tool_radius=1.0)
        tracker.mark_covered(100.0, 100.0, tool_radius=1.0)
        assert tracker.get_coverage_ratio() == pytest.approx(0.0)

    def test_mark_boundary(self):
        """グリッド境界付近のマークでエラーにならない"""
        config = make_config(10, 10, 1.0)
        tracker = CoverageTracker(config)
        # 境界ギリギリ
        tracker.mark_covered(0.0, 0.0, tool_radius=0.5)
        tracker.mark_covered(10.0, 10.0, tool_radius=0.5)
        # エラーなく実行できればOK

    def test_tool_radius_affects_area(self):
        """tool_radius が大きいほど広い領域がカバーされる"""
        config = make_config(20, 20, 1.0)
        tracker_small = CoverageTracker(config)
        tracker_large = CoverageTracker(config)

        tracker_small.mark_covered(10.0, 10.0, tool_radius=1.0)
        tracker_large.mark_covered(10.0, 10.0, tool_radius=5.0)

        assert tracker_large.get_coverage_ratio() > tracker_small.get_coverage_ratio()

    def test_multiple_marks_accumulate(self):
        """複数回のマークでカバレッジ率が累積する"""
        config = make_config(10, 10, 1.0)
        tracker = CoverageTracker(config)

        tracker.mark_covered(2.0, 2.0, tool_radius=1.0)
        ratio1 = tracker.get_coverage_ratio()

        tracker.mark_covered(8.0, 8.0, tool_radius=1.0)
        ratio2 = tracker.get_coverage_ratio()

        assert ratio2 > ratio1

    def test_double_mark_same_spot_no_change(self):
        """同じ場所を2回マークしてもカバレッジ率は変わらない"""
        config = make_config(10, 10, 1.0)
        tracker = CoverageTracker(config)

        tracker.mark_covered(5.0, 5.0, tool_radius=1.0)
        ratio1 = tracker.get_coverage_ratio()

        tracker.mark_covered(5.0, 5.0, tool_radius=1.0)
        ratio2 = tracker.get_coverage_ratio()

        assert ratio2 == pytest.approx(ratio1)


# ============================================================================
# TestCoverageRatio
# ============================================================================

class TestCoverageRatio:
    """get_coverage_ratio のテスト"""

    def test_full_coverage(self):
        """全セルをカバーするとカバレッジ率が 1.0"""
        config = make_config(4, 4, 1.0)
        tracker = CoverageTracker(config)
        # 大きな半径で全セルをカバー
        tracker.mark_covered(2.0, 2.0, tool_radius=10.0)
        assert tracker.get_coverage_ratio() == pytest.approx(1.0)

    def test_ratio_between_0_and_1(self):
        """カバレッジ率が 0.0 ~ 1.0 の範囲"""
        config = make_config(10, 10, 1.0)
        tracker = CoverageTracker(config)
        tracker.mark_covered(5.0, 5.0, tool_radius=2.0)
        ratio = tracker.get_coverage_ratio()
        assert 0.0 < ratio < 1.0

    def test_ratio_with_target_polygon(self):
        """target_polygon 指定時、ポリゴン内のみのカバレッジ率"""
        # 10x10 グリッド (0,0)-(10,10)
        config = make_config(10, 10, 1.0, origin_x=0.0, origin_y=0.0)
        # ターゲット: 左下 5x5 の領域
        polygon = Polygon([(0, 0), (5, 0), (5, 5), (0, 5)])
        tracker = CoverageTracker(config, target_polygon=polygon)

        # ポリゴン内の点をマーク
        tracker.mark_covered(2.5, 2.5, tool_radius=10.0)
        ratio = tracker.get_coverage_ratio()
        # ポリゴン内のセルは全てカバーされているはず
        assert ratio >= 0.9


# ============================================================================
# TestUncoveredCells
# ============================================================================

class TestUncoveredCells:
    """get_uncovered_cells のテスト"""

    def test_initial_all_uncovered(self):
        """初期状態では全セルが未踏破"""
        config = make_config(5, 5, 1.0)
        tracker = CoverageTracker(config)
        uncovered = tracker.get_uncovered_cells()
        assert uncovered.shape == (5, 5)
        assert np.all(uncovered)  # 全て True (未踏破)

    def test_covered_cells_are_false(self):
        """カバー済みセルは False"""
        config = make_config(5, 5, 1.0)
        tracker = CoverageTracker(config)
        tracker.mark_covered(2.5, 2.5, tool_radius=10.0)
        uncovered = tracker.get_uncovered_cells()
        assert not np.any(uncovered)  # 全て False (踏破済み)


# ============================================================================
# TestOccupancyGrid
# ============================================================================

class TestOccupancyGrid:
    """to_occupancy_grid のテスト"""

    def test_dtype_is_int8(self):
        """出力が int8 配列"""
        config = make_config(5, 5, 1.0)
        tracker = CoverageTracker(config)
        grid = tracker.to_occupancy_grid()
        assert grid.dtype == np.int8

    def test_values_0_or_100(self):
        """値は 0 (未踏破) または 100 (踏破済み)"""
        config = make_config(5, 5, 1.0)
        tracker = CoverageTracker(config)
        tracker.mark_covered(2.5, 2.5, tool_radius=1.0)
        grid = tracker.to_occupancy_grid()
        unique_values = set(np.unique(grid))
        assert unique_values.issubset({0, 100})

    def test_grid_shape(self):
        """グリッド形状が (height, width)"""
        config = make_config(width=8, height=6, resolution=0.5)
        tracker = CoverageTracker(config)
        grid = tracker.to_occupancy_grid()
        assert grid.shape == (6, 8)


# ============================================================================
# TestDetectUncoveredRegions
# ============================================================================

class TestDetectUncoveredRegions:
    """detect_uncovered_regions のテスト"""

    def test_all_covered_returns_empty(self):
        """全セルカバー済み → 空リスト"""
        config = make_config(10, 10, 1.0)
        grid = np.full((10, 10), 100, dtype=np.int8)
        regions = detect_uncovered_regions(grid, config)
        assert regions == []

    def test_all_uncovered_returns_one_region(self):
        """全セル未カバー → 1 つの大きなポリゴン"""
        config = make_config(10, 10, 1.0)
        grid = np.zeros((10, 10), dtype=np.int8)
        regions = detect_uncovered_regions(grid, config, min_area=0.5)
        assert len(regions) >= 1
        # 合計面積がグリッド面積に近い
        total_area = sum(r.area for r in regions)
        assert total_area > 50.0  # 10x10 = 100 m² の半分以上

    def test_corners_uncovered(self):
        """四隅が未カバー → 複数ポリゴン検出"""
        config = make_config(20, 20, 0.5)  # 10m x 10m
        grid = np.full((20, 20), 100, dtype=np.int8)
        # 四隅を未カバーに (各 3x3 セル)
        grid[0:3, 0:3] = 0    # 左下
        grid[0:3, 17:20] = 0  # 右下
        grid[17:20, 0:3] = 0  # 左上
        grid[17:20, 17:20] = 0  # 右上
        regions = detect_uncovered_regions(grid, config, min_area=0.1)
        assert len(regions) == 4

    def test_noise_filtered_by_min_area(self):
        """小さなノイズ (1セル) は min_area でフィルタ"""
        config = make_config(20, 20, 0.5)  # 10m x 10m
        grid = np.full((20, 20), 100, dtype=np.int8)
        # 1 セルだけ未カバー (0.25 m²)
        grid[10, 10] = 0
        regions = detect_uncovered_regions(grid, config, min_area=0.5)
        assert regions == []

    def test_returns_shapely_polygons(self):
        """結果が Shapely Polygon のリスト"""
        config = make_config(10, 10, 1.0)
        grid = np.zeros((10, 10), dtype=np.int8)
        regions = detect_uncovered_regions(grid, config)
        for region in regions:
            assert isinstance(region, Polygon)
            assert region.is_valid
            assert not region.is_empty

    def test_strip_uncovered(self):
        """帯状の未カバー領域を検出"""
        config = make_config(20, 20, 0.5)  # 10m x 10m
        grid = np.full((20, 20), 100, dtype=np.int8)
        # 中央の横帯を未カバーに (行 9-11, 全幅)
        grid[9:12, :] = 0
        regions = detect_uncovered_regions(grid, config, min_area=0.5)
        assert len(regions) >= 1
        # 帯の面積: 3 rows × 20 cols × 0.25 m²/cell = 15 m²
        # (モルフォロジー + simplify で縮小される)
        total_area = sum(r.area for r in regions)
        assert total_area > 5.0
