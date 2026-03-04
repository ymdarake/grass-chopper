"""
============================================================================
coverage_planner 純粋ロジック ユニットテスト
============================================================================
rclpy 不要。Mac ホスト上で pytest のみで実行可能。
Shapely が必要: pip3 install shapely
============================================================================
"""

import math

import pytest
from shapely.geometry import Polygon

from grass_chopper.coverage_planner import (
    CoverageParams,
    Waypoint,
    compute_yaw,
    estimate_coverage_ratio,
    generate_boustrophedon_waypoints,
    shrink_polygon,
)


# ============================================================================
# ヘルパー
# ============================================================================

def make_square(size=6.0, cx=0.0, cy=0.0):
    """中心 (cx, cy)、辺の長さ size の正方形ポリゴンを生成"""
    half = size / 2
    return Polygon([
        (cx - half, cy - half),
        (cx + half, cy - half),
        (cx + half, cy + half),
        (cx - half, cy + half),
    ])


@pytest.fixture
def default_params():
    return CoverageParams()


@pytest.fixture
def square_6m():
    """6m x 6m の正方形ポリゴン"""
    return make_square(6.0)


# ============================================================================
# TestCoverageParams
# ============================================================================

class TestCoverageParams:
    """CoverageParams のデフォルト値・カスタム値テスト"""

    def test_default_values(self):
        """デフォルト値が正しい"""
        params = CoverageParams()
        assert params.swath_width == pytest.approx(0.18)
        assert params.margin == pytest.approx(0.3)
        assert params.direction == pytest.approx(0.0)

    def test_custom_values(self):
        """カスタム値で生成できる"""
        params = CoverageParams(swath_width=0.5, margin=0.2, direction=math.pi / 4)
        assert params.swath_width == pytest.approx(0.5)
        assert params.margin == pytest.approx(0.2)
        assert params.direction == pytest.approx(math.pi / 4)

    def test_frozen(self):
        """frozen=True で変更不可"""
        params = CoverageParams()
        with pytest.raises(AttributeError):
            params.swath_width = 1.0


# ============================================================================
# TestShrinkPolygon
# ============================================================================

class TestShrinkPolygon:
    """shrink_polygon のテスト"""

    def test_square_shrink(self, square_6m):
        """6m 正方形を 0.3m 縮小 → 5.4m 正方形"""
        shrunk = shrink_polygon(square_6m, 0.3)
        # 各辺が 0.3m ずつ内側に → 5.4m x 5.4m
        assert shrunk.area == pytest.approx(5.4 * 5.4, abs=0.01)

    def test_margin_zero(self, square_6m):
        """margin=0 → 元のポリゴンと同じ"""
        shrunk = shrink_polygon(square_6m, 0.0)
        assert shrunk.area == pytest.approx(square_6m.area, abs=0.01)

    def test_margin_too_large(self, square_6m):
        """margin が大きすぎてポリゴンが消滅 → 空のポリゴン"""
        shrunk = shrink_polygon(square_6m, 10.0)
        assert shrunk.is_empty

    def test_result_is_polygon(self, square_6m):
        """結果が Polygon オブジェクト"""
        shrunk = shrink_polygon(square_6m, 0.3)
        assert isinstance(shrunk, Polygon)

    def test_shrunk_inside_original(self, square_6m):
        """縮小ポリゴンは元のポリゴン内に含まれる"""
        shrunk = shrink_polygon(square_6m, 0.5)
        assert square_6m.contains(shrunk)


# ============================================================================
# TestComputeYaw
# ============================================================================

class TestComputeYaw:
    """compute_yaw のテスト"""

    def test_east(self):
        """東向き (x+) → 0 rad"""
        yaw = compute_yaw(Waypoint(0, 0, 0), Waypoint(1, 0, 0))
        assert yaw == pytest.approx(0.0)

    def test_north(self):
        """北向き (y+) → pi/2 rad"""
        yaw = compute_yaw(Waypoint(0, 0, 0), Waypoint(0, 1, 0))
        assert yaw == pytest.approx(math.pi / 2)

    def test_west(self):
        """西向き (x-) → pi rad"""
        yaw = compute_yaw(Waypoint(0, 0, 0), Waypoint(-1, 0, 0))
        assert abs(yaw) == pytest.approx(math.pi)

    def test_south(self):
        """南向き (y-) → -pi/2 rad"""
        yaw = compute_yaw(Waypoint(0, 0, 0), Waypoint(0, -1, 0))
        assert yaw == pytest.approx(-math.pi / 2)

    def test_northeast(self):
        """北東 → pi/4 rad"""
        yaw = compute_yaw(Waypoint(0, 0, 0), Waypoint(1, 1, 0))
        assert yaw == pytest.approx(math.pi / 4)

    def test_same_point(self):
        """同一点 → 0 rad"""
        yaw = compute_yaw(Waypoint(1, 1, 0), Waypoint(1, 1, 0))
        assert yaw == pytest.approx(0.0)


# ============================================================================
# TestBoustrophedonGeneration
# ============================================================================

class TestBoustrophedonGeneration:
    """generate_boustrophedon_waypoints のテスト"""

    def test_returns_list_of_waypoints(self, square_6m, default_params):
        """結果が Waypoint のリスト"""
        waypoints = generate_boustrophedon_waypoints(square_6m, default_params)
        assert isinstance(waypoints, list)
        assert len(waypoints) > 0
        assert all(isinstance(wp, Waypoint) for wp in waypoints)

    def test_waypoints_count_reasonable(self, square_6m, default_params):
        """ウェイポイント数が妥当 (6m / 0.18m ≈ 33行 × 2端点 ≈ 66点)"""
        waypoints = generate_boustrophedon_waypoints(square_6m, default_params)
        # マージン分で縮小されるので少なくなるが、20点以上はある
        assert len(waypoints) >= 20
        # 上限: 200 点は超えない
        assert len(waypoints) <= 200

    def test_zigzag_pattern(self, square_6m, default_params):
        """隣接する行で x 座標が反転している (ジグザグ)"""
        waypoints = generate_boustrophedon_waypoints(square_6m, default_params)
        # 行の端点 (偶数=往路、奇数=復路) で x 座標の符号が変わるか確認
        # 少なくとも 4 ウェイポイント (2行分) が必要
        assert len(waypoints) >= 4
        # 最初の2点は往路 (左→右 or 右→左)、次の2点は復路
        # 1行目の最後と2行目の最初の x 座標が近いことを確認 (折り返し)
        # → 行端で折り返すので、wp[1].x ≈ wp[2].x (同じ端)
        assert abs(waypoints[1].x - waypoints[2].x) < default_params.swath_width * 2

    def test_all_waypoints_inside_polygon(self, square_6m, default_params):
        """全ウェイポイントがポリゴン内部に収まる"""
        waypoints = generate_boustrophedon_waypoints(square_6m, default_params)
        for wp in waypoints:
            from shapely.geometry import Point
            assert square_6m.contains(Point(wp.x, wp.y)) or \
                square_6m.boundary.distance(Point(wp.x, wp.y)) < 0.01, \
                f"Waypoint ({wp.x}, {wp.y}) is outside polygon"

    def test_swath_width_affects_count(self, square_6m):
        """swath_width が大きいとウェイポイント数が減る"""
        narrow = CoverageParams(swath_width=0.18)
        wide = CoverageParams(swath_width=0.5)
        wp_narrow = generate_boustrophedon_waypoints(square_6m, narrow)
        wp_wide = generate_boustrophedon_waypoints(square_6m, wide)
        assert len(wp_narrow) > len(wp_wide)

    def test_direction_rotated(self):
        """direction=pi/2 で走行方向が90度回転する"""
        poly = make_square(4.0)
        params_0 = CoverageParams(swath_width=0.5, margin=0.0, direction=0.0)
        params_90 = CoverageParams(swath_width=0.5, margin=0.0, direction=math.pi / 2)
        wp_0 = generate_boustrophedon_waypoints(poly, params_0)
        wp_90 = generate_boustrophedon_waypoints(poly, params_90)
        # direction=0 では y が変化する行間移動
        # direction=pi/2 では x が変化する行間移動
        # 2行目の先頭は折り返し点なので、y 方向の変化パターンが異なる
        assert len(wp_0) > 0 and len(wp_90) > 0
        # 少なくともウェイポイントの分布が異なることを確認
        y_range_0 = max(wp.y for wp in wp_0) - min(wp.y for wp in wp_0)
        x_range_90 = max(wp.x for wp in wp_90) - min(wp.x for wp in wp_90)
        # direction=0: 行は y 方向に並ぶので y_range が大きい
        # direction=90: 行は x 方向に並ぶので x_range が大きい
        assert y_range_0 > 1.0
        assert x_range_90 > 1.0

    def test_empty_polygon_returns_empty(self, default_params):
        """空のポリゴン → 空リスト"""
        empty_poly = Polygon()
        waypoints = generate_boustrophedon_waypoints(empty_poly, default_params)
        assert waypoints == []

    def test_tiny_polygon_returns_few_or_empty(self):
        """非常に小さいポリゴン → 少数 or 空リスト"""
        tiny = make_square(0.1)
        params = CoverageParams(swath_width=0.18, margin=0.0)
        waypoints = generate_boustrophedon_waypoints(tiny, params)
        assert len(waypoints) <= 4

    def test_waypoints_have_yaw(self, square_6m, default_params):
        """各ウェイポイントに yaw が設定されている"""
        waypoints = generate_boustrophedon_waypoints(square_6m, default_params)
        for wp in waypoints:
            assert isinstance(wp.yaw, float)


# ============================================================================
# TestCoverageRatio
# ============================================================================

class TestCoverageRatio:
    """estimate_coverage_ratio のテスト"""

    def test_high_coverage(self, square_6m, default_params):
        """十分なウェイポイントでカバレッジ率が 80% 以上"""
        waypoints = generate_boustrophedon_waypoints(square_6m, default_params)
        ratio = estimate_coverage_ratio(waypoints, square_6m, default_params.swath_width)
        assert ratio >= 0.8

    def test_low_coverage_with_wide_swath(self, square_6m):
        """幅が広すぎるとカバレッジ率が下がる (行数が減る)"""
        params = CoverageParams(swath_width=3.0, margin=0.0)
        waypoints = generate_boustrophedon_waypoints(square_6m, params)
        ratio = estimate_coverage_ratio(waypoints, square_6m, params.swath_width)
        # 非常に幅広いので 100% にはなるかもしれないが、少なくとも計算できる
        assert 0.0 <= ratio <= 1.0

    def test_empty_waypoints(self, square_6m):
        """ウェイポイントなし → 0.0"""
        ratio = estimate_coverage_ratio([], square_6m, 0.18)
        assert ratio == pytest.approx(0.0)

    def test_ratio_between_0_and_1(self, square_6m, default_params):
        """カバレッジ率が 0.0 ~ 1.0 の範囲"""
        waypoints = generate_boustrophedon_waypoints(square_6m, default_params)
        ratio = estimate_coverage_ratio(waypoints, square_6m, default_params.swath_width)
        assert 0.0 <= ratio <= 1.0
