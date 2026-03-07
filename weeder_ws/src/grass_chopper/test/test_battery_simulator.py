"""
============================================================================
BatterySimulator 純粋ロジック ユニットテスト
============================================================================
rclpy 不要。Mac ホスト上で pytest のみで実行可能。
============================================================================
"""

import pytest

from grass_chopper.battery_simulator import BatteryParams, BatterySimulator


# ============================================================================
# TestBatteryParams
# ============================================================================

class TestBatteryParams:
    """BatteryParams データクラステスト"""

    def test_default_values(self):
        """デフォルト値が正しい"""
        params = BatteryParams()
        assert params.capacity_ah == pytest.approx(5.0)
        assert params.voltage_full == pytest.approx(12.6)
        assert params.voltage_empty == pytest.approx(10.0)
        assert params.low_threshold == pytest.approx(0.2)
        assert params.critical_threshold == pytest.approx(0.1)

    def test_frozen(self):
        """frozen=True で変更不可"""
        params = BatteryParams()
        with pytest.raises(AttributeError):
            params.capacity_ah = 10.0


# ============================================================================
# TestBatterySimulatorInit
# ============================================================================

class TestBatterySimulatorInit:
    """BatterySimulator 初期化テスト"""

    def test_initial_full_charge(self):
        """初期状態は満充電 (100%)"""
        sim = BatterySimulator(BatteryParams())
        assert sim.get_percentage() == pytest.approx(1.0)

    def test_initial_voltage_is_full(self):
        """初期電圧は voltage_full"""
        params = BatteryParams(voltage_full=12.6)
        sim = BatterySimulator(params)
        assert sim.get_voltage() == pytest.approx(12.6)

    def test_initial_not_low(self):
        """初期状態は低バッテリーでない"""
        sim = BatterySimulator(BatteryParams())
        assert sim.is_low() is False

    def test_initial_is_full(self):
        """初期状態は満充電"""
        sim = BatterySimulator(BatteryParams())
        assert sim.is_full() is True


# ============================================================================
# TestDischarge
# ============================================================================

class TestDischarge:
    """放電テスト"""

    def test_idle_discharge(self):
        """アイドル時の放電は遅い"""
        params = BatteryParams(capacity_ah=5.0, idle_current_a=0.5)
        sim = BatterySimulator(params)
        sim.update(dt=1.0, state="idle")
        # 1 秒のアイドル: 0.5 A / (5.0 Ah * 3600) = 0.0000278
        assert sim.get_percentage() < 1.0
        assert sim.get_percentage() > 0.99

    def test_drive_discharge_faster_than_idle(self):
        """走行中の放電はアイドルより速い"""
        params = BatteryParams()
        sim_idle = BatterySimulator(params)
        sim_drive = BatterySimulator(params)

        for _ in range(100):
            sim_idle.update(dt=1.0, state="idle")
            sim_drive.update(dt=1.0, state="drive")

        assert sim_drive.get_percentage() < sim_idle.get_percentage()

    def test_mow_discharge_fastest(self):
        """草刈り中の放電が最も速い"""
        params = BatteryParams()
        sim_drive = BatterySimulator(params)
        sim_mow = BatterySimulator(params)

        for _ in range(100):
            sim_drive.update(dt=1.0, state="drive")
            sim_mow.update(dt=1.0, state="mow")

        assert sim_mow.get_percentage() < sim_drive.get_percentage()

    def test_clamp_at_zero(self):
        """バッテリー残量は 0 以下にならない"""
        params = BatteryParams(capacity_ah=0.001)  # 極小容量
        sim = BatterySimulator(params)
        for _ in range(1000):
            sim.update(dt=10.0, state="mow")
        assert sim.get_percentage() == pytest.approx(0.0)
        assert sim.get_percentage() >= 0.0


# ============================================================================
# TestCharge
# ============================================================================

class TestCharge:
    """充電テスト"""

    def test_charge_increases_percentage(self):
        """充電でバッテリー残量が回復する"""
        params = BatteryParams(capacity_ah=1.0, charge_current_a=2.0)
        sim = BatterySimulator(params)
        # まず放電
        for _ in range(500):
            sim.update(dt=1.0, state="drive")
        low_pct = sim.get_percentage()
        # 充電
        for _ in range(500):
            sim.update(dt=1.0, state="charge")
        assert sim.get_percentage() > low_pct

    def test_clamp_at_full(self):
        """充電は 100% を超えない"""
        params = BatteryParams(capacity_ah=0.001, charge_current_a=10.0)
        sim = BatterySimulator(params)
        for _ in range(100):
            sim.update(dt=10.0, state="charge")
        assert sim.get_percentage() == pytest.approx(1.0)
        assert sim.get_percentage() <= 1.0


# ============================================================================
# TestVoltage
# ============================================================================

class TestVoltage:
    """電圧テスト"""

    def test_voltage_decreases_with_discharge(self):
        """放電すると電圧が下がる"""
        params = BatteryParams(voltage_full=12.6, voltage_empty=10.0)
        sim = BatterySimulator(params)
        initial_v = sim.get_voltage()

        for _ in range(1000):
            sim.update(dt=1.0, state="drive")

        assert sim.get_voltage() < initial_v
        assert sim.get_voltage() >= 10.0

    def test_voltage_linear_interpolation(self):
        """電圧は SOC に線形補間される"""
        params = BatteryParams(voltage_full=12.0, voltage_empty=10.0)
        sim = BatterySimulator(params)
        # 50% まで放電
        sim._remaining_ah = params.capacity_ah * 0.5
        # 50% → voltage = 10.0 + 0.5 * (12.0 - 10.0) = 11.0
        assert sim.get_voltage() == pytest.approx(11.0)


# ============================================================================
# TestThresholds
# ============================================================================

class TestThresholds:
    """閾値判定テスト"""

    def test_is_low_at_threshold(self):
        """残量が low_threshold 以下で True"""
        params = BatteryParams(low_threshold=0.2)
        sim = BatterySimulator(params)
        sim._remaining_ah = params.capacity_ah * 0.19
        assert sim.is_low() is True

    def test_is_not_low_above_threshold(self):
        """残量が low_threshold より上で False"""
        params = BatteryParams(low_threshold=0.2)
        sim = BatterySimulator(params)
        sim._remaining_ah = params.capacity_ah * 0.5
        assert sim.is_low() is False

    def test_is_critical(self):
        """残量が critical_threshold 以下で True"""
        params = BatteryParams(critical_threshold=0.1)
        sim = BatterySimulator(params)
        sim._remaining_ah = params.capacity_ah * 0.05
        assert sim.is_critical() is True

    def test_is_full_after_charge(self):
        """100% で is_full() が True"""
        sim = BatterySimulator(BatteryParams())
        assert sim.is_full() is True
        # 少し放電すると False
        sim.update(dt=100.0, state="drive")
        assert sim.is_full() is False
