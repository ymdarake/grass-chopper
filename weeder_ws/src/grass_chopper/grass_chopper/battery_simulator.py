"""
============================================================================
バッテリーシミュレーション 純粋計算ロジック (battery_simulator)
============================================================================
ROS 2 に依存しない純粋関数モジュール。
Mac ホスト上で pytest のみでテスト実行可能。

バッテリー残量をシミュレーションし、電圧・SOC・閾値判定を提供する。
battery_sim_node.py (ROS 2 アダプター) から使用される。
============================================================================
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class BatteryParams:
    """バッテリーシミュレーションのパラメータ"""
    capacity_ah: float = 5.0         # バッテリー容量 [Ah]
    voltage_full: float = 12.6       # 満充電電圧 [V]
    voltage_empty: float = 10.0      # 空電圧 [V]
    idle_current_a: float = 0.5      # アイドル時の消費電流 [A]
    drive_current_a: float = 2.0     # 走行時の消費電流 [A]
    mow_current_a: float = 3.0      # 草刈り時の消費電流 [A]
    charge_current_a: float = 2.0    # 充電電流 [A]
    low_threshold: float = 0.2       # 低バッテリー閾値 (0.0~1.0)
    critical_threshold: float = 0.1  # クリティカル閾値 (0.0~1.0)


# 状態名 → 消費電流のマッピング用定数
_CURRENT_MAP = {
    "idle": "idle_current_a",
    "drive": "drive_current_a",
    "mow": "mow_current_a",
}


class BatterySimulator:
    """
    バッテリー残量シミュレーター

    クーロンカウント方式で残量を追跡する。
    update() を一定間隔で呼び出して放電/充電をシミュレーションする。

    Args:
        params: バッテリーパラメータ
    """

    def __init__(self, params: BatteryParams):
        self._params = params
        self._remaining_ah = params.capacity_ah  # 初期は満充電

    def update(self, dt: float, state: str) -> None:
        """
        バッテリー状態を dt 秒分更新する

        Args:
            dt: 経過時間 [秒]
            state: ロボットの状態 ("idle", "drive", "mow", "charge")
        """
        if state == "charge":
            # 充電: 残量が増加
            charge_ah = self._params.charge_current_a * dt / 3600.0
            self._remaining_ah = min(
                self._remaining_ah + charge_ah, self._params.capacity_ah)
        else:
            # 放電: 残量が減少
            current_attr = _CURRENT_MAP.get(state, "idle_current_a")
            current_a = getattr(self._params, current_attr)
            discharge_ah = current_a * dt / 3600.0
            self._remaining_ah = max(self._remaining_ah - discharge_ah, 0.0)

    def get_percentage(self) -> float:
        """バッテリー残量を返す (0.0 ~ 1.0)"""
        if self._params.capacity_ah <= 0:
            return 0.0
        return self._remaining_ah / self._params.capacity_ah

    def get_voltage(self) -> float:
        """現在の電圧を返す (SOC に線形補間)"""
        soc = self.get_percentage()
        return (self._params.voltage_empty
                + soc * (self._params.voltage_full - self._params.voltage_empty))

    def is_low(self) -> bool:
        """バッテリーが低い (low_threshold 以下) かどうか"""
        return self.get_percentage() <= self._params.low_threshold

    def is_critical(self) -> bool:
        """バッテリーがクリティカル (critical_threshold 以下) かどうか"""
        return self.get_percentage() <= self._params.critical_threshold

    def is_full(self) -> bool:
        """バッテリーが満充電 (>= 99.9%) かどうか"""
        return self.get_percentage() >= 0.999
