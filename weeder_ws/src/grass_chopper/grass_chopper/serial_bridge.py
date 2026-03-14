"""
============================================================================
シリアルブリッジ 純粋計算ロジック (serial_bridge)
============================================================================
ROS 2 / MicroPython に依存しない純粋関数モジュール。
Mac ホスト上で pytest のみでテスト実行可能。

Pi 5 ↔ Pico 間の通信状態管理、Watchdog タイマー、
差動駆動オドメトリの計算を担当する。

serial_bridge_node.py (ROS 2 アダプター) から使用される。
============================================================================
"""

import math


class BridgeState:
    """
    Pi ↔ Pico 間の通信状態管理

    DISCONNECTED → (データ受信) → CONNECTED → (タイムアウト) → DISCONNECTED
    """

    def __init__(self):
        self.is_connected: bool = False
        self.last_recv_time: float | None = None

    def on_receive(self, timestamp: float):
        """データ受信時に呼ぶ"""
        self.is_connected = True
        self.last_recv_time = timestamp

    def on_timeout(self):
        """タイムアウト時に呼ぶ"""
        self.is_connected = False


class WatchdogTimer:
    """
    通信途絶検出タイマー

    feed() で生存通知し、is_timed_out() でタイムアウトを検出する。
    Pico との UART 通信が途絶えた場合にモーターを停止するために使用。

    Args:
        timeout_sec: タイムアウト時間 [秒]
    """

    def __init__(self, timeout_sec: float = 0.5):
        self._timeout_sec = timeout_sec
        self._last_feed_time: float | None = None

    def feed(self, timestamp: float):
        """生存通知 (データ受信のたびに呼ぶ)"""
        self._last_feed_time = timestamp

    def is_timed_out(self, current_time: float) -> bool:
        """タイムアウトしているか判定する"""
        if self._last_feed_time is None:
            return True  # 一度も feed されていない → 安全側でタイムアウト扱い
        return (current_time - self._last_feed_time) > self._timeout_sec


def compute_odometry_delta(
    left_vel: float,
    right_vel: float,
    wheel_separation: float,
    dt: float,
) -> tuple[float, float, float]:
    """
    差動駆動オドメトリの増分を計算する

    Args:
        left_vel: 左車輪の線速度 [m/s] (wheel_radius * angular_vel)
        right_vel: 右車輪の線速度 [m/s]
        wheel_separation: 車輪間距離 [m]
        dt: 時間ステップ [s]

    Returns:
        (dx, dy, dtheta): ロボット座標系での移動量 [m, m, rad]
    """
    if dt <= 0:
        return 0.0, 0.0, 0.0

    if wheel_separation <= 1e-6:
        return 0.0, 0.0, 0.0

    # 車体中心の線速度と角速度
    linear_vel = (left_vel + right_vel) / 2.0
    angular_vel = (right_vel - left_vel) / wheel_separation

    # ロボット座標系での移動量
    dtheta = angular_vel * dt
    if abs(dtheta) < 1e-6:
        # ほぼ直進 (特異点回避)
        dx = linear_vel * dt
        dy = 0.0
    else:
        # 円弧移動
        radius = linear_vel / angular_vel
        dx = radius * math.sin(dtheta)
        dy = radius * (1.0 - math.cos(dtheta))

    return dx, dy, dtheta


class OdometryAccumulator:
    """
    オドメトリの累積計算

    差動駆動の左右車輪速度から、ワールド座標系での
    ロボット位置 (x, y, theta) を累積的に計算する。

    Args:
        wheel_separation: 車輪間距離 [m]
    """

    def __init__(self, wheel_separation: float = 0.24):
        self._wheel_separation = wheel_separation
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0

    def update(self, left_vel: float, right_vel: float, dt: float):
        """
        オドメトリを更新する

        Args:
            left_vel: 左車輪の線速度 [m/s]
            right_vel: 右車輪の線速度 [m/s]
            dt: 時間ステップ [s]
        """
        dx, dy, dtheta = compute_odometry_delta(
            left_vel, right_vel, self._wheel_separation, dt)

        # ロボット座標系 → ワールド座標系に変換
        cos_th = math.cos(self.theta)
        sin_th = math.sin(self.theta)
        self.x += dx * cos_th - dy * sin_th
        self.y += dx * sin_th + dy * cos_th
        self.theta += dtheta
        # [-π, π] に正規化
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

    def reset(self):
        """位置をリセットする"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
