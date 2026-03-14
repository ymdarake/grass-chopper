"""
============================================================================
傾斜検知 ROS 2 ノード (incline_monitor_node)
============================================================================
IMU データ (/imu) を購読し、傾斜レベルを判定する。
EMERGENCY 時は /cmd_vel にゼロ速度を発行して緊急停止する。

Humble Object パターン:
  - 純粋ロジック: incline_monitor.py (quaternion_to_rpy, evaluate_incline, should_stop)
  - ROS 2 アダプター: 本ファイル (Subscribe /imu, Publish /cmd_vel)
============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

from grass_chopper.incline_monitor import (
    InclineLevel,
    quaternion_to_rpy,
    evaluate_incline,
    should_stop,
)


class InclineMonitorNode(Node):
    """IMU データから傾斜を監視し、緊急停止する ROS 2 ノード"""

    def __init__(self):
        super().__init__('incline_monitor_node')

        # --- パラメータ ---
        self.declare_parameter('warning_deg', 15.0)
        self.declare_parameter('emergency_deg', 25.0)
        self.declare_parameter('check_frequency', 10.0)

        self._warning_deg = self.get_parameter('warning_deg').value
        self._emergency_deg = self.get_parameter('emergency_deg').value
        check_freq = self.get_parameter('check_frequency').value

        # --- 状態 ---
        self._last_level = InclineLevel.SAFE
        self._latest_imu = None

        # --- Subscriber: IMU データ ---
        # Gazebo センサーは BEST_EFFORT QoS
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._imu_sub = self.create_subscription(
            Imu, '/imu', self._imu_callback, sensor_qos)

        # --- Publisher: 緊急停止用 /cmd_vel_incline ---
        # twist_mux 経由で最優先で統合される (priority: 30)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_incline', 10)

        # --- タイマー: 定期的に傾斜チェック ---
        period = 1.0 / check_freq
        self._timer = self.create_timer(period, self._check_incline)

        self.get_logger().info(
            f'傾斜検知ノード起動: 警告={self._warning_deg}°, '
            f'緊急={self._emergency_deg}°, 周期={check_freq}Hz')

    def _imu_callback(self, msg: Imu):
        """IMU データを保存"""
        self._latest_imu = msg

    def _check_incline(self):
        """最新の IMU データから傾斜レベルを判定"""
        if self._latest_imu is None:
            return

        # 四元数 → RPY
        q = self._latest_imu.orientation
        roll, pitch, _yaw = quaternion_to_rpy(q.x, q.y, q.z, q.w)

        # 傾斜レベル判定
        level = evaluate_incline(
            roll, pitch,
            warning_deg=self._warning_deg,
            emergency_deg=self._emergency_deg,
        )

        # レベル変化時にログ出力
        if level != self._last_level:
            if level == InclineLevel.EMERGENCY:
                self.get_logger().error(
                    f'傾斜緊急停止: roll={roll:.2f}rad, pitch={pitch:.2f}rad')
            elif level == InclineLevel.WARNING:
                self.get_logger().warn(
                    f'傾斜警告: roll={roll:.2f}rad, pitch={pitch:.2f}rad')
            elif level == InclineLevel.SAFE:
                self.get_logger().info('傾斜: 安全レベルに復帰')
            self._last_level = level

        # 緊急停止
        if should_stop(level):
            stop_cmd = Twist()  # 全フィールド 0.0
            self._cmd_vel_pub.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = InclineMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
