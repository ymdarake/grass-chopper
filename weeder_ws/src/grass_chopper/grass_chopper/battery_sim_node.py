"""
============================================================================
バッテリーシミュレーション ROS 2 アダプターノード (battery_sim_node)
============================================================================
BatterySimulator 純粋ロジックを ROS 2 ノードでラップする。

配信:
  - /battery_state (sensor_msgs/BatteryState): バッテリー状態

購読:
  - /cmd_vel (geometry_msgs/Twist): 走行判定 (速度 > 0 → drive)
  - /is_charging (std_msgs/Bool): 充電状態 (mission_tree_node から受信)

使い方:
  ros2 run grass_chopper battery_sim_node \
    --ros-args -p use_sim_time:=true \
    --params-file battery_params.yaml

パラメータ:
  - update_frequency (double): 更新頻度 [Hz] (default: 1.0)
  - capacity_ah (double): バッテリー容量 [Ah] (default: 5.0)
  - voltage_full (double): 満充電電圧 [V] (default: 12.6)
  - voltage_empty (double): 空電圧 [V] (default: 10.0)
  - idle_current_a (double): アイドル電流 [A] (default: 0.5)
  - drive_current_a (double): 走行電流 [A] (default: 2.0)
  - mow_current_a (double): 草刈り電流 [A] (default: 3.0)
  - charge_current_a (double): 充電電流 [A] (default: 2.0)
  - low_threshold (double): 低バッテリー閾値 (default: 0.2)
  - critical_threshold (double): クリティカル閾値 (default: 0.1)
============================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool

from grass_chopper.battery_simulator import BatteryParams, BatterySimulator


class BatterySimNode(Node):
    """バッテリーシミュレーションノード"""

    def __init__(self):
        super().__init__('battery_sim_node')

        # --- パラメータ ---
        self.declare_parameter('update_frequency', 1.0)
        self.declare_parameter('capacity_ah', 5.0)
        self.declare_parameter('voltage_full', 12.6)
        self.declare_parameter('voltage_empty', 10.0)
        self.declare_parameter('idle_current_a', 0.5)
        self.declare_parameter('drive_current_a', 2.0)
        self.declare_parameter('mow_current_a', 3.0)
        self.declare_parameter('charge_current_a', 2.0)
        self.declare_parameter('low_threshold', 0.2)
        self.declare_parameter('critical_threshold', 0.1)

        update_freq = self.get_parameter('update_frequency').value
        params = BatteryParams(
            capacity_ah=self.get_parameter('capacity_ah').value,
            voltage_full=self.get_parameter('voltage_full').value,
            voltage_empty=self.get_parameter('voltage_empty').value,
            idle_current_a=self.get_parameter('idle_current_a').value,
            drive_current_a=self.get_parameter('drive_current_a').value,
            mow_current_a=self.get_parameter('mow_current_a').value,
            charge_current_a=self.get_parameter('charge_current_a').value,
            low_threshold=self.get_parameter('low_threshold').value,
            critical_threshold=self.get_parameter('critical_threshold').value,
        )

        # --- 純粋ロジック ---
        self._sim = BatterySimulator(params)
        self._current_state = "idle"
        self._is_charging = False

        # --- /cmd_vel 購読 (走行判定) ---
        self._cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        # --- /is_charging 購読 (ミッション管理から充電状態受信) ---
        self._charging_sub = self.create_subscription(
            Bool, '/is_charging', self._charging_callback, 10)

        # --- /battery_state 配信 ---
        self._battery_pub = self.create_publisher(
            BatteryState, '/battery_state', 10)

        # --- タイマー ---
        timer_period = 1.0 / update_freq
        self._timer = self.create_timer(timer_period, self._update_callback)

        self.get_logger().info(
            f'バッテリーシミュレーション起動: '
            f'capacity={params.capacity_ah:.1f}Ah, freq={update_freq:.1f}Hz')

    def _charging_callback(self, msg: Bool):
        """充電状態を受信 (/is_charging トピック)"""
        self._is_charging = msg.data

    def _cmd_vel_callback(self, msg: Twist):
        """速度指令から走行状態を判定"""
        speed = abs(msg.linear.x) + abs(msg.angular.z)
        if speed > 0.01:
            self._current_state = "drive"
        else:
            self._current_state = "idle"

    def _update_callback(self):
        """タイマーコールバック: バッテリー状態を更新して配信"""
        state = "charge" if self._is_charging else self._current_state
        dt = 1.0 / self.get_parameter('update_frequency').value

        self._sim.update(dt, state)

        # BatteryState メッセージ作成
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = self._sim.get_voltage()
        msg.percentage = self._sim.get_percentage()
        msg.current = 0.0  # シミュレーションでは未使用
        msg.present = True

        if self._is_charging:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif self._sim.is_full():
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_FULL
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

        self._battery_pub.publish(msg)


def main(args=None):
    """ノードのエントリーポイント"""
    rclpy.init(args=args)
    node = BatterySimNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('バッテリーシミュレーションを停止します')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
