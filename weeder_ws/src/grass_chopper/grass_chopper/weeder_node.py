"""
============================================================================
草刈りロボット 障害物回避ノード (weeder_node)
============================================================================
LiDARセンサーで障害物を検知し、状態遷移マシンで回避行動を制御するROS 2ノード。

計算ロジックは obstacle_avoidance モジュールに委譲し、
このノードは ROS 2 接続 (パブリッシャー/サブスクライバー/パラメータ) のみ担当する。

状態遷移マシン:
  FORWARD     → 前方クリア: 直進
  AVOID_LEFT  → 左が空いている: 左回転回避
  AVOID_RIGHT → 右が空いている: 右回転回避
  WALL_FOLLOW → 壁沿い走行 (PD制御)
  U_TURN      → 行き止まり: Uターン

トピック:
  購読: /scan (sensor_msgs/msg/LaserScan)
  発行: /cmd_vel (geometry_msgs/msg/Twist)
============================================================================
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult

from grass_chopper.obstacle_avoidance import (
    RobotState,
    AvoidanceParams,
    ScanConfig,
    analyze_zones,
    update_state,
    compute_twist,
    deg_to_index,
    get_zone_stats,
)


class WeederNode(Node):
    """LiDARを使った障害物回避ノード"""

    def __init__(self):
        super().__init__('weeder_node')

        # --- ROS 2 パラメータ定義 ---
        self.declare_parameter(
            'safe_distance', 0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='障害物検知の安全距離 [m]'
            )
        )
        self.declare_parameter(
            'forward_speed', 0.2,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='前進速度 [m/s]'
            )
        )
        self.declare_parameter(
            'turn_speed', 0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='回転速度 [rad/s] (正の値 = 左回転)'
            )
        )
        # 壁沿い走行パラメータ
        self.declare_parameter(
            'wall_target_distance', 0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='壁との目標距離 [m]'
            )
        )
        self.declare_parameter(
            'wall_follow_kp', 1.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='壁沿い PD制御の比例ゲイン'
            )
        )
        self.declare_parameter(
            'wall_follow_kd', 0.3,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='壁沿い PD制御の微分ゲイン'
            )
        )
        self.declare_parameter(
            'wall_follow_speed', 0.15,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='壁沿い走行時の前進速度 [m/s]'
            )
        )
        self.declare_parameter(
            'angular_z_limit', 0.8,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='角速度の上限 [rad/s]'
            )
        )
        self.declare_parameter(
            'u_turn_speed', 1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Uターン時の回転速度 [rad/s]'
            )
        )

        # パラメータの初期値を取得
        self.safe_distance = self.get_parameter('safe_distance').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.wall_target_distance = self.get_parameter('wall_target_distance').value
        self.wall_follow_kp = self.get_parameter('wall_follow_kp').value
        self.wall_follow_kd = self.get_parameter('wall_follow_kd').value
        self.wall_follow_speed = self.get_parameter('wall_follow_speed').value
        self.angular_z_limit = self.get_parameter('angular_z_limit').value
        self.u_turn_speed = self.get_parameter('u_turn_speed').value

        # 動的パラメータ変更コールバック
        self.add_on_set_parameters_callback(self._on_parameter_change)

        # --- 状態遷移マシン ---
        self.state = RobotState.FORWARD
        self.prev_wall_error = 0.0  # PD制御の前回エラー

        # --- パブリッシャー: 速度指令を送信 ---
        # /cmd_vel トピックに Twist メッセージを発行
        # キューサイズ10: 最大10個のメッセージをバッファリング
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- サブスクライバー: LiDARデータを受信 ---
        # QoS (Quality of Service) 設定
        # Gazeboのセンサーデータは BEST_EFFORT で配信されるため、
        # サブスクライバー側もそれに合わせる必要がある
        # (RELIABLE にすると受信できない場合がある)
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.scan_sub = self.create_subscription(
            LaserScan,           # メッセージ型
            '/scan',             # トピック名
            self.scan_callback,  # コールバック関数
            qos_profile          # QoS設定
        )

        self.get_logger().info('草刈りロボット 障害物回避ノードを起動しました')

    def _on_parameter_change(self, params) -> SetParametersResult:
        """パラメータ動的変更時のコールバック"""
        for param in params:
            if param.name == 'safe_distance':
                self.safe_distance = param.value
            elif param.name == 'forward_speed':
                self.forward_speed = param.value
            elif param.name == 'turn_speed':
                self.turn_speed = param.value
            elif param.name == 'wall_target_distance':
                self.wall_target_distance = param.value
            elif param.name == 'wall_follow_kp':
                self.wall_follow_kp = param.value
            elif param.name == 'wall_follow_kd':
                self.wall_follow_kd = param.value
            elif param.name == 'wall_follow_speed':
                self.wall_follow_speed = param.value
            elif param.name == 'angular_z_limit':
                self.angular_z_limit = param.value
            elif param.name == 'u_turn_speed':
                self.u_turn_speed = param.value
        return SetParametersResult(successful=True)

    def _get_params(self) -> AvoidanceParams:
        """現在の ROS 2 パラメータを AvoidanceParams に集約"""
        return AvoidanceParams(
            safe_distance=self.safe_distance,
            forward_speed=self.forward_speed,
            turn_speed=self.turn_speed,
            wall_target_distance=self.wall_target_distance,
            wall_follow_kp=self.wall_follow_kp,
            wall_follow_kd=self.wall_follow_kd,
            wall_follow_speed=self.wall_follow_speed,
            angular_z_limit=self.angular_z_limit,
            u_turn_speed=self.u_turn_speed,
        )

    # --- 後方互換メソッド (既存テスト test_weeder_node.py が使用) ---

    def _deg_to_index(self, deg: float, angle_min: float, angle_increment: float) -> int:
        """角度[度]を ranges 配列のインデックスに変換 (委譲)"""
        return deg_to_index(deg, angle_min, angle_increment)

    def get_zone_stats(
        self,
        ranges: list,
        start_deg: float,
        end_deg: float,
        range_min: float,
        range_max: float,
        angle_min: float = -math.pi,
        angle_increment: float = 0.0,
    ) -> tuple:
        """指定角度範囲の最小距離と平均距離を返す (委譲)"""
        return get_zone_stats(
            ranges, start_deg, end_deg, range_min, range_max,
            angle_min, angle_increment,
        )

    def _analyze_zones(self, msg: LaserScan) -> dict:
        """LiDAR データをゾーンごとに解析する (LaserScan → ScanConfig 変換)"""
        scan_config = ScanConfig(
            angle_min=msg.angle_min,
            angle_increment=msg.angle_increment,
            range_min=msg.range_min,
            range_max=msg.range_max,
        )
        return analyze_zones(list(msg.ranges), scan_config)

    def _update_state(self, zones: dict):
        """ゾーン情報に基づいて状態を遷移させる (返り値で self.state 更新)"""
        params = self._get_params()
        new_state, wall_error_reset = update_state(self.state, zones, params)
        self.state = new_state
        if wall_error_reset:
            self.prev_wall_error = 0.0

    def _compute_twist(self, zones: dict) -> Twist:
        """現在の状態に応じた Twist メッセージを生成する (TwistCommand → Twist 変換)"""
        params = self._get_params()
        cmd, new_error = compute_twist(self.state, zones, self.prev_wall_error, params)
        self.prev_wall_error = new_error

        twist = Twist()
        twist.linear.x = cmd.linear_x
        twist.angular.z = cmd.angular_z
        return twist

    def scan_callback(self, msg: LaserScan):
        """
        LiDARスキャンデータを受信したときに呼ばれるコールバック関数
        """
        if len(msg.ranges) == 0:
            return

        zones = self._analyze_zones(msg)
        self._update_state(zones)
        twist = self._compute_twist(zones)

        if self.state != RobotState.FORWARD:
            self.get_logger().info(
                f'状態: {self.state.name} | '
                f'前方: {zones["front_min"]:.2f}m',
                throttle_duration_sec=1.0
            )

        self.cmd_pub.publish(twist)


def main(args=None):
    """ノードのエントリーポイント"""
    # ROS 2の初期化
    rclpy.init(args=args)

    # ノードを作成
    node = WeederNode()

    try:
        # ノードを実行（Ctrl+Cで停止するまでコールバックを待ち続ける）
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('ノードを停止します')
    finally:
        # クリーンアップ
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
