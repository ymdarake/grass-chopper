"""
============================================================================
草刈りロボット 障害物回避ノード (weeder_node)
============================================================================
LiDARセンサーで障害物を検知し、状態遷移マシンで回避行動を制御するROS 2ノード。

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
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult


class RobotState(Enum):
    """ロボットの行動状態"""
    FORWARD = auto()      # 前方クリア: 直進
    AVOID_LEFT = auto()   # 左が空いている: 左回転回避
    AVOID_RIGHT = auto()  # 右が空いている: 右回転回避
    WALL_FOLLOW = auto()  # 壁沿い走行
    U_TURN = auto()       # 行き止まり: Uターン


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

    def _deg_to_index(self, deg: float, angle_min: float, angle_increment: float) -> int:
        """
        角度[度]を ranges 配列のインデックスに変換

        Args:
            deg: 角度 [度] (-180 ~ 180)
            angle_min: LaserScan の angle_min [rad]
            angle_increment: LaserScan の angle_increment [rad]
        Returns:
            ranges 配列のインデックス
        """
        if angle_increment == 0.0:
            return 0
        rad = math.radians(deg)
        return round((rad - angle_min) / angle_increment)

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
        """
        指定角度範囲の最小距離と平均距離を返す

        Args:
            ranges: LaserScan.ranges 配列
            start_deg: ゾーン開始角度 [度] (-180 ~ 180)
            end_deg: ゾーン終了角度 [度]
            range_min: 有効最小距離
            range_max: 有効最大距離
            angle_min: LaserScan の angle_min [rad]
            angle_increment: LaserScan の angle_increment [rad] (0の場合は自動計算)
        Returns:
            (min_distance, avg_distance) のタプル。有効データなしの場合は (inf, inf)
        """
        num_samples = len(ranges)
        if num_samples == 0:
            return (float('inf'), float('inf'))

        # angle_increment が指定されていない場合は自動計算（後方互換性）
        if angle_increment == 0.0:
            angle_increment = (2 * math.pi) / num_samples

        idx1 = self._deg_to_index(start_deg, angle_min, angle_increment)
        idx2 = self._deg_to_index(end_deg, angle_min, angle_increment)
        start_idx = max(0, min(idx1, idx2))
        end_idx = min(num_samples - 1, max(idx1, idx2))

        zone_ranges = ranges[start_idx:end_idx + 1]
        valid = [
            r for r in zone_ranges
            if not math.isinf(r) and not math.isnan(r)
            and r > range_min and r < range_max
        ]

        if not valid:
            return (float('inf'), float('inf'))

        return (min(valid), sum(valid) / len(valid))

    def _analyze_zones(self, msg: LaserScan) -> dict:
        """
        LiDAR データをゾーンごとに解析する

        Returns:
            ゾーンごとの (min, avg) 辞書
        """
        r_min = msg.range_min
        r_max = msg.range_max
        ranges = msg.ranges
        a_min = msg.angle_min
        a_inc = msg.angle_increment

        front_min, front_avg = self.get_zone_stats(
            ranges, -30.0, 30.0, r_min, r_max, a_min, a_inc)
        left_min, left_avg = self.get_zone_stats(
            ranges, 60.0, 120.0, r_min, r_max, a_min, a_inc)
        right_min, right_avg = self.get_zone_stats(
            ranges, -120.0, -60.0, r_min, r_max, a_min, a_inc)

        return {
            'front_min': front_min, 'front_avg': front_avg,
            'left_min': left_min, 'left_avg': left_avg,
            'right_min': right_min, 'right_avg': right_avg,
        }

    def _update_state(self, zones: dict):
        """
        ゾーン情報に基づいて状態を遷移させる

        優先度: U_TURN > AVOID_LEFT/RIGHT > WALL_FOLLOW > FORWARD
        """
        front_min = zones['front_min']
        left_min = zones['left_min']
        right_min = zones['right_min']
        left_avg = zones['left_avg']
        right_avg = zones['right_avg']

        # 行き止まり: 三方が塞がれている → U_TURN
        if (front_min < self.safe_distance
                and left_min < self.safe_distance
                and right_min < self.safe_distance):
            self.state = RobotState.U_TURN
            return

        # 前方に障害物 → 左右を比較して回避方向を選択
        if front_min < self.safe_distance:
            if left_avg > right_avg:
                self.state = RobotState.AVOID_LEFT
            else:
                self.state = RobotState.AVOID_RIGHT
            return

        # 回避中に前方がクリアになった → FORWARD に復帰
        if self.state in (RobotState.AVOID_LEFT, RobotState.AVOID_RIGHT,
                          RobotState.U_TURN):
            if front_min > self.safe_distance * 1.2:
                self.state = RobotState.FORWARD
                return

        # WALL_FOLLOW 状態での遷移判定
        if self.state == RobotState.WALL_FOLLOW:
            # 右側に壁がなくなった → FORWARD に復帰
            if right_min > self.wall_target_distance * 2.0:
                self.state = RobotState.FORWARD
                self.prev_wall_error = 0.0
                return
            # 壁沿い継続
            return

        # FORWARD 状態で右側に壁が近い → WALL_FOLLOW に遷移
        if self.state == RobotState.FORWARD:
            if right_min < self.wall_target_distance * 1.5 and front_min >= self.safe_distance:
                self.state = RobotState.WALL_FOLLOW
                self.prev_wall_error = 0.0
                return

    def _compute_wall_follow_twist(self, right_min: float) -> Twist:
        """
        PD制御で壁沿い走行の Twist を生成

        error > 0: 壁に近すぎる → angular.z > 0 (左に曲がって離れる)
        error < 0: 壁から遠すぎる → angular.z < 0 (右に曲がって近づく)
        """
        error = self.wall_target_distance - right_min
        p_term = self.wall_follow_kp * error
        d_term = self.wall_follow_kd * (error - self.prev_wall_error)
        self.prev_wall_error = error

        twist = Twist()
        twist.linear.x = self.wall_follow_speed
        twist.angular.z = max(-self.angular_z_limit,
                              min(self.angular_z_limit, p_term + d_term))
        return twist

    def _compute_twist(self, zones: dict) -> Twist:
        """
        現在の状態に応じた Twist メッセージを生成する
        """
        twist = Twist()

        if self.state == RobotState.FORWARD:
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0
        elif self.state == RobotState.AVOID_LEFT:
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
        elif self.state == RobotState.AVOID_RIGHT:
            twist.linear.x = 0.0
            twist.angular.z = -self.turn_speed
        elif self.state == RobotState.U_TURN:
            twist.linear.x = 0.0
            twist.angular.z = self.u_turn_speed
        elif self.state == RobotState.WALL_FOLLOW:
            twist = self._compute_wall_follow_twist(zones['right_min'])

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
