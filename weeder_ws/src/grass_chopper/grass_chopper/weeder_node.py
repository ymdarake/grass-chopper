"""
============================================================================
草刈りロボット 障害物回避ノード (weeder_node)
============================================================================
LiDARセンサーで前方の障害物を検知し、回避しながら前進するROS 2ノードです。

動作ロジック:
  1. LiDARの全方位スキャンデータ (LaserScan) を受信
  2. 前方±30度の範囲から最小距離を計算
  3. 最小距離が安全距離 (0.5m) 未満 → その場で左回転して回避
  4. 最小距離が安全距離以上 → 直進
  5. 速度指令 (Twist) を /cmd_vel トピックに発行

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


class WeederNode(Node):
    """LiDARを使った障害物回避ノード"""

    def __init__(self):
        super().__init__('weeder_node')

        # --- パラメータ定義 ---
        # 障害物を検知する安全距離 [メートル]
        self.safe_distance = 0.5
        # 前進速度 [m/s]
        self.forward_speed = 0.2
        # 回転速度 [rad/s] (正の値 = 左回転)
        self.turn_speed = 0.5

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

    def scan_callback(self, msg: LaserScan):
        """
        LiDARスキャンデータを受信したときに呼ばれるコールバック関数

        Args:
            msg: LaserScanメッセージ
                - ranges: 各角度での距離データの配列
                - angle_min: スキャン開始角度 [rad]
                - angle_max: スキャン終了角度 [rad]
                - range_min: 有効最小距離 [m]
                - range_max: 有効最大距離 [m]
        """
        # --- 前方の範囲を計算 ---
        # LiDARは360度スキャンするが、障害物回避には前方のデータだけ使う
        # 前方±30度 (合計60度) の範囲を確認する
        num_samples = len(msg.ranges)
        if num_samples == 0:
            return

        # 角度分解能を計算 [rad/サンプル]
        angle_increment = msg.angle_increment

        # 前方（0度）のインデックスを計算
        # angle_minからの相対角度で前方のインデックスを求める
        front_index = int(-msg.angle_min / angle_increment) if angle_increment > 0 else num_samples // 2

        # ±30度分のサンプル数を計算
        angle_range_rad = math.radians(30)  # 30度をラジアンに変換
        sample_range = int(angle_range_rad / angle_increment) if angle_increment > 0 else num_samples // 6

        # 前方範囲のインデックスを計算（配列の範囲外にならないよう調整）
        start_index = max(0, front_index - sample_range)
        end_index = min(num_samples - 1, front_index + sample_range)

        # --- 前方の最小距離を計算 ---
        # 無効な値（inf, NaN, 範囲外）を除外して最小距離を求める
        front_ranges = msg.ranges[start_index:end_index + 1]
        valid_ranges = [
            r for r in front_ranges
            if not math.isinf(r) and not math.isnan(r)
            and r > msg.range_min and r < msg.range_max
        ]

        # 有効なデータがない場合は安全とみなす
        if not valid_ranges:
            min_distance = float('inf')
        else:
            min_distance = min(valid_ranges)

        # --- 速度指令を生成 ---
        twist = Twist()

        if min_distance < self.safe_distance:
            # 障害物が近い → 停止して左回転で回避
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
            self.get_logger().info(
                f'障害物検知! 距離: {min_distance:.2f}m → 回転回避中'
            )
        else:
            # 障害物なし → 直進
            twist.linear.x = self.forward_speed
            twist.angular.z = 0.0

        # 速度指令を発行
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
