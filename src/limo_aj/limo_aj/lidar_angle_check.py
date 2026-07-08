import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LidarAngleCheckNode(Node):
    def __init__(self):
        super().__init__('lidar_angle_check_node')
        
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, lidar_qos)
        self.last_log_time = 0.0
        self.get_logger().info("🔎 LIMO 진짜 정면 각도 판별 노드가 가동되었습니다.")

    def scan_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_log_time < 0.4:
            return
        self.last_log_time = current_time

        # 전 영역에서 가장 가까운 물체의 index와 거리를 찾습니다.
        closest_dist = float('inf')
        closest_angle = 0.0

        for index, distance in enumerate(msg.ranges):
            if math.isnan(distance) or math.isinf(distance) or distance < msg.range_min or distance > msg.range_max:
                continue
                
            # 드라이버가 계산하는 수식 그대로 도출
            angle_rad = msg.angle_min + index * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            if distance < closest_dist:
                closest_dist = distance
                closest_angle = angle_deg

        print("\n" + "="*50)
        print(f"🎯 [현재 가장 가까운 물체 정보]")
        print(f"  - 감지된 최단 거리: {closest_dist:.2f}m")
        print(f"  - 🚨 감지된 실제 각도: {closest_angle:.1f}°")
        print("="*50)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAngleCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()