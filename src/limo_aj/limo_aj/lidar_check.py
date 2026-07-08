import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LidarCheckNode(Node):
    def __init__(self):
        super().__init__('lidar_check_node')
        
        # 좌측 검사 구역 파라미터
        self.declare_parameter('left_angle_min', 5.0)
        self.declare_parameter('left_angle_max', 35.0)
        
        # 우측 검사 구역 파라미터
        self.declare_parameter('right_angle_min', -35.0)
        self.declare_parameter('right_angle_max', -5.0)
        
        # 라이다 QoS 설정
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, lidar_qos)
        self.last_log_time = 0.0
        self.get_logger().info("🔍 좌/우측 경로 라바콘 거리 비교 노드가 기동되었습니다.")

    def scan_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        # 0.5초마다 출력
        if current_time - self.last_log_time < 0.5:
            return
        self.last_log_time = current_time

        # 실시간 파라미터 값 읽기
        l_min = self.get_parameter('left_angle_min').get_parameter_value().double_value
        l_max = self.get_parameter('left_angle_max').get_parameter_value().double_value
        r_min = self.get_parameter('right_angle_min').get_parameter_value().double_value
        r_max = self.get_parameter('right_angle_max').get_parameter_value().double_value

        # 각 구역별 최소 거리 초기화
        left_zone_dist = float('inf')
        right_zone_dist = float('inf')

        for index, distance in enumerate(msg.ranges):
            # 유효하지 않은 데이터 필터링
            if math.isnan(distance) or math.isinf(distance) or distance < msg.range_min or distance > msg.range_max:
                continue
                
            # 현재 데이터의 실제 각도 계산 (Degree)
            angle_rad = msg.angle_min + index * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            # 1. 좌측 영역 검사
            if l_min <= angle_deg <= l_max:
                if distance < left_zone_dist:
                    left_zone_dist = distance

            # 2. 우측 영역 검사
            if r_min <= angle_deg <= r_max:
                if distance < right_zone_dist:
                    right_zone_dist = distance

        # 3. 좌/우측 거리 차이 계산
        diff_str = "N/A (양쪽 모두 감지되어야 계산됨)"
        closer_side = "없음"
        
        if left_zone_dist != float('inf') and right_zone_dist != float('inf'):
            diff = abs(left_zone_dist - right_zone_dist)
            diff_str = f"{diff:.2f}m"
            if left_zone_dist < right_zone_dist:
                closer_side = f"👈 좌측이 {diff:.2f}m 더 가까움"
            elif right_zone_dist < left_zone_dist:
                closer_side = f"👉 우측이 {diff:.2f}m 더 가까움"
            else:
                closer_side = "⚖️ 좌우 거리가 동일함"
        elif left_zone_dist != float('inf'):
            closer_side = "👈 좌측 구역만 감지됨"
        elif right_zone_dist != float('inf'):
            closer_side = "👉 우측 구역만 감지됨"

        # 터미널 리포트 출력
        print("\n" + "="*60)
        print(f"📐 [설정된 감시 영역]")
        print(f"  - 👈 좌측 경로 범위: {l_min}° ~ {l_max}°")
        print(f"  - 👉 우측 경로 범위: {r_min}° ~ {r_max}°")
        print("-"*60)
        print(f"📊 [실시간 구역별 최소 거리 결과]")
        print(f"  - 👈 좌측 최소 거리: {self.fmt_dist(left_zone_dist)}")
        print(f"  - 👉 우측 최소 거리: {self.fmt_dist(right_zone_dist)}")
        print(f"  - 📂 두 경로의 거리 차이: {diff_str}")
        print(f"  - 🚨 판단 결과: {closer_side}")
        print("="*60)

    def fmt_dist(self, val):
        return f"{val:.2f}m" if val != float('inf') else "inf (장애물 없음)"


def main(args=None):
    rclpy.init(args=args)
    node = LidarCheckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()