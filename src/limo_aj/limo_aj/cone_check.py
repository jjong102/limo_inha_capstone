import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class LidarDualPeakNode(Node):
    def __init__(self):
        super().__init__('lidar_dual_peak_node')
        
        # ===== [유저 제안 기반 듀얼 피크 파라미터] =====
        self.CONE_DETECT_DIST = 1.20       # 탐색 기준 거리 1.2m
        self.SEARCH_MAX_ANGLE = 80.0       # 🎯 데드존 없이 전방 ±60도 통짜 탐색
        self.PEAK_SEPARATION_DEG = 15.0    # 두 피크가 동일한 콘 파편이 되지 않도록 떨어뜨릴 최소 각도
        # ===============================================

        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, lidar_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("🎯 [듀얼 피크 모드] 데드존 제거 / ±60° Top 2 피크 검출 노드가 가동되었습니다.")

    def scan_callback(self, msg):
        valid_peaks = []

        # [단계 1] ±60도 영역 안에서 1.2m 이내의 모든 유효 데이터 수집
        for index, distance in enumerate(msg.ranges):
            if math.isnan(distance) or math.isinf(distance) or distance < msg.range_min or distance > msg.range_max:
                continue
            
            if distance > self.CONE_DETECT_DIST:
                continue

            angle_deg = math.degrees(msg.angle_min + index * msg.angle_increment)

            if -self.SEARCH_MAX_ANGLE <= angle_deg <= self.SEARCH_MAX_ANGLE:
                valid_peaks.append({'dist': distance, 'angle': angle_deg})

        # 거리가 가장 가까운 순서(오름차순)로 정렬
        valid_peaks.sort(key=lambda x: x['dist'])

        # [단계 2] 서로 다른 독립된 물체(라바콘) 2개 추출하기
        peak1 = None  # 1등 피크 (가장 가까운 물체)
        peak2 = None  # 2등 피크 (그다음 가까운 독립된 물체)

        if len(valid_peaks) > 0:
            peak1 = valid_peaks[0]
            
            # 1등 피크와 같은 라바콘 덩어리가 아닌, 최소 15도 이상 떨어진 진짜 '다른' 물체를 2등으로 선정
            for p in valid_peaks[1:]:
                if abs(p['angle'] - peak1['angle']) >= self.PEAK_SEPARATION_DEG:
                    peak2 = p
                    break

        # [단계 3] 검출된 피크들을 기반으로 최종 회피 판정
        if peak1 is None:
            decision = "🟢 [통로 클리어] 60도 이내에 물체 없음 -> 🚀 직진"
        else:
            p1_str = f"{peak1['dist']:.2f}m({peak1['angle']:+.1f}°)"
            p2_str = f"{peak2['dist']:.2f}m({peak2['angle']:+.1f}°)" if peak2 else "없음"
            
            # 제일 위험한 1등 피크(가장 가까운 물체)의 각도를 보고 회피 방향 결정
            # 각도가 + 이면 내 기준 왼쪽에 콘이 밀고 들어온 것 -> 우측 회피
            # 각도가 - 이면 내 기준 오른쪽에 콘이 밀고 들어온 것 -> 좌측 회피
            if peak1['angle'] > 0:
                decision = f"🚨 [피크 감지] 1등:{p1_str} | 2등:{p2_str} -> 👉 우측 회피"
            else:
                decision = f"🚨 [피크 감지] 1등:{p1_str} | 2등:{p2_str} -> 👈 좌측 회피"

        print(f"{decision}                                                       ", end='\r')
        self.cmd_vel_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = LidarDualPeakNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()