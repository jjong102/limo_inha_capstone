import math  # 라디안과 각도 변환을 위해 사용한다.
import rclpy  # ROS2 Python 라이브러리를 가져온다.
from rclpy.node import Node  # ROS2 노드 클래스를 가져온다.
from sensor_msgs.msg import LaserScan  # /scan 메시지 타입을 가져온다.
from geometry_msgs.msg import Twist  # /cmd_vel 메시지 타입을 가져온다.
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# ===== 주행 튜닝 파라미터 =====
MAX_STEER = 0.42  # LIMO Ackermann 조향 제한값이다.
DRIVE_SPEED = 0.12  # 라바콘 회피 주행 속도 [m/s] 이다.

# 회피를 위해 꺾을 조향값 설정 (맵 곡률에 맞춰 조절 가능)
AVOID_LEFT_STEER = 0.28   # Type B일 때 왼쪽 길로 가기 위한 조향값이다.
AVOID_RIGHT_STEER = -0.28  # Type A일 때 오른쪽 길로 가기 위한 조향값이다.

# 라바콘 판별을 위한 측면 감지 각도 범위 (정면 기준)
LEFT_ANGLE_MIN = 5.0
LEFT_ANGLE_MAX = 30.0

RIGHT_ANGLE_MIN = -30.0
RIGHT_ANGLE_MAX = -5.0
# =============================


def clamp_steer(value):  # 조향 제한 함수이다.
    return max(-MAX_STEER, min(MAX_STEER, value))  # 값을 -0.42 ~ 0.42로 제한한다.


class LidarConeAvoid(Node):
    def __init__(self):
        super().__init__('camera_line_follow')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 1. 라이다 센서용 QoS 프로필 생성 (Incompatible QoS 해결)
        lidar_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT, 
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # 2. 라이다 스캔 구독 설정
        self.scan_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            lidar_qos
        )
        self.get_logger().info("D구간 라바콘 회피 주행 오리지널 노드가 기동되었습니다.")

    def scan_callback(self, msg):
        # 왼쪽과 오른쪽 영역에서 가장 가까운 장애물까지의 거리를 저장할 변수
        min_left_dist = float('inf')
        min_right_dist = float('inf')

        angle_increment = msg.angle_increment

        # 1. 라이다 전체 데이터를 돌며 좌/우 특정 각도 영역의 최소 거리 계산
        for index, distance in enumerate(msg.ranges):
            # 유효하지 않은 거리 데이터 정보는 패스
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle_rad = msg.angle_min + index * angle_increment
            angle_deg = math.degrees(angle_rad)

            # [전방 왼쪽 영역 체크] (5도 ~ 30도 사이)
            if LEFT_ANGLE_MIN <= angle_deg <= LEFT_ANGLE_MAX:
                if distance < min_left_dist:
                    min_left_dist = distance

            # [전방 오른쪽 영역 체크] (-30도 ~ -5도 사이)
            elif RIGHT_ANGLE_MIN <= angle_deg <= RIGHT_ANGLE_MAX:
                if distance < min_right_dist:
                    min_right_dist = distance

        cmd = Twist()

        # 2. 복귀 기능이 완전히 제거된 오리지널 2단계 회피 판별 로직
        if min_left_dist < min_right_dist:
            # [Type A 상황] 왼쪽에 라바콘이 더 가까움 -> 우회 주행 (우회전)
            steer_target = AVOID_RIGHT_STEER
            cmd.linear.x = DRIVE_SPEED
            self.get_logger().info(f"🚧 [Type A] 왼쪽에 라바콘 발견! 우회 회피 중... (Left_Dist: {min_left_dist:.2f}m)")
        else:
            # [Type B 상황] 오른쪽에 라바콘이 더 가까움 -> 좌측 회피 (좌회전)
            steer_target = AVOID_LEFT_STEER
            cmd.linear.x = DRIVE_SPEED
            self.get_logger().info(f"🚧 [Type B] 오른쪽에 라바콘 발견! 좌측 회피 중... (Right_Dist: {min_right_dist:.2f}m)")

        cmd.angular.z = clamp_steer(steer_target)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarConeAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()