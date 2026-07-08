import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarPedestrianStopGo(Node):
    """
    A 구간 보행자 Stop & Go 노드.

    동작 흐름:
      1) 출발하면 평상시 속도로 직진한다.
      2) 전방 cone 안에서 보행자(장애물)가 연속으로 감지되면 정지한다.
      3) 보행자가 완전히 지나가(전방이 연속으로 비어) 안정되면 다시 출발한다.
    """

    def __init__(self):
        super().__init__('lidar_pedestrian_stop_go')

        # ---------------- 파라미터 (대회 트랙에 맞게 튜닝) ----------------
        self.declare_parameter('detection_distance', 0.7)   # 감지 거리 (m)
        self.declare_parameter('detection_angle_deg', 20.0)  # 정면 기준 좌우 각도 (±deg)
        self.declare_parameter('cruise_speed', 0.2)          # 평상시 전진 속도 (m/s)
        self.declare_parameter('stop_frames', 3)             # 이만큼 연속 감지되면 정지
        self.declare_parameter('clear_frames', 5)            # 이만큼 연속 비어있으면 재출발

        self.DETECTION_DISTANCE = self.get_parameter('detection_distance').value
        self.DETECTION_ANGLE = self.get_parameter('detection_angle_deg').value
        self.CRUISE_SPEED = self.get_parameter('cruise_speed').value
        self.STOP_FRAMES = self.get_parameter('stop_frames').value
        self.CLEAR_FRAMES = self.get_parameter('clear_frames').value

        # ---------------- 상태 변수 ----------------
        self.is_stopped = False     # 현재 정지 상태인지
        self.detect_count = 0       # 연속 감지 프레임 카운터
        self.clear_count = 0        # 연속 클리어 프레임 카운터

        # ---------------- 퍼블리셔 / 서브스크라이버 ----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # ★ 핵심 수정: 라이다는 BEST_EFFORT로 발행되므로 센서용 QoS로 구독해야 데이터가 들어온다.
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f'Stop&Go 시작 | 감지거리={self.DETECTION_DISTANCE}m '
            f'각도=±{self.DETECTION_ANGLE}deg 속도={self.CRUISE_SPEED}m/s'
        )

    def is_obstacle_in_front(self, msg):
        """전방 cone 안에서 감지거리 이내의 유효 포인트가 있으면 True."""
        for index, distance in enumerate(msg.ranges):
            # inf / nan / 센서 유효범위 밖 값은 무시
            if math.isinf(distance) or math.isnan(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle_deg = math.degrees(msg.angle_min + index * msg.angle_increment)

            # 정면 기준 좌우 cone 안만 확인
            if -self.DETECTION_ANGLE <= angle_deg <= self.DETECTION_ANGLE:
                if distance < self.DETECTION_DISTANCE:
                    return True
        return False

    def scan_callback(self, msg):
        obstacle = self.is_obstacle_in_front(msg)

        # 연속 프레임 카운터 갱신 (노이즈 한두 프레임에 흔들리지 않게)
        if obstacle:
            self.detect_count += 1
            self.clear_count = 0
        else:
            self.clear_count += 1
            self.detect_count = 0

        # ---------------- 상태 전환 ----------------
        if not self.is_stopped and self.detect_count >= self.STOP_FRAMES:
            # 주행 중 → 보행자 연속 감지 → 정지
            self.is_stopped = True
            self.get_logger().warn('보행자 감지 -> 정지')
        elif self.is_stopped and self.clear_count >= self.CLEAR_FRAMES:
            # 정지 중 → 보행자 완전 통과 → 재출발
            self.is_stopped = False
            self.get_logger().warn('보행자 통과 완료 -> 재출발')

        # ---------------- 명령 발행 ----------------
        cmd = Twist()
        cmd.linear.x = 0.0 if self.is_stopped else self.CRUISE_SPEED
        cmd.angular.z = 0.0   # 직진 (조향/라인추종은 별도 노드 담당)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPedestrianStopGo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 로봇을 안전하게 정지
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()