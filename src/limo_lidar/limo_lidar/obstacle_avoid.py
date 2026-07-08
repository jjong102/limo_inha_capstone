import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(value, low, high):
    return max(low, min(high, value))


class LidarObstacleAvoid(Node):
    """
    라이다 기반 박스 회피 노드.

    동작 흐름:
      1) 평소에는 평상시 속도로 직진한다.
      2) 전방 cone 안에서 박스가 감지되면, 박스가 있는 쪽의 '반대'로
         핸들을 틀어 옆으로 비켜 간다 (멈추지 않음).
      3) 전방이 다시 비면 핸들을 풀고 직진으로 복귀한다.
    """

    def __init__(self):
        super().__init__('lidar_obstacle_avoid')

        # ---------------- 파라미터 (트랙에 맞게 튜닝) ----------------
        self.declare_parameter('detect_distance', 0.5)    # 회피 시작 거리 (m)
        self.declare_parameter('front_angle_deg', 25.0)   # 전방 판단 cone (±deg)
        self.declare_parameter('cruise_speed', 0.2)       # 평상시 전진 속도 (m/s)
        self.declare_parameter('avoid_speed', 0.15)       # 회피 중 전진 속도 (m/s, 조금 느리게)
        self.declare_parameter('turn_strength', 0.4)      # 회피 조향 세기 (rad/s 부호 포함 크기)
        self.declare_parameter('max_steer', 0.42)         # 조향 안전 제한
        self.declare_parameter('clear_frames', 5)         # 이만큼 연속 비면 직진 복귀

        self.DETECT_DIST = self.get_parameter('detect_distance').value
        self.FRONT_ANGLE = self.get_parameter('front_angle_deg').value
        self.CRUISE_SPEED = self.get_parameter('cruise_speed').value
        self.AVOID_SPEED = self.get_parameter('avoid_speed').value
        self.TURN = self.get_parameter('turn_strength').value
        self.MAX_STEER = self.get_parameter('max_steer').value
        self.CLEAR_FRAMES = self.get_parameter('clear_frames').value

        # ---------------- 상태 변수 ----------------
        self.is_avoiding = False    # 현재 회피 중인지
        self.avoid_dir = 0.0        # 회피 조향 방향 (+: 좌회전, -: 우회전)
        self.clear_count = 0        # 연속 클리어 프레임 카운터

        # ---------------- 퍼블리셔 / 서브스크라이버 ----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f'장애물 회피 시작 | 감지거리={self.DETECT_DIST}m '
            f'전방각=±{self.FRONT_ANGLE}deg 속도={self.CRUISE_SPEED}m/s'
        )

    def scan_front(self, msg):
        """
        전방 cone을 좌/우로 나눠서, 양쪽에서 가장 가까운 유효 거리를 구한다.
        반환: (정면에 박스 있나?, 왼쪽 최소거리, 오른쪽 최소거리)
        """
        left_min = float('inf')    # 정면 기준 왼쪽(양의 각도)
        right_min = float('inf')   # 정면 기준 오른쪽(음의 각도)
        obstacle = False

        for index, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle_deg = math.degrees(msg.angle_min + index * msg.angle_increment)

            # 전방 cone 밖은 무시
            if not (-self.FRONT_ANGLE <= angle_deg <= self.FRONT_ANGLE):
                continue

            # 감지 거리 이내면 박스가 있는 것으로 판단
            if distance < self.DETECT_DIST:
                obstacle = True

            # 좌/우 최소거리 갱신 (어느 쪽이 더 막혔는지 보기 위함)
            if angle_deg >= 0.0:
                left_min = min(left_min, distance)
            else:
                right_min = min(right_min, distance)

        return obstacle, left_min, right_min

    def scan_callback(self, msg):
        obstacle, left_min, right_min = self.scan_front(msg)

        cmd = Twist()

        if obstacle:
            # 회피 시작/유지: 더 가까운(=막힌) 쪽의 반대로 핸들을 튼다.
            self.clear_count = 0
            if not self.is_avoiding:
                # 박스가 왼쪽에 더 가까우면 오른쪽으로(-), 오른쪽이면 왼쪽으로(+)
                if left_min < right_min:
                    self.avoid_dir = -self.TURN   # 우회전
                else:
                    self.avoid_dir = +self.TURN   # 좌회전
                self.is_avoiding = True
                side = '오른쪽' if self.avoid_dir < 0 else '왼쪽'
                self.get_logger().warn(f'박스 감지 -> {side}으로 회피')

            cmd.linear.x = self.AVOID_SPEED
            cmd.angular.z = clamp(self.avoid_dir, -self.MAX_STEER, self.MAX_STEER)

        else:
            # 전방이 비었음. 연속으로 충분히 비면 직진 복귀.
            self.clear_count += 1
            if self.is_avoiding and self.clear_count >= self.CLEAR_FRAMES:
                self.is_avoiding = False
                self.avoid_dir = 0.0
                self.get_logger().warn('회피 완료 -> 직진 복귀')

            if self.is_avoiding:
                # 아직 복귀 판정 전: 회피 조향 유지하며 통과
                cmd.linear.x = self.AVOID_SPEED
                cmd.angular.z = clamp(self.avoid_dir, -self.MAX_STEER, self.MAX_STEER)
            else:
                # 평상시 직진
                cmd.linear.x = self.CRUISE_SPEED
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()