#!/usr/bin/env python3
"""
오른쪽 벽 추종 테스트 노드 (수직거리 변환 방식)

- 라이다 사용 범위: 정면 오른쪽 절반(0° ~ -90°) + 오른쪽(-90° ~ 원하는 뒤쪽까지)
  왼쪽 앞/왼쪽 라이다는 완전히 사용 안 함 (무시)
- 오른쪽 벽에 세 각도로 점을 찍고, 각 거리를 "벽까지의 수직거리"로 변환
- 세 수직거리가 일정 = 로봇이 벽과 평행
- 세 수직거리의 평균 = 벽까지의 실제 오프셋
- 목표 오프셋 0.45m, 앞벽 0.36m 이내면 정지
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data


class WallFollowPerp(Node):
    def __init__(self):
        super().__init__('wall_follow_perp')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # ===== 튜닝값 =====
        # --- 오른쪽 벽 세 점 각도 (도 단위, 정면=0°, 오른쪽=음수) ---
        # 세 점을 균등하게 배치 (앞 / 옆 / 뒤)
        self.ANGLES = [-60.0, -90.0, -120.0]   # 세 점 측정 각도 ★튜닝
        self.ANGLE_BAND = 3.0                  # 각 점 ±범위(도) ★튜닝

        # --- 오른쪽 벽 추종 ---
        self.TARGET_RIGHT_DIST = 0.45   # 오른쪽 벽 목표 수직거리(m) ★오프셋
        self.DIST_GAIN = 1.8            # 거리 오차 조향 게인 ★튜닝
        self.ANGLE_GAIN = 3.0           # 평행(각도) 오차 조향 게인 ★튜닝

        # --- 정면 앞벽 정지 ---
        # 유효 스캔 각도 범위: -90° ~ +FRONT_ANGLE (왼쪽 앞/왼쪽은 완전히 무시)
        self.STOP_DISTANCE = 0.36       # 앞벽 이 거리(m)면 정지 ★튜닝
        self.FRONT_ANGLE = 10.0         # 정면 감지용 ±각도 (오른쪽 절반만 사용)

        # --- 유효 스캔 각도 범위 (이 밖의 라이다는 완전히 무시) ---
        # 오른쪽 벽 뒤쪽 점까지 포함하려면 ANGLES 최솟값보다 약간 넉넉하게
        self.SCAN_MIN_DEG = -135.0      # 이 각도보다 뒤쪽은 무시
        self.SCAN_MAX_DEG = self.FRONT_ANGLE  # 이 각도보다 왼쪽(+)은 무시

        # --- 주행 ---
        self.CRUISE_SPEED = 0.15
        self.MAX_STEER = 0.42
        # 조향 부호(전진): +=왼쪽, -=오른쪽
        # ==================

        self.front_dist = 999.0
        self.perp_dists = [None] * len(self.ANGLES)  # 각 점의 벽 수직거리
        self.stopped = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("오른쪽 벽 추종 시작 (수직거리 변환 방식)")

    def _avg_at(self, msg, center, band):
        """center ± band 각도 범위의 유효 거리 평균.
        단, self.SCAN_MIN_DEG ~ self.SCAN_MAX_DEG 범위 밖은 무시."""
        vals = []
        for i, dist in enumerate(msg.ranges):
            if dist <= 0.0 or math.isinf(dist) or math.isnan(dist):
                continue
            if dist < msg.range_min or dist > msg.range_max:
                continue
            ang = math.degrees(msg.angle_min + i * msg.angle_increment)
            # 유효 스캔 범위 밖은 무시 (왼쪽 앞/왼쪽 라이다 완전 차단)
            if ang < self.SCAN_MIN_DEG or ang > self.SCAN_MAX_DEG:
                continue
            if center - band <= ang <= center + band:
                vals.append(dist)
        return (sum(vals) / len(vals)) if vals else None

    def scan_callback(self, msg):
        # 정면 앞벽 (오른쪽 절반만 사용: -FRONT_ANGLE ~ 0°)
        # 왼쪽 앞 라이다는 완전히 무시
        front_min = 999.0
        for i, dist in enumerate(msg.ranges):
            if dist <= 0.0 or math.isinf(dist) or math.isnan(dist):
                continue
            if dist < msg.range_min or dist > msg.range_max:
                continue
            ang = math.degrees(msg.angle_min + i * msg.angle_increment)
            # 정면 감지는 오른쪽 절반만 (-FRONT_ANGLE ~ 0°)
            if -self.FRONT_ANGLE <= ang <= 0.0:
                front_min = min(front_min, dist)
        self.front_dist = front_min

        # 오른쪽 벽 세 점 → 각도 offset(90°+각도)의 sin으로 수직거리 변환
        # 예: -90°는 정옆, 수직거리 = 거리 그대로 (sin(90°)=1)
        #     -60°는 앞쪽, 수직거리 = 거리 × sin(60°)
        #     -120°는 뒤쪽, 수직거리 = 거리 × sin(60°)
        # 일반화하면 수직거리 = 거리 × sin(각도의 절댓값)   (오른쪽이므로 각도는 음수)
        self.perp_dists = []
        for ang_deg in self.ANGLES:
            d = self._avg_at(msg, ang_deg, self.ANGLE_BAND)
            if d is None:
                self.perp_dists.append(None)
            else:
                perp = d * math.sin(math.radians(abs(ang_deg)))
                self.perp_dists.append(perp)

    def control_loop(self):
        if self.stopped:
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()

        # 정지: 정면 앞벽 이내
        if self.front_dist <= self.STOP_DISTANCE:
            self.stopped = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f"\n정지! 앞벽 {self.front_dist:.2f}m")
            return

        cmd.linear.x = self.CRUISE_SPEED

        # 세 점의 수직거리로 평행/거리 오차 계산
        valid = [p for p in self.perp_dists if p is not None]
        if len(valid) >= 2:
            avg_perp = sum(valid) / len(valid)
            dist_error = avg_perp - self.TARGET_RIGHT_DIST   # 거리 오차

            # 평행 오차: 앞쪽 점 수직거리 - 뒤쪽 점 수직거리
            # 평행이면 세 수직거리가 같아야 하므로 (앞 - 뒤)가 0에 가까워야 함
            # 앞쪽/뒤쪽 인덱스 자동 결정 (ANGLES에서 가장 앞각/뒤각 찾음)
            front_idx = max(range(len(self.ANGLES)), key=lambda k: self.ANGLES[k])
            back_idx = min(range(len(self.ANGLES)), key=lambda k: self.ANGLES[k])
            p_front = self.perp_dists[front_idx]
            p_back = self.perp_dists[back_idx]

            if p_front is not None and p_back is not None:
                angle_error = p_front - p_back
                # 앞쪽 수직거리가 뒤쪽보다 크면 로봇 머리가 벽에서 벌어짐 → 오른쪽으로(-)
                # 벽에서 멀면(dist_error>0) 오른쪽으로 붙기(-)
                steer = -(self.DIST_GAIN * dist_error + self.ANGLE_GAIN * angle_error)
            else:
                # 앞/뒤 중 하나만 잡히면 거리 오차만 사용
                angle_error = 0.0
                steer = -(self.DIST_GAIN * dist_error)

            steer = max(-self.MAX_STEER, min(self.MAX_STEER, steer))
            cmd.angular.z = steer
        else:
            # 오른쪽 벽 안 잡히면 직진
            avg_perp = None
            angle_error = None
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        # 상태 출력
        perp_str = " ".join(
            f"{p:.2f}" if p is not None else "N" for p in self.perp_dists
        )
        avg_str = f"{avg_perp:.2f}" if valid else "N"
        ae_str = f"{angle_error:+.3f}" if angle_error is not None else "N"
        print(
            f"수직거리[{perp_str}] 평균 {avg_str} (목표{self.TARGET_RIGHT_DIST}) "
            f"| 평행오차 {ae_str} | 앞벽 {self.front_dist:.2f} | 조향 {cmd.angular.z:+.2f}",
            end='\r'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowPerp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()