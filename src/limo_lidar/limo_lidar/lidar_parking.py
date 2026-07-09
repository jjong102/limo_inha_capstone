#!/usr/bin/env python3
"""
오른쪽 벽 추종(수직거리 변환) + 주차 시퀀스 통합 노드 (단일 노드 버전)

[Phase 1] 오른쪽 벽과 오프셋 유지 → 앞벽 0.36m에서 정지
[Phase 2] 같은 노드의 같은 퍼블리셔로 즉시 주차 시퀀스 실행
          (노드/퍼블리셔를 새로 만들지 않으므로 DDS discovery 지연 없음)

- Phase 1 튜닝: __init__ 안의 "Phase 1 튜닝값" 영역
- Phase 2 튜닝: 파일 상단의 STEPS / 속도 상수
"""

import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data


# ============================================================
# [Phase 2 튜닝 영역] 주차 시퀀스
# ============================================================

LINEAR_SPEED = 0.15       # 전진/후진 시 선속도 (m/s)
STEER_SPEED  = 0.6        # 조향 angular.z 절대값
PUBLISH_RATE_HZ = 20.0    # cmd_vel publish 주기

STEPS = [
    # direction, steer_sign,   duration
    ('REV',      -1,           5.3),
    ('REV',      +1,           1.25),
    ('FWD',      +1,           1.0),
    ('REV',      +1,           1.0),
    ('FWD',      +1,           1.0),
    ('REV',      +1,           0.8),
    ('FWD',      +1,           0.8),
]

# ============================================================


class WallFollowAndPark(Node):
    def __init__(self):
        super().__init__('wall_follow_and_park')

        # 퍼블리셔 하나로 Phase 1, 2 모두 사용 (discovery 지연 제거)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # ===== Phase 1 튜닝값 =====
        # --- 오른쪽 벽 세 점 각도 ---
        self.ANGLES = [-60.0, -90.0, -120.0]
        self.ANGLE_BAND = 3.0

        # --- 오른쪽 벽 추종 ---
        self.TARGET_RIGHT_DIST = 0.5
        self.DIST_GAIN = 2.2
        self.ANGLE_GAIN = 4.0

        # --- 오차 연동 속도 조절 ---
        self.ERR_LOW = 0.03
        self.ERR_HIGH = 0.15
        self.CORRECT_SPEED = 0.06

        # --- 정면 앞벽 정지 ---
        self.STOP_DISTANCE = 0.34
        self.FRONT_ANGLE = 10.0

        # --- 유효 스캔 범위 ---
        self.SCAN_MIN_DEG = -135.0
        self.SCAN_MAX_DEG = self.FRONT_ANGLE

        # --- 주행 ---
        self.CRUISE_SPEED = 0.15
        self.MAX_STEER = 0.42
        # ==================

        self.front_dist = 999.0
        self.perp_dists = [None] * len(self.ANGLES)
        self.stopped = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Phase 1: 오른쪽 벽 추종 시작")

    # ──────────────────────────────────────────────
    # Phase 1: 벽 추종 (원본 로직 그대로)
    # ──────────────────────────────────────────────

    def _avg_at(self, msg, center, band):
        vals = []
        for i, dist in enumerate(msg.ranges):
            if dist <= 0.0 or math.isinf(dist) or math.isnan(dist):
                continue
            if dist < msg.range_min or dist > msg.range_max:
                continue
            ang = math.degrees(msg.angle_min + i * msg.angle_increment)
            if ang < self.SCAN_MIN_DEG or ang > self.SCAN_MAX_DEG:
                continue
            if center - band <= ang <= center + band:
                vals.append(dist)
        return (sum(vals) / len(vals)) if vals else None

    def scan_callback(self, msg):
        front_min = 999.0
        for i, dist in enumerate(msg.ranges):
            if dist <= 0.0 or math.isinf(dist) or math.isnan(dist):
                continue
            if dist < msg.range_min or dist > msg.range_max:
                continue
            ang = math.degrees(msg.angle_min + i * msg.angle_increment)
            if -self.FRONT_ANGLE <= ang <= 0.0:
                front_min = min(front_min, dist)
        self.front_dist = front_min

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
            return

        cmd = Twist()

        if self.front_dist <= self.STOP_DISTANCE:
            self.stopped = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f"\n정지! 앞벽 {self.front_dist:.2f}m")
            return

        cmd.linear.x = self.CRUISE_SPEED

        valid = [p for p in self.perp_dists if p is not None]
        if len(valid) >= 2:
            avg_perp = sum(valid) / len(valid)
            dist_error = avg_perp - self.TARGET_RIGHT_DIST

            front_idx = max(range(len(self.ANGLES)), key=lambda k: self.ANGLES[k])
            back_idx = min(range(len(self.ANGLES)), key=lambda k: self.ANGLES[k])
            p_front = self.perp_dists[front_idx]
            p_back = self.perp_dists[back_idx]

            if p_front is not None and p_back is not None:
                angle_error = p_front - p_back
                steer = -(self.DIST_GAIN * dist_error + self.ANGLE_GAIN * angle_error)
            else:
                angle_error = 0.0
                steer = -(self.DIST_GAIN * dist_error)

            steer = max(-self.MAX_STEER, min(self.MAX_STEER, steer))
            cmd.angular.z = steer

            combined_err = abs(dist_error) + abs(angle_error)
            if combined_err <= self.ERR_LOW:
                cmd.linear.x = self.CRUISE_SPEED
            elif combined_err >= self.ERR_HIGH:
                cmd.linear.x = self.CORRECT_SPEED
            else:
                ratio = (combined_err - self.ERR_LOW) / (self.ERR_HIGH - self.ERR_LOW)
                cmd.linear.x = self.CRUISE_SPEED - ratio * (self.CRUISE_SPEED - self.CORRECT_SPEED)
        else:
            avg_perp = None
            angle_error = None
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        perp_str = " ".join(f"{p:.2f}" if p is not None else "N" for p in self.perp_dists)
        avg_str = f"{avg_perp:.2f}" if valid else "N"
        ae_str = f"{angle_error:+.3f}" if angle_error is not None else "N"
        print(
            f"수직거리[{perp_str}] 평균 {avg_str} (목표{self.TARGET_RIGHT_DIST}) "
            f"| 평행오차 {ae_str} | 앞벽 {self.front_dist:.2f} | 조향 {cmd.angular.z:+.2f} 속도 {cmd.linear.x:.2f}",
            end='\r'
        )

    # ──────────────────────────────────────────────
    # Phase 2: 주차 시퀀스 (같은 퍼블리셔 사용)
    # ──────────────────────────────────────────────

    def _publish_twist(self, linear_x, angular_z, duration):
        """같은 self.cmd_pub으로 duration(초) 동안 publish"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        dt = 1.0 / PUBLISH_RATE_HZ
        steps = int(duration / dt)
        for _ in range(steps):
            self.cmd_pub.publish(msg)
            time.sleep(dt)

    def _stop_brief(self):
        self._publish_twist(0.0, 0.0, 0.3)

    def run_parking_sequence(self):
        self.get_logger().info('Phase 2: 주차 시퀀스 시작')

        for i, (direction, steer_sign, duration) in enumerate(STEPS, start=1):
            linear_x = LINEAR_SPEED if direction == 'FWD' else -LINEAR_SPEED
            angular_z = steer_sign * STEER_SPEED

            self.get_logger().info(
                f'{i}) {"전진" if direction == "FWD" else "후진"} + '
                f'{"좌" if steer_sign > 0 else "우"} 조향 ({duration}s)'
            )
            self._publish_twist(linear_x, angular_z, duration)

        self._stop_brief()
        self.get_logger().info('Phase 2: 주차 시퀀스 종료')


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowAndPark()

    try:
        # Phase 1: 벽 추종 (stopped 될 때까지)
        while rclpy.ok() and not node.stopped:
            rclpy.spin_once(node, timeout_sec=0.1)

        # Phase 2: 즉시 주차 시퀀스 (같은 노드, 같은 퍼블리셔)
        if rclpy.ok():
            node.run_parking_sequence()

    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()