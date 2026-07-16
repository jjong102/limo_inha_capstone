#!/usr/bin/env python3
"""
오른쪽 벽 추종(수직거리 변환) + 주차 시퀀스 통합 노드 (mission_parking_node 테스트 변형)

mission_parking_node.py와 동일하되 두 가지가 다르다:
  1. /parking_start 대기 없이 노드 실행 즉시 Phase 1을 시작한다 (단독 테스트용 —
     mission_manager_node의 warm standby 흐름과 무관).
  2. 오른쪽 벽 추종 목표 거리(TARGET_RIGHT_DIST)가 Phase 1 시작 후
     STAGE1_DURATION_SEC 동안은 STAGE1_TARGET_RIGHT_DIST, 그 이후부터는
     STAGE2_TARGET_RIGHT_DIST로 바뀌는 2단계 구조다 (아래 STAGE1_* 상수 참고).

[Phase 1] 오른쪽 벽과 오프셋 유지, CRUISE_SPEED(1.0m/s)로 접근하다가 앞벽이
          DECEL_DISTANCE 이내로 들어오면 DECEL_MIN_SPEED까지 선형 감속 →
          앞벽 STOP_DISTANCE(0.34m)에서 정지
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
    ('REV',      -1,           4.6),
    ('REV',      +1,           0.8),
    ('FWD',      +1,           0.9),
    ('REV',      +1,           0.9),
    ('FWD',      +1,           0.9),
    ('REV',      +1,           0.8),
    ('FWD',      +1,           0.8),
]

# ============================================================


class MissionParkingTestNode(Node):
    def __init__(self):
        super().__init__('mission_parking_test_node')

        # 퍼블리셔 하나로 Phase 1, 2 모두 사용 (discovery 지연 제거)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # ===== Phase 1 튜닝값 =====
        # --- 오른쪽 벽 세 점 각도 ---
        self.ANGLES = [-60.0, -90.0, -120.0]
        self.ANGLE_BAND = 3.0

        # --- 오른쪽 벽 추종 (2단계: Phase 1 시작 후 STAGE1_DURATION_SEC 동안은
        # STAGE1_TARGET_RIGHT_DIST, 그 이후는 STAGE2_TARGET_RIGHT_DIST) ---
        self.STAGE1_DURATION_SEC = 7.0
        self.STAGE1_TARGET_RIGHT_DIST = 0.59
        self.STAGE2_TARGET_RIGHT_DIST = 0.59
        self.DIST_GAIN = 2.2
        self.ANGLE_GAIN = 4.0

        # --- 오차 연동 속도 조절 ---
        self.ERR_LOW = 0.03
        self.ERR_HIGH = 0.15
        self.CORRECT_SPEED = 0.06

        # --- 정면 앞벽 정지 ---
        self.STOP_DISTANCE = 0.4
        self.FRONT_ANGLE = 10.0

        # --- 유효 스캔 범위 ---
        self.SCAN_MIN_DEG = -135.0
        self.SCAN_MAX_DEG = self.FRONT_ANGLE

        # --- 주행 ---
        self.CRUISE_SPEED = 1.0    # 개방 구간 순항 속도 [m/s] (건드리지 않음 — 참고/비교용)
        self.PARKING_APPROACH_SPEED = 0.15  # 주차 지정 위치까지 접근하는 실제 속도 [m/s]
        self.MAX_STEER = 0.42

        # --- 앞벽 거리 기반 감속 (정렬 오차 기반 감속과 별개로 항상 같이 적용) ---
        # 앞벽이 DECEL_DISTANCE보다 가까워지면 STOP_DISTANCE에 다다를 때까지
        # CRUISE_SPEED -> DECEL_MIN_SPEED로 선형 감속한다. 감속 없이 CRUISE_SPEED
        # 그대로 벽까지 달리면 control_loop 주기(0.1s) 특성상 정지 명령이 한 박자
        # 늦게 먹어 오버슈트(벽에 너무 가깝게/부딪히듯 정지)할 위험이 있어서 추가함.
        self.DECEL_DISTANCE = 1.0   # 이 거리부터 감속 시작 [m]
        self.DECEL_MIN_SPEED = 0.15  # 정지 직전(STOP_DISTANCE 도달 시) 목표 속도 [m/s]
        # ==================

        self.front_dist = 999.0
        self.perp_dists = [None] * len(self.ANGLES)
        self.stopped = False

        # /parking_start 대기 없이 노드 실행 즉시 Phase 1 시작 (단독 테스트용).
        self._activated = True
        self._phase1_start_time = time.monotonic()
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            f"Phase 1: 오른쪽 벽 추종 즉시 시작 "
            f"({self.STAGE1_DURATION_SEC}초간 목표 {self.STAGE1_TARGET_RIGHT_DIST}m, "
            f"이후 {self.STAGE2_TARGET_RIGHT_DIST}m)")

    def _current_target_right_dist(self):
        elapsed = time.monotonic() - self._phase1_start_time
        if elapsed < self.STAGE1_DURATION_SEC:
            return self.STAGE1_TARGET_RIGHT_DIST
        return self.STAGE2_TARGET_RIGHT_DIST

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

    def _front_distance_speed_limit(self):
        if self.front_dist >= self.DECEL_DISTANCE:
            return self.PARKING_APPROACH_SPEED
        if self.front_dist <= self.STOP_DISTANCE:
            return self.DECEL_MIN_SPEED
        ratio = (self.front_dist - self.STOP_DISTANCE) / (self.DECEL_DISTANCE - self.STOP_DISTANCE)
        return self.DECEL_MIN_SPEED + ratio * (self.PARKING_APPROACH_SPEED - self.DECEL_MIN_SPEED)

    def control_loop(self):
        if self.stopped:
            return

        cmd = Twist()

        if self.front_dist <= self.STOP_DISTANCE:
            self.stopped = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f"\n정지! 앞벽 {self.front_dist:.2f}m")
            return

        cmd.linear.x = self.PARKING_APPROACH_SPEED

        target_right_dist = self._current_target_right_dist()

        valid = [p for p in self.perp_dists if p is not None]
        if len(valid) >= 2:
            avg_perp = sum(valid) / len(valid)
            dist_error = avg_perp - target_right_dist

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
                cmd.linear.x = self.PARKING_APPROACH_SPEED
            elif combined_err >= self.ERR_HIGH:
                cmd.linear.x = self.CORRECT_SPEED
            else:
                ratio = (combined_err - self.ERR_LOW) / (self.ERR_HIGH - self.ERR_LOW)
                cmd.linear.x = self.PARKING_APPROACH_SPEED - ratio * (self.PARKING_APPROACH_SPEED - self.CORRECT_SPEED)
        else:
            avg_perp = None
            angle_error = None
            cmd.angular.z = 0.0

        # 정렬 오차 기반 속도와 별개로, 앞벽 거리 기반 감속 한계를 항상 같이 적용
        # (둘 중 더 느린 쪽을 최종 속도로 사용)
        cmd.linear.x = min(cmd.linear.x, self._front_distance_speed_limit())

        self.cmd_pub.publish(cmd)

        perp_str = " ".join(f"{p:.2f}" if p is not None else "N" for p in self.perp_dists)
        avg_str = f"{avg_perp:.2f}" if avg_perp is not None else "N"
        ae_str = f"{angle_error:+.3f}" if angle_error is not None else "N"
        print(
            f"수직거리[{perp_str}] 평균 {avg_str} (목표{target_right_dist}) "
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
    node = MissionParkingTestNode()

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