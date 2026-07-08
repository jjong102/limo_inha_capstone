#!/usr/bin/env python3
"""
라이다 벽 추종 + 주차 시퀀스 통합 노드

[Phase 1] 오른쪽 벽과 오프셋을 유지하며 라인 추종 → 앞벽 0.36m에서 정지
[Phase 2] 정지 후 하드코딩된 주차 시퀀스 실행

두 파트의 튜닝값은 원래 코드 그대로 분리되어 있음.
- Phase 1 튜닝: LidarParking.__init__ 안의 "튜닝값" 영역
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
# [Phase 2 튜닝 영역] 주차 시퀀스 (원본 그대로)
# ============================================================

# --- 속도 (전체 시퀀스 공통, 절대값 기준 / 방향은 STEPS의 direction으로 결정) ---
LINEAR_SPEED = 0.15   # 전진/후진 시 사용할 선속도 (m/s)
STEER_SPEED  = 0.6    # 최대 조향 시 angular.z 절대값 (rad 또는 rad/s, Limo 모드에 따라 다름)
                      # Ackermann 모드 최대 조향각 or Diff 모드 angular.z 범위 확인 후 조정 필요

PUBLISH_RATE_HZ = 20.0  # cmd_vel publish 주기 (보통 안 건드려도 됨)

# --- 튜닝 ---
STEPS = [
    # direction, steer_sign,   duration
    ('REV',      -1,           5.4),
    ('REV',      +1,           1.8),
    ('FWD',      +1,           1.0),
    ('REV',      +1,           1.0),
    ('FWD',      +1,           1.0),
    ('REV',      +1,           0.8),
    ('FWD',      +1,           0.8),
]

# ============================================================


class LidarParking(Node):
    """[Phase 1] 오른쪽 벽 평행추종 + 앞벽 정지 (원본 그대로)"""

    def __init__(self):
        super().__init__('lidar_parking')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # ===== 튜닝값 =====
        # --- 오른쪽 벽 평행 추종 ---
        self.TARGET_RIGHT_DIST = 0.48    # 오른쪽 벽 목표거리(m) ★오프셋
        self.ANGLE_FRONT = -70.0         # 오른쪽 앞쪽 측정 각도
        self.ANGLE_BACK = -110.0         # 오른쪽 뒤쪽 측정 각도
        self.ANGLE_BAND = 5.0            # 각 측정점 ±범위(도)
        self.DIST_GAIN = 1.5             # 거리 오차 조향 게인 ★튜닝
        self.ANGLE_GAIN = 2.5            # 평행(각도) 오차 조향 게인 ★튜닝

        # --- 정면 앞벽 정지 ---
        self.STOP_DISTANCE = 0.36        # 앞벽 이 거리(m)면 정지 ★튜닝
        self.FRONT_ANGLE = 10.0          # 정면 ±각도

        # --- 주행 ---
        self.CRUISE_SPEED = 0.15       # 전진 속도 ★튜닝
        self.MAX_STEER = 0.42
        # 조향 부호(전진): +=왼쪽, -=오른쪽
        # ==================

        self.front_dist = 999.0
        self.d_front = None    # 오른쪽 앞점 거리
        self.d_back = None     # 오른쪽 뒤점 거리
        self.stopped = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("라이다 주차 접근 시작 (오른쪽 벽 평행추종)")

    def _avg_at(self, msg, center, band):
        """center ± band 각도 범위의 유효 거리 평균"""
        vals = []
        for i, dist in enumerate(msg.ranges):
            if dist <= 0.0 or math.isinf(dist) or math.isnan(dist):
                continue
            if dist < msg.range_min or dist > msg.range_max:
                continue
            ang = math.degrees(msg.angle_min + i * msg.angle_increment)
            if center - band <= ang <= center + band:
                vals.append(dist)
        return (sum(vals) / len(vals)) if vals else None

    def scan_callback(self, msg):
        # 정면 앞벽 (최소거리)
        front_min = 999.0
        for i, dist in enumerate(msg.ranges):
            if dist <= 0.0 or math.isinf(dist) or math.isnan(dist):
                continue
            if dist < msg.range_min or dist > msg.range_max:
                continue
            ang = math.degrees(msg.angle_min + i * msg.angle_increment)
            if -self.FRONT_ANGLE <= ang <= self.FRONT_ANGLE:
                front_min = min(front_min, dist)
        self.front_dist = front_min

        # 오른쪽 앞/뒤 두 점
        self.d_front = self._avg_at(msg, self.ANGLE_FRONT, self.ANGLE_BAND)
        self.d_back = self._avg_at(msg, self.ANGLE_BACK, self.ANGLE_BAND)

    def control_loop(self):
        if self.stopped:
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()

        # 정지: 정면 앞벽 0.36m 이내
        if self.front_dist <= self.STOP_DISTANCE:
            self.stopped = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f"\n정지! 앞벽 {self.front_dist:.2f}m")
            return

        cmd.linear.x = self.CRUISE_SPEED

        # 오른쪽 벽 평행 추종 (거리 + 각도)
        if self.d_front is not None and self.d_back is not None:
            avg_dist = (self.d_front + self.d_back) / 2.0
            dist_error = avg_dist - self.TARGET_RIGHT_DIST   # 벽 거리 오차
            angle_error = self.d_front - self.d_back         # 평행 오차(앞-뒤)

            # 벽에서 멀면(dist_error>0) 오른쪽으로 붙기(-)
            # 앞이 더 멀면(angle_error>0) 머리가 벌어짐 → 오른쪽으로(-)
            steer = -(self.DIST_GAIN * dist_error + self.ANGLE_GAIN * angle_error)
            steer = max(-self.MAX_STEER, min(self.MAX_STEER, steer))
            cmd.angular.z = steer
        else:
            # 오른쪽 벽 안 잡히면 직진
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

        # 상태 출력
        df = f"{self.d_front:.2f}" if self.d_front is not None else "N"
        db = f"{self.d_back:.2f}" if self.d_back is not None else "N"
        print(f"우앞 {df} 우뒤 {db} (목표{self.TARGET_RIGHT_DIST}) | 앞벽 {self.front_dist:.2f} | 조향 {cmd.angular.z:.2f}", end='\r')


class ParkingSequence(Node):
    """[Phase 2] 하드코딩 주차 시퀀스 (원본 그대로)"""

    def __init__(self):
        super().__init__('limo_parking_sequence')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dt = 1.0 / PUBLISH_RATE_HZ

    def publish_twist(self, linear_x: float, angular_z: float, duration: float):
        """지정한 linear/angular 값을 duration(초) 동안 일정 주기로 publish"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        steps = int(duration / self.dt)
        for _ in range(steps):
            self.pub.publish(msg)
            time.sleep(self.dt)

    def stop(self):
        self.publish_twist(0.0, 0.0, 0.3)  # 정지 (짧게 0 twist publish)

    def run_sequence(self):
        self.get_logger().info('주차 시퀀스 시작')

        for i, (direction, steer_sign, duration) in enumerate(STEPS, start=1):
            linear_x = LINEAR_SPEED if direction == 'FWD' else -LINEAR_SPEED
            angular_z = steer_sign * STEER_SPEED

            self.get_logger().info(
                f'{i}) {"전진" if direction == "FWD" else "후진"} + '
                f'{"좌" if steer_sign > 0 else "우"} 조향 ({duration}s)'
            )
            self.publish_twist(linear_x, angular_z, duration)

        # 종료 시 정지
        self.stop()
        self.get_logger().info('주차 시퀀스 종료')


def main(args=None):
    rclpy.init(args=args)

    lidar_node = LidarParking()
    parking_node = None

    try:
        # ---- Phase 1: 라이다 벽 추종 (stopped 될 때까지 스핀) ----
        while rclpy.ok() and not lidar_node.stopped:
            rclpy.spin_once(lidar_node, timeout_sec=0.1)

        # 확실히 멈춘 뒤 노드 정리
        lidar_node.cmd_pub.publish(Twist())
        lidar_node.get_logger().info("Phase 1 완료 → 주차 시퀀스로 전환")
        lidar_node.destroy_node()
        lidar_node = None

        # ---- Phase 2: 주차 시퀀스 ----
        if rclpy.ok():
            parking_node = ParkingSequence()
            parking_node.stop()          # 0.3s 정지 안정화(원치 않으면 이 줄 삭제)
            parking_node.run_sequence()

    except KeyboardInterrupt:
        pass
    finally:
        # 어느 단계에서 끝나든 안전하게 정지 + 정리
        if lidar_node is not None:
            lidar_node.cmd_pub.publish(Twist())
            lidar_node.destroy_node()
        if parking_node is not None:
            parking_node.stop()
            parking_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()