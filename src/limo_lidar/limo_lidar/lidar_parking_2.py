#!/usr/bin/env python3
"""
오른쪽 벽 추종(수직거리 변환 방식) + 주차 시퀀스 통합 노드

[Phase 1] 오른쪽 벽과 오프셋을 유지하며 라인 추종 → 앞벽 0.36m에서 정지
          (라이다 유효범위: -135°~+10°만 사용, 왼쪽 앞/왼쪽은 완전히 무시)
[Phase 2] 정지 후 하드코딩된 주차 시퀀스 실행

두 파트의 튜닝값은 원래 코드 그대로 분리되어 있음.
- Phase 1 튜닝: WallFollowPerp.__init__ 안의 "튜닝값" 영역
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
    ('REV',      -1,           5.2),
    ('REV',      +1,           1.6),
    ('FWD',      +1,           1.0),
    ('REV',      +1,           1.0),
    ('FWD',      +1,           1.0),
    ('REV',      +1,           0.8),
    ('FWD',      +1,           0.8),
]

# ============================================================


class WallFollowPerp(Node):
    """[Phase 1] 오른쪽 벽 추종 (수직거리 변환 방식, 원본 그대로)"""

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
        self.DIST_GAIN = 2.2            # 거리 오차 조향 게인 ★튜닝 (기존 1.8→상향)
        self.ANGLE_GAIN = 4.0           # 평행(각도) 오차 조향 게인 ★튜닝 (기존 3.0→상향)

        # --- 정렬 오차 연동 속도 조절 (자세 보정 시간을 벌기 위함) ★핵심 튜닝 ---
        # 오차가 클수록 천천히 가서 헤딩/오프셋을 고칠 "시간(=거리)"을 확보.
        # 오차가 ERR_HIGH 이상이면 CORRECT_SPEED로, ERR_LOW 이하면 CRUISE_SPEED로,
        # 그 사이는 선형 보간.
        self.ERR_LOW = 0.03             # 이 오차 이하면 완전히 정렬된 것으로 보고 순항속도 ★튜닝
        self.ERR_HIGH = 0.15            # 이 오차 이상이면 최대로 감속 ★튜닝
        self.CORRECT_SPEED = 0.06       # 정렬 오차 클 때 속도(m/s) ★튜닝

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

        cmd.linear.x = self.CRUISE_SPEED  # 기본값, 아래에서 오차에 따라 재조정

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

            # ---- 오차 연동 속도 조절 ----
            # 두 오차를 하나의 크기로 합쳐서(단순 절댓값 합) 정렬 정도를 평가.
            # 오차가 클수록 CORRECT_SPEED 쪽으로, 작을수록 CRUISE_SPEED 쪽으로 선형 보간.
            combined_err = abs(dist_error) + abs(angle_error)
            if combined_err <= self.ERR_LOW:
                cmd.linear.x = self.CRUISE_SPEED
            elif combined_err >= self.ERR_HIGH:
                cmd.linear.x = self.CORRECT_SPEED
            else:
                ratio = (combined_err - self.ERR_LOW) / (self.ERR_HIGH - self.ERR_LOW)
                cmd.linear.x = self.CRUISE_SPEED - ratio * (self.CRUISE_SPEED - self.CORRECT_SPEED)
        else:
            # 오른쪽 벽 안 잡히면 직진 (속도는 기본 순항속도 유지)
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
            f"| 평행오차 {ae_str} | 앞벽 {self.front_dist:.2f} | 조향 {cmd.angular.z:+.2f} 속도 {cmd.linear.x:.2f}",
            end='\r'
        )


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

    wall_node = WallFollowPerp()
    parking_node = None

    try:
        # ---- Phase 1: 오른쪽 벽 추종 (stopped 될 때까지 스핀) ----
        while rclpy.ok() and not wall_node.stopped:
            rclpy.spin_once(wall_node, timeout_sec=0.1)

        # 확실히 멈춘 뒤 노드 정리
        wall_node.cmd_pub.publish(Twist())
        wall_node.get_logger().info("Phase 1 완료 → 주차 시퀀스로 전환")
        wall_node.destroy_node()
        wall_node = None

        # ---- Phase 2: 주차 시퀀스 ----
        if rclpy.ok():
            parking_node = ParkingSequence()
            parking_node.stop()          # 0.3s 정지 안정화(원치 않으면 이 줄 삭제)
            parking_node.run_sequence()

    except KeyboardInterrupt:
        pass
    finally:
        # 어느 단계에서 끝나든 안전하게 정지 + 정리
        if wall_node is not None:
            wall_node.cmd_pub.publish(Twist())
            wall_node.destroy_node()
        if parking_node is not None:
            parking_node.stop()
            parking_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()