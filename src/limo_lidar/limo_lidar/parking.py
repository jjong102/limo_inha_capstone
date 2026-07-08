#!/usr/bin/env python3
"""
Limo Pro 주차 시퀀스 노드

동작 순서 (총 5 스텝): 후진 -> 후진 -> 전진 -> 후진 -> 후진
각 스텝마다 조향 방향(+ / -)과 지속 시간(초)만 바꿔서 튜닝합니다.

/cmd_vel 로 geometry_msgs/Twist 를 publish 합니다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


# ============================================================
# [튜닝 영역] 여기 값들만 눈으로 보고 바꾸면 됩니다.
# ============================================================

# --- 속도 (전체 시퀀스 공통, 절대값 기준 / 방향은 STEPS의 direction으로 결정) ---
LINEAR_SPEED = 0.15   # 전진/후진 시 사용할 선속도 (m/s)
STEER_SPEED  = 0.6    # 최대 조향 시 angular.z 절대값 (rad 또는 rad/s, Limo 모드에 따라 다름)
                      # Ackermann 모드 최대 조향각 or Diff 모드 angular.z 범위 확인 후 조정 필요

PUBLISH_RATE_HZ = 20.0  # cmd_vel publish 주기 (보통 안 건드려도 됨)

# --- 튜닝 ---
STEPS = [
    # direction, steer_sign,   duration
    ('REV',      -1,           5.5),
    ('REV',      0,           0.8),   
    ('REV',      +1,           1.8),   
    ('FWD',      +1,           1.0),   
    ('REV',      +1,           1.0),
    ('FWD',      +1,           1.0),
    ('REV',      +1,           0.8),
    ('FWD',      +1,           0.8),        
]

# ============================================================


class ParkingSequence(Node):
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
    node = ParkingSequence()
    try:
        node.run_sequence()
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    