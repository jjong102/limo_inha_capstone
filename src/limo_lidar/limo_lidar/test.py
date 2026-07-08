import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ParkingSequence(Node):
    def __init__(self):
        super().__init__('parking_sequence')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ===== 튜닝값 =====
        self.SPEED = 0.10
        self.MAX_STEER = 0.42

        self.T_BACK = 2.0    # 1) 우측 조향 후진 시간(초)
        self.T_FWD = 2.0     # 2) 좌측 조향 전진 시간(초)
        # ==================

        self.get_logger().info("주차 시퀀스 기동. 2초 후 시작.")
        time.sleep(2.0)
        self.run_parking()

    def drive(self, linear, angular, duration, label):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        start = time.time()
        while time.time() - start < duration:
            print(f"[{label}] 속도: {linear}, 조향: {angular}", end='\r')
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        print()

    def stop_robot(self, wait=1.0):
        stop = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop)
            time.sleep(0.05)
        time.sleep(wait)

    def run_parking(self):
        print("\n=== 주차 시퀀스 시작 ===\n")

        # 1) 우측 조향 + 후진  (후진 시 우측바퀴 = 음수)
        self.drive(-self.SPEED, -self.MAX_STEER, self.T_BACK, "1. 우측 후진")

        # 2) 정지
        self.stop_robot()

        # 3) 좌측 조향 + 전진  (전진 시 좌측바퀴 = 양수)
        self.drive(self.SPEED, -self.MAX_STEER, self.T_FWD, "2. 좌측 전진")

        # 마무리 정지
        self.stop_robot()

        print("\n완료. 종료.")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ParkingSequence()

if __name__ == '__main__':
    main()
