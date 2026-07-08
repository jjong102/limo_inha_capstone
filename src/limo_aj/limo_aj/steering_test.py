import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class LimoSteeringTestNode(Node):
    def __init__(self):
        super().__init__('limo_steering_test_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 테스트 파라미터 설정
        self.TEST_SPEED = 0.15       # 테스트 속도 (m/s)
        self.MAX_STEER = 0.42        # 테스트할 최대 조향각 (rad)
        self.TEST_DURATION = 3.0     # 각 동작별 지속 시간 (초)

        self.get_logger().info("🚀 LIMO 조향 상태 진단 노드가 기동되었습니다. 2초 후 테스트를 시작합니다.")
        time.sleep(2.0)
        self.run_test()

    def publish_twist(self, linear, angular, msg_text):
        """명령을 퍼블리시하고 터미널에 상태를 출력하는 함수"""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        
        start_time = time.time()
        # 3초 동안 주기적으로 메시지 송신 (LIMO 모터 타임아웃 방지)
        while time.time() - start_time < self.TEST_DURATION:
            print(f"[{msg_text}] 📥 입력 명령 -> 선속도: {linear} m/s, 조향각: {angular} rad (약 {angular * 57.2958:.1f}°)", end='\r')
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        print() # 줄바꿈

    def run_test(self):
        print("\n📢 === LIMO 조향 및 구동 4단계 테스트 시작 ===")
        print("👀 [확인 방법]: 터미널의 입력값(Input)을 보시면서, 실제 바퀴(Output)가 그만큼 꺾이는지 눈으로 확인하세요!\n")

        # 1. 전진 + 좌측 최대 조향
        self.publish_twist(self.TEST_SPEED, self.MAX_STEER, "1/4. 전진 우회전(좌측 꺾임)")
        self.stop_robot()

        # 2. 전진 + 우측 최대 조향
        self.publish_twist(self.TEST_SPEED, -self.MAX_STEER, "2/4. 전진 좌회전(우측 꺾임)")
        self.stop_robot()

        # 3. 후진 + 좌측 최대 조향
        self.publish_twist(-self.TEST_SPEED, self.MAX_STEER, "3/4. 후진 우회전(좌측 꺾임)")
        self.stop_robot()

        # 4. 후진 + 우측 최대 조향
        self.publish_twist(-self.TEST_SPEED, -self.MAX_STEER, "4/4. 후진 좌회전(우측 꺾임)")
        self.stop_robot()

        print("\n🏁 모든 테스트가 완료되었습니다. 노드를 종료합니다.")
        self.destroy_node()
        rclpy.shutdown()

    def stop_robot(self):
        """동작 사이 안전을 위한 일시 정지"""
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_vel_pub.publish(stop_twist)
            time.sleep(0.05)
        time.sleep(1.0) # 1초간 대기 후 다음 테스트 진행

def main(args=None):
    rclpy.init(args=args)
    node = LimoSteeringTestNode()

if __name__ == '__main__':
    main()