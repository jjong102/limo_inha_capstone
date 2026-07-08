import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ResetSteer(Node):
    def __init__(self):
        super().__init__('reset_steer')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("조향 중립 리셋 중...")

        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        start = time.time()
        while time.time() - start < 2.0:
            self.cmd_vel_pub.publish(stop)
            time.sleep(0.1)

        self.get_logger().info("조향 중립 리셋 완료")
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ResetSteer()


if __name__ == '__main__':
    main()