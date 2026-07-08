import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import cv2
import numpy as np


class YellowWallStop(Node):
    def __init__(self):
        super().__init__('yellow_wall_stop')
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 카메라 구독
        self.img_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        # 라이다 구독 (센서 QoS)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        # ===== 튜닝값 =====
        # 카메라 (노란색) - 지금은 표시용
        self.ROI_TOP = 0.0
        self.lower_yellow = np.array([22, 50, 100])
        self.upper_yellow = np.array([35, 255, 255])
        self.MIN_PIXELS = 300

        # 라이다 (앞 벽 거리) - 이걸로 정지
        self.STOP_DISTANCE = 0.37    # 앞 벽 이 거리(m) 이하면 정지 ★튜닝
        self.FRONT_ANGLE = 10.0      # 정면 ±각도(도) 안의 거리만 봄

        # 주행
        self.CRUISE_SPEED = 0.08     # 전진 속도 ★튜닝
        # ==================

        # 상태
        self.yellow_seen = False
        self.front_dist = 999.0
        self.stopped = False

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("앞벽 거리 정지 노드 시작")

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = img.shape[:2]
        roi = img[int(h * self.ROI_TOP):h, 0:w]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        yellow_pixels = cv2.countNonZero(mask)
        self.yellow_seen = (yellow_pixels >= self.MIN_PIXELS)
        self.last_yellow_px = yellow_pixels

    def scan_callback(self, msg):
        min_dist = 999.0
        for i, dist in enumerate(msg.ranges):
            if dist <= 0.0 or math.isinf(dist) or math.isnan(dist):
                continue
            if dist < msg.range_min or dist > msg.range_max:
                continue
            angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            if -self.FRONT_ANGLE <= angle_deg <= self.FRONT_ANGLE:
                min_dist = min(min_dist, dist)
        self.front_dist = min_dist

    def control_loop(self):
        if self.stopped:
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()

        # 앞벽 거리만으로 정지
        if self.front_dist <= self.STOP_DISTANCE:
            self.stopped = True
            self.cmd_pub.publish(Twist())
            self.get_logger().info(f"\n정지! 앞벽 {self.front_dist:.2f}m")
            return
        else:
            cmd.linear.x = self.CRUISE_SPEED

        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        y = "노란색O" if self.yellow_seen else "노란색X"
        print(f"{y} ({getattr(self,'last_yellow_px',0)}px) | 앞벽 {self.front_dist:.2f}m | 정지기준 {self.STOP_DISTANCE}m", end='\r')


def main(args=None):
    rclpy.init(args=args)
    node = YellowWallStop()
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