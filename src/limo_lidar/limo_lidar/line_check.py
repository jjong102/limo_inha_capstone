import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class RightLineDetector(Node):
    def __init__(self):
        super().__init__('right_line_detector')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)

        # ===== 튜닝값 =====
        # 흰색 라인 HSV 범위 (흰색 = 채도 낮고 명도 높음)
        self.lower_white = np.array([0, 0, 180])
        self.upper_white = np.array([180, 60, 255])
        self.ROI_TOP = 0.5    # 화면 아래 50%만 봄
        # ==================

        self.get_logger().info("오른쪽 라인 감지 시작 (표시만)")

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = img.shape[:2]

        # ROI: 화면 아래쪽 + 오른쪽 절반 (오른쪽 라인 찾기)
        roi_top = int(h * self.ROI_TOP)
        roi = img[roi_top:h, w//2:w]   # 아래쪽 + 오른쪽 절반

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_white, self.upper_white)

        white_pixels = cv2.countNonZero(mask)

        if white_pixels > 100:
            M = cv2.moments(mask)
            cx = int(M['m10'] / M['m00'])   # ROI 내 x
            cy = int(M['m01'] / M['m00'])
            # 전체 화면 기준 x (ROI가 오른쪽 절반이라 w//2 더함)
            line_x = cx + w // 2
            line_x_ratio = line_x / w
            print(f"오른쪽라인 x={line_x} ({line_x_ratio:.2f}) | {white_pixels}px", end='\r')

            # 라인 위치에 초록 원 + 세로선
            cv2.circle(img, (line_x, roi_top + cy), 10, (0, 255, 0), -1)
            cv2.line(img, (line_x, roi_top), (line_x, h), (0, 255, 0), 2)
        else:
            print(f"오른쪽라인 없음 | {white_pixels}px", end='\r')

        # ROI 영역 표시 (파란 박스)
        cv2.rectangle(img, (w//2, roi_top), (w, h), (255, 0, 0), 2)

        # 리모 화면에 창 띄우기
        cv2.imshow("Line Detect", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RightLineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()