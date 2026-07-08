import cv2  # OpenCV 라이브러리이다.
import numpy as np  # 이미지 배열 처리를 위해 사용한다.
import rclpy  # ROS2 Python 라이브러리이다.
from rclpy.node import Node  # 노드 클래스를 가져온다.
from sensor_msgs.msg import CompressedImage  # 압축 이미지 메시지 타입이다.
from geometry_msgs.msg import Twist  # /cmd_vel 메시지 타입이다.

# ===== 튜닝 파라미터 =====
MAX_STEER = 0.42  # LIMO Ackermann angular.z 제한값이다.
LINEAR_SPEED = 0.12 # 전진 속도 [m/s] 이다.

# 신호등용 HSV 색상 범위
LOWER_GREEN = np.array([35, 100, 100])
UPPER_GREEN = np.array([85, 255, 255])
LOWER_RED1 = np.array([0, 100, 100])
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([160, 100, 100])
UPPER_RED2 = np.array([180, 255, 255])

# 신호등 감지 픽셀 수 제한 (가로 30%, 세로 50% 영역에 맞춤)
PIXEL_THRESHOLD = 300 
# =========================


def clamp_steer(value):  # 조향 제한 함수이다.
    return max(-MAX_STEER, min(MAX_STEER, value))  # -0.42 ~ 0.42 범위로 제한한다.


class TrafficLightOnly(Node):  # 신호등 인식 전용 노드이다.
    def __init__(self):  # 노드 생성 시 실행된다.
        super().__init__('camera_line_follow')  # 노드 이름을 기존과 동일하게 설정한다.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # /cmd_vel Publisher를 만든다.
        self.image_sub = self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self.image_callback, 10)  # 압축 이미지 토픽을 구독한다.
        
        # 차량 출발 여부 상태 제어 변수 (기본값: 정지)
        self.is_running = False
        self.get_logger().info("신호등 전용 노드가 기동되었습니다. (초기 상태: 정지)")

    def image_callback(self, msg):  # 이미지가 들어올 때마다 실행된다.
        np_arr = np.frombuffer(msg.data, np.uint8)  # 압축 이미지 바이트를 numpy 배열로 바꾼다.
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 압축 이미지를 OpenCV 이미지로 복원한다.

        if frame is None:  # 이미지 복원 실패 시 종료한다.
            return  # 콜백을 끝낸다.

        height, width, _ = frame.shape  # 이미지 높이와 너비를 가져온다.
        
        # 1. 신호등 영역 추출 (가로 왼쪽 30%, 세로 가운데 50%)
        start_y = int(height * 0.25)
        end_y = int(height * 0.75)
        start_x = 0
        end_x = int(width * 0.3)
        traffic_roi = frame[start_y:end_y, start_x:end_x]
        
        # 2. HSV 변환 및 마스크 생성
        hsv = cv2.cvtColor(traffic_roi, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN)
        mask_red1 = cv2.inRange(hsv, LOWER_RED1, UPPER_RED1)
        mask_red2 = cv2.inRange(hsv, LOWER_RED2, UPPER_RED2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        # 노이즈 제거
        kernel = np.ones((3, 3), np.uint8)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        
        # 3. 픽셀 카운트 및 상태 판별
        green_pixels = cv2.countNonZero(mask_green)
        red_pixels = cv2.countNonZero(mask_red)
        
        if red_pixels > PIXEL_THRESHOLD and red_pixels > green_pixels:
            if self.is_running:
                self.get_logger().info("🔴 빨간불 감지! 정지합니다.")
                self.is_running = False
        elif green_pixels > PIXEL_THRESHOLD and green_pixels > red_pixels:
            if not self.is_running:
                self.get_logger().info("🟢 초록불 감지! 출발합니다.")
                self.is_running = True

        # 4. 제어 명령(Twist) 생성 및 발행
        cmd = Twist()  # 속도 명령 메시지를 만든다.

        if self.is_running:
            # 초록불 상태: 직선 전진 (조향값은 작동하던 코드의 안전 포맷 적용)
            cmd.linear.x = LINEAR_SPEED  
            cmd.angular.z = clamp_steer(0.0)  
        else:
            # 빨간불 또는 초기 대기 상태: 정지
            cmd.linear.x = 0.0
            cmd.angular.z = clamp_steer(0.0)

        self.cmd_pub.publish(cmd)  # /cmd_vel을 발행한다.

    def destroy_node(self):  # 노드 종료 시 실행된다.
        cv2.destroyAllWindows()  # OpenCV 창을 닫는다.
        super().destroy_node()  # ROS2 노드를 정리한다.


def main(args=None):  # 실행 시작 함수이다.
    rclpy.init(args=args)  # ROS2를 초기화한다.
    node = TrafficLightOnly()  # 노드를 생성한다.
    rclpy.spin(node)  # 콜백을 계속 처리한다.
    node.destroy_node()  # 노드를 정리한다.
    rclpy.shutdown()  # ROS2를 종료한다.