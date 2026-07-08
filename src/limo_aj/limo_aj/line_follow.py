import cv2  # OpenCV 라이브러리이다.
import numpy as np  # 이미지 배열 처리를 위해 사용한다.
import rclpy  # ROS2 Python 라이브러리이다.
from rclpy.node import Node  # 노드 클래스를 가져온다.
from sensor_msgs.msg import CompressedImage  # 압축 이미지 메시지를 가져온다.
from geometry_msgs.msg import Twist  # Twist 메시지를 가져온다.

# ===== 주행 튜닝 파라미터 =====
MAX_STEER = 0.42  # LIMO Ackermann angular.z 제한값이다.
LINEAR_SPEED = 0.12  # 전진 속도 [m/s] 이다.
STEER_GAIN = 0.7  # 조향 민감도(P 게인)이다.
THRESH_VALUE = 200  # 흰색 라인 검출 임계값이다.
ROI_RATIO = 0.3  # 이미지 아래쪽 30% 영역만 바닥 라인으로 본다.

# 우측 차선 유실 방지 튜닝 파라미터
LEFT_MASK_RATIO = 0.45  # [수정] 급커브 시 내 차선이 중앙 쪽으로 올 수 있으므로 가리는 영역을 45%로 살짝 감소
CX_OFFSET = -240  # 차선과의 거리 (max: -300)

# [추가] 라인 유실 시 강제로 감아버릴 탈출 조향값 (우측 차선 기준 우회전하므로 음수값)
# 코너를 너무 탈출 못 하고 밀려 나가면 이 값을 -0.35, -0.40 쪽으로 더 키우세요!
ESCAPE_RIGHT_STEER = -0.30 
# =============================


def clamp_steer(value):  # 조향 명령 제한 함수이다.
    return max(-MAX_STEER, min(MAX_STEER, value))  # -0.42 ~ 0.42 범위로 제한한다.


class CameraLineFollowAdvanced(Node):
    def __init__(self):
        super().__init__('camera_line_follow')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_sub = self.create_subscription(CompressedImage, '/camera/color/image_raw/compressed', self.image_callback, 10)
        
        self.last_steer = 0.0

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        height, width, _ = frame.shape
        
        # 1. 바닥만 보기 위해 ROI 추출 (하단 30% 영역)
        roi = frame[int(height * (1.0 - ROI_RATIO)):height, :]
        
        # 2. 이진화로 흰색 라인 마스크 생성
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, THRESH_VALUE, 255, cv2.THRESH_BINARY)
        
        # 노이즈 제거
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 3. 왼쪽 영역 가리기 (오른쪽 차선만 남기기)
        left_cut = int(width * LEFT_MASK_RATIO)
        mask[:, :left_cut] = 0  # 왼쪽 절반 영역의 픽셀을 강제로 0으로 지움

        # 4. 남은 오른쪽 차선의 무게중심 계산
        moments = cv2.moments(mask)
        
        cmd = Twist()

        if moments['m00'] > 0:
            # [라인이 보일 때] 정상 주행 및 조향값 업데이트
            cx = int(moments['m10'] / moments['m00']) + CX_OFFSET
            center_x = width // 2

            error = center_x - cx  # 오차 계산
            normalized_error = error / center_x  # 정규화 (-1 ~ 1)
            
            steer_cmd = STEER_GAIN * normalized_error
            steer_cmd = clamp_steer(steer_cmd)

            cmd.linear.x = LINEAR_SPEED
            cmd.angular.z = steer_cmd
            
            self.last_steer = steer_cmd
            self.get_logger().info(f'Target_CX={cx}, Steer={steer_cmd:.2f}')
        else:
            # [수정 핵심: 라인을 놓쳤을 때] 우회전 급커브 영역에서 놓쳤을 확률 99%!
            # 직전 조향에만 의존하지 않고, 강제 탈출 조향값(ESCAPE_RIGHT_STEER)을 밀어 넣습니다.
            cmd.linear.x = LINEAR_SPEED * 0.8  # 안전을 위해 속도를 80% 수준으로 하향
            cmd.angular.z = clamp_steer(ESCAPE_RIGHT_STEER)  # 강제 우회전 기동 발생!
            
            self.get_logger().warn(f'⚠️ 우측 차선 유실! 강제 우회전 탈출 중... (Steer={ESCAPE_RIGHT_STEER:.2f})')

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CameraLineFollowAdvanced()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()