import os
import subprocess
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from jetracer_ros2.inference_node import InferenceNode, _extract_plain_arg

# inference_node를 그대로 재사용하되, 조이스틱 Y/A로 자율주행 cmd_vel 발행만
# 멈췄다 다시 시작했다 할 수 있게 한 노드 (inference_node.py는 건드리지 않음).
#
# 조이스틱 버튼 (/joy 기준, 이 패드에서 실측 확인된 인덱스):
#   Y = buttons[4] → 주행 시작 → /inference/drive_status에 'start' 발행, cmd_vel 재개
#   A = buttons[0] → 주행 정지 → /inference/drive_status에 'stop' 발행, cmd_vel = 0
# A를 눌러도 모델 추론(이미지 → 조향 예측)은 계속 돈다 — 실제 /cmd_vel 발행만 막는다.
# 시작 직후엔 안전을 위해 정지 상태로 대기하고, Y를 눌러야 주행이 시작된다.
#
# section_id:=N 을 주면 시작할 때부터 해당 구간 모델로 추론한다 (inference_node와 동일).
#
# 사용법:
#   ros2 run jetracer_ros2 inference_StopAndGo_with_joy_node
#   ros2 run jetracer_ros2 inference_StopAndGo_with_joy_node section_id:=2

START_BUTTON = 4  # Y
STOP_BUTTON = 0   # A


class _GatedPublisher:
    """실제 cmd_vel publisher를 감싸서, enabled=False면 무조건 정지(Twist())로 내보낸다."""

    def __init__(self, real_publisher):
        self._real = real_publisher
        self.enabled = False

    def publish(self, msg: Twist):
        self._real.publish(msg if self.enabled else Twist())


class InferenceStopAndGoNode(InferenceNode):
    def __init__(self):
        super().__init__()

        # InferenceNode.image_callback은 self.cmd_pub.publish(...)만 호출하므로,
        # publisher 객체를 바꿔치기하는 것만으로 inference_node.py 코드를 전혀
        # 건드리지 않고 발행을 게이팅할 수 있다.
        self._gated_pub = _GatedPublisher(self.cmd_pub)
        self.cmd_pub = self._gated_pub

        self.status_pub = self.create_publisher(String, '/inference/drive_status', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.get_logger().info(
            f'StopAndGo ready (정지 상태로 시작) — '
            f'Y(button[{START_BUTTON}])=start, A(button[{STOP_BUTTON}])=stop')

    def joy_callback(self, msg: Joy):
        if (len(msg.buttons) > START_BUTTON and msg.buttons[START_BUTTON]
                and not self._gated_pub.enabled):
            self._gated_pub.enabled = True
            self.status_pub.publish(String(data='start'))
            self.get_logger().info('START (Y) — 주행 재개')

        if (len(msg.buttons) > STOP_BUTTON and msg.buttons[STOP_BUTTON]
                and self._gated_pub.enabled):
            self._gated_pub.enabled = False
            self.status_pub.publish(String(data='stop'))
            self.get_logger().info('STOP (A) — 주행 정지 (추론은 계속 동작)')
            self._gated_pub.publish(Twist())


def main(args=None):
    argv = sys.argv if args is None else args
    section_id = int(_extract_plain_arg(argv, 'section_id', 1))

    pkg = get_package_share_directory('jetracer_ros2')
    params_path = os.path.join(pkg, 'params', 'params.yaml')
    ros_args = ['--ros-args', '--params-file', params_path]

    # 조이스틱 드라이버 — /joy 토픽 발행
    joy_proc = subprocess.Popen(
        ['ros2', 'run', 'joy', 'joy_node',
         '--ros-args', '-p', 'autorepeat_rate:=20.0'])

    rclpy.init(args=[argv[0]] + ros_args)
    node = InferenceStopAndGoNode()

    node.current_section = section_id
    if section_id in node.models:
        node.get_logger().info(f'Starting with section={section_id}')
    else:
        node.get_logger().warn(
            f'section={section_id} 모델이 로드되지 않았습니다 (engine_dir 확인).')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        joy_proc.terminate()
        joy_proc.wait()


if __name__ == '__main__':
    main()
