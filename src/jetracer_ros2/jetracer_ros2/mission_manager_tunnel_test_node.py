import os
import signal
import subprocess
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32

# mission_inference_node가 내보내는 inference/cmd_vel을 그대로 /cmd_vel로 중계하다가,
# mission_tunnel_node가 보고하는 밝기가 brightness_threshold 이하로 떨어지면
# (터널 진입으로 판단) mission_tunnel_node를 종료시키고, slow_down_delay_sec 뒤부터
# slow_speed로 감속해서 계속 중계하는 노드. /cmd_vel에 실제로 발행하는 건 이 노드
# 하나뿐이라 — mission_inference_node는 inference/cmd_vel에만 발행하도록 이미
# 만들어져 있어서 — 이 노드가 뜨기 전까지는 아무도 실제 /cmd_vel에 발행하지 않는다
# (mission_manager_node와 동일한 구조).
#
# 감속 원리: linear.x와 angular.z를 같은 비율로 스케일하면 r = v/ω(회전반경)이
# 그대로 보존된다. limo_base는 Ackermann 모드에서 이 r로 실제 조향각을 역산하기
# 때문에, 비율만 맞추면 로봇이 그리는 경로(조향)는 그대로고 속도만 바뀐다.
# (자세한 배경은 ackermann_utils.inner_angle_to_omega, mission_manager_node 참고)
#
# 이 조도 감지는 1회성이다 — 한 번 어두워짐을 감지하면 그걸로 끝, 다시 밝아져도
# 원래 속도로 안 돌아온다.
#
# 사용법:
#   ros2 run jetracer_ros2 mission_manager_tunnel_test_node


class MissionManagerTunnelTestNode(Node):
    def __init__(self):
        super().__init__('mission_manager_tunnel_test_node')

        self.declare_parameter('input_topic', 'inference/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('tunnel_topic', 'mission/tunnel_brightness')
        self.declare_parameter('brightness_threshold', 75.0)  # 이 이하로 떨어지면 터널로 판단
        self.declare_parameter('slow_down_delay_sec', 1.5)    # 감지 후 이만큼 지나서 감속 시작
        self.declare_parameter('slow_speed', 0.5)             # 감속 목표 속도 [m/s]

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        tunnel_topic = self.get_parameter('tunnel_topic').value
        self.brightness_threshold = self.get_parameter('brightness_threshold').value
        self.slow_down_delay_sec = self.get_parameter('slow_down_delay_sec').value
        self.slow_speed = self.get_parameter('slow_speed').value

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.cmd_sub = self.create_subscription(
            Twist, input_topic, self.cmd_callback, 10)
        self.tunnel_sub = self.create_subscription(
            Float32, tunnel_topic, self.tunnel_callback, 10)

        self._triggered = False   # 조도 감지 이벤트가 이미 발생했는지 (1회성)
        self._slow_mode = False   # 감속 모드로 전환됐는지
        self._slow_down_timer = None

        self.get_logger().info(
            f'MissionManagerTunnelTest ready  {input_topic} -> {output_topic}  '
            f'brightness_threshold={self.brightness_threshold}  '
            f'slow_down_delay_sec={self.slow_down_delay_sec}s  slow_speed={self.slow_speed}m/s')

    def tunnel_callback(self, msg: Float32):
        if self._triggered:
            return  # 이미 한 번 발동했으면 더 볼 필요 없음
        if msg.data <= self.brightness_threshold:
            self._triggered = True
            self.get_logger().info(
                f'조도 {msg.data:.1f} <= {self.brightness_threshold} 감지 — '
                f'mission_tunnel_node 종료, {self.slow_down_delay_sec}초 뒤 '
                f'{self.slow_speed}m/s로 감속')
            self._kill_process('mission_tunnel_node')
            self._slow_down_timer = self.create_timer(
                self.slow_down_delay_sec, self._start_slow_mode)

    def _start_slow_mode(self):
        self._slow_down_timer.cancel()
        self._slow_mode = True
        self.get_logger().info(f'감속 모드 진입 — {self.slow_speed}m/s로 주행')

    def cmd_callback(self, msg: Twist):
        """들어온 cmd_vel을 그대로 중계하다가, 감속 모드 진입 후엔 비율 재조정한다."""
        if not self._slow_mode:
            self.cmd_pub.publish(msg)
            return

        if abs(msg.linear.x) < 1e-6:
            # 원본 속도가 0(정지 신호)이면 비율 계산이 불가능하니 그대로 전달한다.
            self.cmd_pub.publish(msg)
            return

        scale = self.slow_speed / msg.linear.x
        out = Twist()
        out.linear.x = self.slow_speed
        out.angular.z = msg.angular.z * scale
        self.cmd_pub.publish(out)

    def _kill_process(self, process_name: str):
        """process_name 실행파일 경로를 가진 프로세스에 SIGINT를 보낸다.

        pgrep -x는 리눅스가 프로세스 이름(comm)을 15자로 잘라서 긴 이름엔 안 먹히고,
        그냥 pgrep -f는 부분 문자열까지 걸릴 수 있어서, 앞에 '/'를 붙여 실행파일
        경로의 마지막 구성요소로 정확히 매칭한다 (mission_manager_node와 동일한 방식).
        """
        pattern = f'/{process_name}([[:space:]]|$)'
        try:
            output = subprocess.check_output(['pgrep', '-f', pattern])
        except subprocess.CalledProcessError:
            self.get_logger().warn(f'{process_name} 프로세스를 못 찾았습니다 (이미 꺼져있는 듯).')
            return

        for pid_str in output.decode().split():
            pid = int(pid_str)
            if pid == os.getpid():
                continue
            try:
                os.kill(pid, signal.SIGINT)
                self.get_logger().info(f'{process_name}(pid={pid})에 SIGINT 전송')
            except ProcessLookupError:
                pass


def main(args=None):
    argv = sys.argv if args is None else args

    pkg = get_package_share_directory('jetracer_ros2')
    params_path = os.path.join(pkg, 'params', 'params.yaml')
    ros_args = ['--ros-args', '--params-file', params_path]

    rclpy.init(args=[argv[0]] + ros_args)
    node = MissionManagerTunnelTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
