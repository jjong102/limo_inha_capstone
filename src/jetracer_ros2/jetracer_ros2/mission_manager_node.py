import os
import signal
import subprocess
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

# mission_inference_node가 내보내는 inference/cmd_vel을 구독해서, 시간 기반으로
# 정해둔 속도 구간(phase_speeds/phase_durations)에 맞게 실시간으로 비율을 재조정한
# 뒤 실제 /cmd_vel로 중계하는 노드. /cmd_vel에 실제로 발행하는 건 이 노드 하나뿐이라,
# 다른 노드가 동시에 /cmd_vel에 쏴서 생기는 경쟁 문제가 구조적으로 없다.
#
# 재조정 원리: linear.x와 angular.z를 같은 비율로 스케일하면 r = v/ω(회전반경)이
# 그대로 보존된다. limo_base는 Ackermann 모드에서 이 r로 실제 조향각을 역산하기
# 때문에, 비율만 맞추면 로봇이 그리는 경로(조향)는 그대로고 속도만 바뀐다.
# (자세한 배경은 ackermann_utils.inner_angle_to_omega 참고)
#
# 신호등 게이트: mission_traffic_light_node의 mission/traffic_light_state를 구독해서,
# 'green'이 확정되기 전까지는(=아직 인식 안 됐거나 red인 동안) inference/cmd_vel이
# 뭘 보내든 무조건 정지(0)로 내보낸다. green이 되면 그 순간부터 시간을 재서
# phase_durations에 적힌 시간만큼씩 순서대로 phase_speeds 속도로 주행하고,
# mission_traffic_light_node는 더 필요 없으니 바로 SIGINT로 종료시킨다.
# 마지막 구간이 끝나면 mission_inference_node도 같은 방식으로 종료시킨다.
#
# 사용법:
#   ros2 run jetracer_ros2 mission_manager_node


class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        self.declare_parameter('input_topic', 'inference/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('traffic_light_topic', 'mission/traffic_light_state')
        # 시간 기반 속도 구간: phase_speeds[i] 속도로 phase_durations[i]초씩 순서대로 주행.
        self.declare_parameter('phase_speeds', [0.5, 0.2, 1.0])
        self.declare_parameter('phase_durations', [3.0, 2.0, 1.0])

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        traffic_light_topic = self.get_parameter('traffic_light_topic').value
        self.phase_speeds = list(self.get_parameter('phase_speeds').value)
        self.phase_durations = list(self.get_parameter('phase_durations').value)

        if len(self.phase_speeds) != len(self.phase_durations):
            raise ValueError('phase_speeds와 phase_durations의 길이가 서로 다릅니다.')

        # 각 구간이 끝나는 누적 시각(초). 예: [3.0, 2.0, 1.0] -> [3.0, 5.0, 6.0]
        self._phase_end_times = []
        acc = 0.0
        for d in self.phase_durations:
            acc += d
            self._phase_end_times.append(acc)
        self._total_duration = acc

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.cmd_sub = self.create_subscription(
            Twist, input_topic, self.cmd_callback, 10)
        self.traffic_light_sub = self.create_subscription(
            String, traffic_light_topic, self.traffic_light_callback, 10)

        # green이 확정되기 전까지는 무조건 정지 — 신호등 미인식/빨간불 상태의 기본값.
        self._blocked = True
        self._start_time = None
        self._finished = False
        self._end_timer = None

        self.get_logger().info(
            f'MissionManager ready  {input_topic} -> {output_topic}  '
            f'traffic_light={traffic_light_topic}  '
            f'phases={list(zip(self.phase_speeds, self.phase_durations))}  '
            '(초록불 확정 전까지 정지 상태로 대기)')

    def _current_target_speed(self, elapsed_sec: float) -> float:
        for speed, end_time in zip(self.phase_speeds, self._phase_end_times):
            if elapsed_sec < end_time:
                return speed
        return self.phase_speeds[-1]

    def traffic_light_callback(self, msg: String):
        if msg.data == 'green' and self._blocked:
            self._blocked = False
            self._start_time = self.get_clock().now()
            self._end_timer = self.create_timer(self._total_duration, self._on_mission_end)
            self.get_logger().info('GREEN 확정 — 주행 시작, mission_traffic_light_node 종료 시도')
            self._kill_process('mission_traffic_light_node')
        # 'red' 등 그 외 값은 별도 처리 없음 — 기본이 이미 정지(_blocked=True)라서 그대로 유지.

    def cmd_callback(self, msg: Twist):
        """들어온 cmd_vel을 현재 구간 속도에 맞게 비율 재조정해서 실시간 중계한다."""
        if self._blocked or self._finished:
            # 신호등 미확정/빨간불이거나 미션이 끝났으면 무조건 정지시킨다
            # (inference/cmd_vel이 어떤 속도를 보내든 무시).
            self.cmd_pub.publish(Twist())
            return

        elapsed_sec = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        target_speed = self._current_target_speed(elapsed_sec)

        if abs(msg.linear.x) < 1e-6:
            # 원본 속도가 0(정지 신호)이면 비율 계산이 불가능하니 그대로 전달한다.
            self.cmd_pub.publish(msg)
            return

        scale = target_speed / msg.linear.x
        out = Twist()
        out.linear.x = target_speed
        out.angular.z = msg.angular.z * scale
        self.cmd_pub.publish(out)

    def _on_mission_end(self):
        self._end_timer.cancel()
        self._finished = True
        self.cmd_pub.publish(Twist())  # 정지 명령 한 번 내보냄
        self.get_logger().info(
            f'{self._total_duration}초 경과 — 미션 종료, mission_inference_node 종료 시도')
        self._kill_process('mission_inference_node')

    def _kill_process(self, process_name: str):
        """process_name 실행파일 경로를 가진 프로세스에 SIGINT를 보낸다.

        리눅스는 프로세스 이름(comm)을 15자로 자르기 때문에, mission_inference_node
        (22자)나 mission_traffic_light_node(26자) 같은 이름엔 pgrep -x(완전 일치)가
        전혀 안 먹힌다. 그래서 -f(전체 명령줄)로 찾되, 그냥 부분 문자열로 찾으면
        mission_traffic_light_node를 찾다가 debug_mission_traffic_light_node
        (이름에 그대로 포함됨)까지 같이 걸리므로, 앞에 '/'를 붙여 실행파일 경로의
        마지막 구성요소로 정확히 매칭한다.
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
    node = MissionManagerNode()
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
