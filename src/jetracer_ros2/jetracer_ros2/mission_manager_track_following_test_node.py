import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

# mission_inference_node가 내보내는 inference/cmd_vel을 받아서 target_speed(기본
# 0.5m/s)로 재조정해 /cmd_vel로 중계하는 노드. mission_track_following_node가
# 보고하는 mission/track_following_state가 'stop'이면 무조건 정지(0)시키고,
# 다시 'clear'가 오면 resume_delay_sec(기본 1초)만큼 있다가 주행을 재개한다.
# /cmd_vel에 실제로 발행하는 건 이 노드 하나뿐이라 — mission_inference_node는
# inference/cmd_vel에만 발행하도록 이미 만들어져 있어서 — 이 노드가 뜨기 전까지는
# 아무도 실제 /cmd_vel에 발행하지 않는다 (mission_manager_pestop_test_node와
# 동일한 구조). 상태를 아직 한 번도 못 받았을 수도 있으니 안전하게 기본값은
# 정지(차단)로 시작한다.
#
# 재조정 원리: linear.x와 angular.z를 같은 비율로 스케일하면 r = v/ω(회전반경)이
# 그대로 보존된다 (자세한 배경은 ackermann_utils.inner_angle_to_omega 참고).
#
# 사용법:
#   ros2 run jetracer_ros2 mission_manager_track_following_test_node


class MissionManagerTrackFollowingTestNode(Node):
    def __init__(self):
        super().__init__('mission_manager_track_following_test_node')

        self.declare_parameter('input_topic', 'inference/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('track_following_topic', 'mission/track_following_state')
        self.declare_parameter('target_speed', 0.5)  # 주행 속도 [m/s]
        self.declare_parameter('resume_delay_sec', 1.0)  # 정지 해제 후 이만큼 있다가 출발

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        track_following_topic = self.get_parameter('track_following_topic').value
        self.target_speed = self.get_parameter('target_speed').value
        self.resume_delay_sec = self.get_parameter('resume_delay_sec').value

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.cmd_sub = self.create_subscription(
            Twist, input_topic, self.cmd_callback, 10)
        self.track_sub = self.create_subscription(
            String, track_following_topic, self.track_callback, 10)

        self._blocked = True  # 상태 수신 전까지는 안전하게 정지 상태로 대기
        self._resume_timer = None

        self.get_logger().info(
            f'MissionManagerTrackFollowingTest ready  {input_topic} -> {output_topic}  '
            f'track_following={track_following_topic}  target_speed={self.target_speed}  '
            f'resume_delay_sec={self.resume_delay_sec}  (상태 수신 전까지 정지 상태로 대기)')

    def track_callback(self, msg: String):
        if msg.data == 'stop':
            if self._resume_timer is not None:
                self._resume_timer.cancel()
                self._resume_timer = None
            if not self._blocked:
                self._blocked = True
                self.get_logger().info('정지')
            return

        # 'clear': 이미 주행 중이거나 재개 대기 중이면 아무것도 안 함. 정지 상태였을
        # 때만 resume_delay_sec 뒤에 출발하도록 타이머를 건다.
        if self._blocked and self._resume_timer is None:
            self.get_logger().info(f'정지 해제 — {self.resume_delay_sec}초 뒤 주행 재개')
            self._resume_timer = self.create_timer(self.resume_delay_sec, self._do_resume)

    def _do_resume(self):
        self._resume_timer.cancel()
        self._resume_timer = None
        self._blocked = False
        self.get_logger().info('주행 재개')

    def cmd_callback(self, msg: Twist):
        if self._blocked:
            self.cmd_pub.publish(Twist())
            return

        if abs(msg.linear.x) < 1e-6:
            self.cmd_pub.publish(msg)
            return

        scale = self.target_speed / msg.linear.x
        out = Twist()
        out.linear.x = self.target_speed
        out.angular.z = msg.angular.z * scale
        self.cmd_pub.publish(out)


def main(args=None):
    argv = sys.argv if args is None else args

    pkg = get_package_share_directory('jetracer_ros2')
    params_path = os.path.join(pkg, 'params', 'params.yaml')
    ros_args = ['--ros-args', '--params-file', params_path]

    rclpy.init(args=[argv[0]] + ros_args)
    node = MissionManagerTrackFollowingTestNode()
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
