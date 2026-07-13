import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

# mission_inference_node가 내보내는 inference/cmd_vel을 /cmd_vel로 중계하는 노드.
# /cmd_vel에 실제로 발행하는 건 이 노드 하나뿐이라 — mission_inference_node는
# inference/cmd_vel에만 발행하도록 이미 만들어져 있어서 — 이 노드가 뜨기 전까지는
# 아무도 실제 /cmd_vel에 발행하지 않는다 (mission_manager_node와 동일한 구조).
#
# 흐름:
#   1. mission_people_estop_node가 켜져서 mission/people_estop_state에 처음 뭔가를
#      발행하는 순간(그 노드가 시작 직후 보내는 'clear' 신호)을 "센서 켜짐"으로
#      보고 그때부터 주행을 시작한다 (그 전까지는 정지 상태로 대기).
#   2. 주행 시작 후 slow_after_sec(기본 7초)가 지나면 속도를 slow_speed(기본
#      0.5m/s)로 낮춘다 — mission_people_estop_node가 본격적으로 감시를 시작하는
#      시점에 맞춰 미리 감속.
#   3. 그 뒤 'stop'이 오면 즉시 정지한다.
#   4. 그 뒤 'clear'가 다시 오면(=장애물이 사라져서 mission_people_estop_node가
#      종료된다는 신호) resume_delay_sec(기본 1초)만큼 있다가 원래 속도(1m/s,
#      재조정 없이 inference/cmd_vel 그대로)로 주행을 재개한다.
#
# 재조정 원리: linear.x와 angular.z를 같은 비율로 스케일하면 r = v/ω(회전반경)이
# 그대로 보존된다 (자세한 배경은 ackermann_utils.inner_angle_to_omega 참고).
#
# 사용법:
#   ros2 run jetracer_ros2 mission_manager_pestop_test_node


class MissionManagerPestopTestNode(Node):
    def __init__(self):
        super().__init__('mission_manager_pestop_test_node')

        self.declare_parameter('input_topic', 'inference/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('people_estop_topic', 'mission/people_estop_state')
        self.declare_parameter('slow_after_sec', 7.0)  # 첫 clear(센서 켜짐) 이후 이 시간 지나면 감속
        self.declare_parameter('slow_speed', 0.5)        # 감속 목표 속도 [m/s]
        self.declare_parameter('resume_delay_sec', 1.0)  # 장애물 해소 후 이만큼 있다가 출발

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        people_estop_topic = self.get_parameter('people_estop_topic').value
        self.slow_after_sec = self.get_parameter('slow_after_sec').value
        self.slow_speed = self.get_parameter('slow_speed').value
        self.resume_delay_sec = self.get_parameter('resume_delay_sec').value

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.cmd_sub = self.create_subscription(
            Twist, input_topic, self.cmd_callback, 10)
        self.estop_sub = self.create_subscription(
            String, people_estop_topic, self.estop_callback, 10)

        # mission_people_estop_node가 켜졌다는 신호를 받기 전까지는 정지 상태로 대기.
        self._blocked = True
        self._sensor_seen = False
        self._slow_mode = False
        self._slow_timer = None
        self._resume_timer = None

        self.get_logger().info(
            f'MissionManagerPestopTest ready  {input_topic} -> {output_topic}  '
            f'people_estop={people_estop_topic}  slow_after_sec={self.slow_after_sec}  '
            f'slow_speed={self.slow_speed}  (센서 켜짐 신호 받을 때까지 정지 상태로 대기)')

    def estop_callback(self, msg: String):
        if not self._sensor_seen:
            # 첫 메시지 = mission_people_estop_node가 막 켜졌다는 신호. 주행 시작하고
            # slow_after_sec 뒤에 감속하도록 타이머를 건다.
            self._sensor_seen = True
            self._blocked = False
            self.get_logger().info('mission_people_estop_node 켜짐 감지 — 주행 시작')
            self._slow_timer = self.create_timer(self.slow_after_sec, self._enter_slow_mode)
            return

        if msg.data == 'stop':
            self.get_logger().info('STOP 감지 — 정지')
            self._blocked = True
            if self._resume_timer is not None:
                self._resume_timer.cancel()
                self._resume_timer = None
        else:
            # 장애물이 사라져서 mission_people_estop_node가 종료된다는 신호 ->
            # resume_delay_sec만큼 있다가 원래 속도로 복귀.
            self.get_logger().info(f'장애물 해소 — {self.resume_delay_sec}초 뒤 1m/s로 주행 재개')
            self._slow_mode = False
            if self._slow_timer is not None:
                self._slow_timer.cancel()
                self._slow_timer = None
            if self._resume_timer is not None:
                self._resume_timer.cancel()
            self._resume_timer = self.create_timer(self.resume_delay_sec, self._do_resume)

    def _enter_slow_mode(self):
        self._slow_timer.cancel()
        self._slow_timer = None
        self._slow_mode = True
        self.get_logger().info(f'{self.slow_after_sec}초 경과 — {self.slow_speed}m/s로 감속')

    def _do_resume(self):
        self._resume_timer.cancel()
        self._resume_timer = None
        self._blocked = False
        self.get_logger().info('주행 재개')

    def cmd_callback(self, msg: Twist):
        if self._blocked:
            self.cmd_pub.publish(Twist())
            return

        if not self._slow_mode:
            self.cmd_pub.publish(msg)
            return

        if abs(msg.linear.x) < 1e-6:
            self.cmd_pub.publish(msg)
            return

        scale = self.slow_speed / msg.linear.x
        out = Twist()
        out.linear.x = self.slow_speed
        out.angular.z = msg.angular.z * scale
        self.cmd_pub.publish(out)


def main(args=None):
    argv = sys.argv if args is None else args

    pkg = get_package_share_directory('jetracer_ros2')
    params_path = os.path.join(pkg, 'params', 'params.yaml')
    ros_args = ['--ros-args', '--params-file', params_path]

    rclpy.init(args=[argv[0]] + ros_args)
    node = MissionManagerPestopTestNode()
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
