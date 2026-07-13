import math
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# 트럭(따라가는 대상)을 라이다로 좌측면에서 계속 감시하는 노드. 전방을 0도로 뒀을 때
# track_angle_deg_range=[min, max](왼쪽이 +) 범위 안에서 track_distance_m보다
# 가까운 포인트가 track_min_points개 이상이면 mission/track_following_state에
# 'stop'을, 아니면 'clear'를 매 스캔마다 실시간 발행한다. 실제 cmd_vel 게이팅은
# 이 노드가 아니라 mission_manager_track_following_test_node가 이 토픽을 구독해서
# 처리한다 (mission_people_estop_node와 같은 "센서 노드" 역할 분리).
#
# mission_people_estop_node와 달리 자기 자신을 종료시키지 않는 상시 모니터링
# 노드다 — 트럭을 계속 따라가는 동안 계속 켜져 있어야 하기 때문.
#
# 사용법:
#   ros2 run jetracer_ros2 mission_track_following_node


class MissionTrackFollowingNode(Node):
    def __init__(self):
        super().__init__('mission_track_following_node')

        self.declare_parameter('track_angle_deg_range', [45.0, 90.0])
        self.declare_parameter('track_distance_m', 0.5)
        self.declare_parameter('track_min_points', 5)

        angle_deg_range = self.get_parameter('track_angle_deg_range').value
        self.track_angle_min = math.radians(angle_deg_range[0])
        self.track_angle_max = math.radians(angle_deg_range[1])
        self.track_distance_m = self.get_parameter('track_distance_m').value
        self.track_min_points = self.get_parameter('track_min_points').value

        self._state = None  # 'stop' | 'clear' | None(아직 미발행) — debug용

        self.state_pub = self.create_publisher(String, 'mission/track_following_state', 10)
        # 라이다 드라이버는 보통 BEST_EFFORT로 발행하므로 반드시 sensor_data QoS로 구독해야 한다.
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'TrackFollowing ready — 좌측 {angle_deg_range}도, {self.track_distance_m}m '
            f'이내 포인트 {self.track_min_points}개 이상이면 stop 발행')

    def _count_track_points(self, msg: LaserScan) -> int:
        count = 0
        angle = msg.angle_min
        for r in msg.ranges:
            if (self.track_angle_min <= angle <= self.track_angle_max
                    and msg.range_min <= r <= self.track_distance_m):
                count += 1
                if count >= self.track_min_points:
                    break
            angle += msg.angle_increment
        return count

    def scan_callback(self, msg: LaserScan):
        count = self._count_track_points(msg)
        obstacle = count >= self.track_min_points
        state = 'stop' if obstacle else 'clear'

        if state != self._state:
            self._state = state
            self.get_logger().info(f'track_following_state -> {state} (포인트 {count}개)')
        self.state_pub.publish(String(data=state))

        self._on_scan(msg, obstacle, count)

    def _on_scan(self, msg, obstacle, count):
        """디버그 시각화 훅 (기본 노드는 아무것도 안 함, debug 서브클래스가 오버라이드)."""
        pass


def main(args=None):
    argv = sys.argv if args is None else args

    pkg = get_package_share_directory('jetracer_ros2')
    params_path = os.path.join(pkg, 'params', 'params.yaml')
    ros_args = ['--ros-args', '--params-file', params_path]

    rclpy.init(args=[argv[0]] + ros_args)
    node = MissionTrackFollowingNode()
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
