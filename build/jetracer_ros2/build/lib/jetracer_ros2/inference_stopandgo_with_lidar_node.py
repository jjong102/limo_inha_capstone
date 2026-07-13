import math
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from jetracer_ros2.inference_node import InferenceNode, _extract_plain_arg

# inference_node를 그대로 재사용하되, 라이다(/scan)로 전방 장애물을 감지해서
# 자율주행 cmd_vel 발행만 멈췄다 다시 시작했다 할 수 있게 한 노드
# (inference_node.py는 건드리지 않음).
#
# 전방을 0도로 뒀을 때 obstacle_angle_deg_range=[min, max] 범위 안에서,
# obstacle_distance_m보다 가까운 포인트가 obstacle_min_points개 이상이면
# 장애물로 판단해 정지한다 (점 하나만 잡혔다고 바로 멈추지 않도록 개수 조건을 둠).
# 세 값 모두 params.yaml에서 조정 가능하다.
#
# 장애물 감지 → /inference/drive_status에 'stop' 발행, cmd_vel = 0
# 장애물 사라짐 → obstacle_stop_hold_sec만큼 계속 안 잡힌 뒤에야 /inference/drive_status에
# 'start' 발행, cmd_vel 재개 (한 번 감지되면 바로 재개하지 않고 그만큼은 무조건 대기 —
# 포인트가 깜빡거려서 정지/재개가 반복되는 걸 막기 위함).
# 정지 중에도 모델 추론(이미지 → 조향 예측)은 계속 돈다 — 실제 /cmd_vel 발행만 막는다.
# 조이스틱 버전과 달리 사람이 시작 버튼을 누를 필요 없이 기본 주행 상태로 시작한다.
#
# section_id:=N 을 주면 시작할 때부터 해당 구간 모델로 추론한다 (inference_node와 동일).
#
# 사용법:
#   ros2 run jetracer_ros2 inference_StopAndGo_with_lidar_node
#   ros2 run jetracer_ros2 inference_StopAndGo_with_lidar_node section_id:=2


class _GatedPublisher:
    """실제 cmd_vel publisher를 감싸서, enabled=False면 무조건 정지(Twist())로 내보낸다."""

    def __init__(self, real_publisher):
        self._real = real_publisher
        self.enabled = True

    def publish(self, msg: Twist):
        self._real.publish(msg if self.enabled else Twist())


class InferenceStopAndGoLidarNode(InferenceNode):
    def __init__(self):
        super().__init__()

        # 전방(0도) 기준 [min, max] 각도 범위, 이 거리보다 가까운 포인트가
        # 몇 개 이상이어야 장애물로 볼지
        self.declare_parameter('obstacle_angle_deg_range', [-2.5, 2.5])
        self.declare_parameter('obstacle_distance_m', 0.15)
        self.declare_parameter('obstacle_min_points', 3)
        # 장애물이 마지막으로 감지된 뒤 이만큼(초) 계속 안 잡혀야 재개한다.
        self.declare_parameter('obstacle_stop_hold_sec', 3.0)

        angle_deg_range = self.get_parameter('obstacle_angle_deg_range').value
        self.obstacle_angle_min = math.radians(angle_deg_range[0])
        self.obstacle_angle_max = math.radians(angle_deg_range[1])
        self.obstacle_distance_m = self.get_parameter('obstacle_distance_m').value
        self.obstacle_min_points = self.get_parameter('obstacle_min_points').value
        self.obstacle_stop_hold_sec = self.get_parameter('obstacle_stop_hold_sec').value
        self._last_obstacle_time = None

        # InferenceNode.image_callback은 self.cmd_pub.publish(...)만 호출하므로,
        # publisher 객체를 바꿔치기하는 것만으로 inference_node.py 코드를 전혀
        # 건드리지 않고 발행을 게이팅할 수 있다.
        self._gated_pub = _GatedPublisher(self.cmd_pub)
        self.cmd_pub = self._gated_pub

        self.status_pub = self.create_publisher(String, '/inference/drive_status', 10)
        # 라이다 드라이버는 보통 BEST_EFFORT로 발행한다. 기본(RELIABLE) QoS로 구독하면
        # QoS 불일치로 메시지를 아예 못 받으므로 반드시 sensor_data 프로파일을 써야 한다.
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'StopAndGo(lidar) ready — 전방 {angle_deg_range}도, '
            f'{self.obstacle_distance_m}m 이내 포인트 {self.obstacle_min_points}개 이상이면 정지')

    def scan_callback(self, msg: LaserScan):
        count = 0
        angle = msg.angle_min
        for r in msg.ranges:
            if (self.obstacle_angle_min <= angle <= self.obstacle_angle_max
                    and msg.range_min <= r <= self.obstacle_distance_m):
                count += 1
                if count >= self.obstacle_min_points:
                    break
            angle += msg.angle_increment

        obstacle = count >= self.obstacle_min_points
        now = self.get_clock().now()

        if obstacle:
            self._last_obstacle_time = now
            if self._gated_pub.enabled:
                self._gated_pub.enabled = False
                self.status_pub.publish(String(data='stop'))
                self.get_logger().info(
                    f'STOP (lidar) — 전방 장애물 감지 (포인트 {count}개, 추론은 계속 동작)')
                self._gated_pub.publish(Twist())
            return

        if not self._gated_pub.enabled and self._last_obstacle_time is not None:
            elapsed_sec = (now - self._last_obstacle_time).nanoseconds / 1e9
            if elapsed_sec >= self.obstacle_stop_hold_sec:
                self._gated_pub.enabled = True
                self.status_pub.publish(String(data='start'))
                self.get_logger().info('START (lidar) — 장애물 사라짐, 주행 재개')


def main(args=None):
    argv = sys.argv if args is None else args
    section_id = int(_extract_plain_arg(argv, 'section_id', 1))

    pkg = get_package_share_directory('jetracer_ros2')
    params_path = os.path.join(pkg, 'params', 'params.yaml')
    ros_args = ['--ros-args', '--params-file', params_path]

    rclpy.init(args=[argv[0]] + ros_args)
    node = InferenceStopAndGoLidarNode()

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


if __name__ == '__main__':
    main()
