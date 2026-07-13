import math
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# inference_node(또는 다른 주행 노드)가 이미 따로 실행 중인 상황에서 이 노드
# 하나만 추가로 띄워서, 라이다로 전방에 사람/장애물이 감지되면 강제로 멈추게 하는
# 독립 e-stop 노드.
#
# inference_StopAndGo_with_lidar_node와 달리 InferenceNode를 상속해서 publisher를
# 바꿔치기하는 방식을 쓸 수 없다 — 완전히 별도 프로세스라서 그렇다. 대신 같은
# /cmd_vel 토픽에 이 노드도 발행자로 붙어서, ROS 토픽은 "마지막에 발행된 값이
# 이긴다"는 점을 이용한다: 장애물이 감지되면 obstacle_stop_hold_sec 동안 0을
# inference_node보다 훨씬 빠른 주기로 계속 내보내 그쪽 발행을 계속 덮어쓰다가,
# 그 시간이 지나면 이 노드는 스스로 종료(die)한다 — 그 뒤로는 이 노드가 더 이상
# /cmd_vel에 아무것도 안 보내니 inference_node가 다시 정상적으로 주행을 이어간다.
#
# 한 번 감지 → 정지 → 종료 하는 1회성 노드다. 계속 감시하고 싶으면 다시
# ros2 run으로 띄워야 한다.
#
# 사용법 (inference_node가 이미 실행 중이어야 함):
#   ros2 run jetracer_ros2 inference_people_estop_node


class InferencePeopleEstopNode(Node):
    def __init__(self):
        super().__init__('inference_people_estop_node')

        self.declare_parameter('obstacle_angle_deg_range', [-10.0, 10.0])
        self.declare_parameter('obstacle_distance_m', 1.0)
        self.declare_parameter('obstacle_min_points', 1)
        self.declare_parameter('obstacle_stop_hold_sec', 3.0)

        angle_deg_range = self.get_parameter('obstacle_angle_deg_range').value
        self.obstacle_angle_min = math.radians(angle_deg_range[0])
        self.obstacle_angle_max = math.radians(angle_deg_range[1])
        self.obstacle_distance_m = self.get_parameter('obstacle_distance_m').value
        self.obstacle_min_points = self.get_parameter('obstacle_min_points').value
        self.obstacle_stop_hold_sec = self.get_parameter('obstacle_stop_hold_sec').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/inference/drive_status', 10)
        # 라이다 드라이버는 보통 BEST_EFFORT로 발행하므로 반드시 sensor_data QoS로 구독해야 한다.
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)

        self._triggered = False
        self._override_timer = None
        self._stop_deadline = None

        self.get_logger().info(
            f'PeopleEstop ready — 전방 {angle_deg_range}도, '
            f'{self.obstacle_distance_m}m 이내 포인트 {self.obstacle_min_points}개 이상 '
            f'감지되면 {self.obstacle_stop_hold_sec}초간 cmd_vel=0 강제 후 종료')

    def scan_callback(self, msg: LaserScan):
        if self._triggered:
            return  # 이미 발동돼서 곧 종료될 노드 — 더 볼 필요 없음

        count = 0
        angle = msg.angle_min
        for r in msg.ranges:
            if (self.obstacle_angle_min <= angle <= self.obstacle_angle_max
                    and msg.range_min <= r <= self.obstacle_distance_m):
                count += 1
                if count >= self.obstacle_min_points:
                    break
            angle += msg.angle_increment

        if count >= self.obstacle_min_points:
            self._trigger_estop()

    def _trigger_estop(self):
        self._triggered = True
        self.status_pub.publish(String(data='stop'))
        self.get_logger().warn(
            f'ESTOP — 전방 장애물 감지, {self.obstacle_stop_hold_sec}초간 cmd_vel=0 강제')

        self._stop_deadline = self.get_clock().now() + Duration(
            seconds=self.obstacle_stop_hold_sec)
        # inference_node가 카메라 프레임마다(대략 30Hz) cmd_vel을 새로 발행하므로,
        # 그보다 확실히 빠른 주기로 0을 계속 내보내야 마지막 값이 항상 우리 것이 된다.
        self._override_timer = self.create_timer(0.02, self._override_tick)

    def _override_tick(self):
        self.cmd_pub.publish(Twist())
        if self.get_clock().now() >= self._stop_deadline:
            self._override_timer.cancel()
            self.status_pub.publish(String(data='start'))
            self.get_logger().warn(
                f'ESTOP 해제 ({self.obstacle_stop_hold_sec}초 경과) — 제어권 반환, 노드 종료')
            rclpy.shutdown()


def main(args=None):
    argv = sys.argv if args is None else args

    pkg = get_package_share_directory('jetracer_ros2')
    params_path = os.path.join(pkg, 'params', 'params.yaml')
    ros_args = ['--ros-args', '--params-file', params_path]

    rclpy.init(args=[argv[0]] + ros_args)
    node = InferencePeopleEstopNode()
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
