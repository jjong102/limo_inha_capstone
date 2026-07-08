import math
import os

import rclpy
import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# AMCL(로컬라이제이션)이 이미 떠서 map->base_link TF가 나오고 있는 상태에서,
# 로봇을 원하는 주차 위치/각도에 정확히 갖다 놓고 실행하면 그 순간의 pose(x, y, yaw)를
# 딱 한 번 읽어서 parking_pose.yaml에 저장하고 종료하는 "가르치기" 노드.
#
# 사용법 (map_server + amcl이 이미 실행 중이어야 함):
#   ros2 run slam_parking parking_pose_recorder_node

DEFAULT_SAVE_PATH = '/home/wego/third_impact/src/slam_parking/params/parking_pose.yaml'


def _yaw_from_quaternion(q) -> float:
    """geometry_msgs/Quaternion -> yaw(rad). 로봇은 평면 위에서만 움직이므로 yaw만 필요."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class ParkingPoseRecorderNode(Node):
    def __init__(self):
        super().__init__('parking_pose_recorder_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('save_path', DEFAULT_SAVE_PATH)
        self.declare_parameter('timeout_sec', 5.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.save_path = self.get_parameter('save_path').value
        self.timeout_sec = self.get_parameter('timeout_sec').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def record_once(self) -> bool:
        """map->base_link TF가 잡힐 때까지 기다렸다가 딱 한 번 읽어서 저장한다."""
        deadline = self.get_clock().now() + Duration(seconds=self.timeout_sec)
        while rclpy.ok() and self.get_clock().now() < deadline:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_frame, rclpy.time.Time())
                self._save(tf)
                return True
            except (LookupException, ConnectivityException, ExtrapolationException):
                rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(
            f'{self.timeout_sec}초 안에 {self.map_frame}->{self.base_frame} TF를 못 받았습니다. '
            'map_server + amcl(로컬라이제이션)이 켜져 있는지 확인하세요.')
        return False

    def _save(self, tf):
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = _yaw_from_quaternion(tf.transform.rotation)

        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)
        with open(self.save_path, 'w') as f:
            yaml.safe_dump({'x': x, 'y': y, 'yaw': yaw}, f)

        self.get_logger().info(
            f'주차 목표 pose 저장 완료: x={x:.3f}  y={y:.3f}  '
            f'yaw={math.degrees(yaw):.1f}도  ->  {self.save_path}')


def main(args=None):
    rclpy.init(args=args)
    node = ParkingPoseRecorderNode()
    try:
        node.record_once()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
