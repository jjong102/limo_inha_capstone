import math
import os

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

# parking_pose_recorder_node가 저장해둔 목표 pose를 읽어서 nav2의 /navigate_to_pose
# 액션에 goal로 보내고, 결과(성공/실패)만 지켜보는 얇은 액션 클라이언트.
# 실제 주행(경로계획, 장애물회피, 제어)은 전부 nav2가 처리한다.
#
# map_server + amcl(로컬라이제이션) + nav2 navigation 스택이 이미 떠 있어야 한다.
#
# 사용법:
#   ros2 run slam_parking parking_navigator_node

DEFAULT_POSE_PATH = '/home/wego/third_impact/src/slam_parking/params/parking_pose.yaml'


def _yaw_to_quaternion_zw(yaw: float):
    """평면 회전(yaw)만 있는 쿼터니언의 z, w 성분 (x=y=0)."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class ParkingNavigatorNode(Node):
    def __init__(self):
        super().__init__('parking_navigator_node')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('pose_path', DEFAULT_POSE_PATH)

        self.map_frame = self.get_parameter('map_frame').value
        self.pose_path = self.get_parameter('pose_path').value

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self) -> bool:
        if not os.path.exists(self.pose_path):
            self.get_logger().error(
                f'{self.pose_path} 가 없습니다. '
                'parking_pose_recorder_node로 목표 pose를 먼저 저장하세요.')
            return False

        with open(self.pose_path) as f:
            pose_data = yaml.safe_load(f)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.map_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose_data['x']
        goal_pose.pose.position.y = pose_data['y']
        qz, qw = _yaw_to_quaternion_zw(pose_data['yaw'])
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        self.get_logger().info(
            f"목표 pose 로드: x={pose_data['x']:.3f}  y={pose_data['y']:.3f}  "
            f"yaw={math.degrees(pose_data['yaw']):.1f}도  ->  nav2로 전송")

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                '/navigate_to_pose 액션 서버를 못 찾았습니다 (nav2 navigation 스택이 안 떠 있는 듯).')
            return False

        send_goal_future = self._client.send_goal_async(
            NavigateToPose.Goal(pose=goal_pose))
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('nav2가 goal을 거부했습니다.')
            return False

        self.get_logger().info('goal 수락됨, 주행 중...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('주차 완료 (SUCCEEDED)')
            return True
        else:
            self.get_logger().error(f'주차 실패 (status={result.status})')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ParkingNavigatorNode()
    try:
        node.send_goal()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
