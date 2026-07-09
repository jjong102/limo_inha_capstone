import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist

from jetracer_ros2.inference_node import InferenceNode, _extract_plain_arg

# inference_node를 그대로 재사용하되, 실제 /cmd_vel에 직접 쏘지 않고
# inference/cmd_vel로 발행한다. mission_manager_node가 이 토픽을 구독해서
# 속도 비율을 재조정한 뒤 실제 /cmd_vel로 중계한다 — 여러 노드가 동시에
# /cmd_vel에 발행해서 생기는 경쟁 문제(예전 estop 노드에서 겪었던 드드드득 현상)를
# 피하기 위함이다. inference_node.py는 전혀 건드리지 않는다.
#
# 사용법:
#   ros2 run jetracer_ros2 mission_inference_node
#   ros2 run jetracer_ros2 mission_inference_node section_id:=6


class MissionInferenceNode(InferenceNode):
    def __init__(self):
        super().__init__()
        # InferenceNode.image_callback은 self.cmd_pub.publish(...)만 호출하므로,
        # publisher 객체를 바꿔치기하는 것만으로 inference_node.py를 전혀 건드리지
        # 않고 발행 토픽만 바꿀 수 있다.
        self.cmd_pub = self.create_publisher(Twist, 'inference/cmd_vel', 10)
        self.get_logger().info('MissionInference: publishing to inference/cmd_vel')


def main(args=None):
    argv = sys.argv if args is None else args
    section_id = int(_extract_plain_arg(argv, 'section_id', 1))

    pkg = get_package_share_directory('jetracer_ros2')
    params_path = os.path.join(pkg, 'params', 'params.yaml')
    ros_args = ['--ros-args', '--params-file', params_path]

    rclpy.init(args=[argv[0]] + ros_args)
    node = MissionInferenceNode()

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
