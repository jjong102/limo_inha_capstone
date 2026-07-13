import math
import os
import sys

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from jetracer_ros2.utils.ackermann_utils import inner_angle_to_omega

try:
    import tensorrt as trt
    import pycuda.driver as cuda
    import pycuda.autoinit  # noqa: F401
    HAS_TRT = True
except ImportError:
    HAS_TRT = False

try:
    import torch  # noqa: F401
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

_IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
_IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)


class _GoStopTRTModel:
    def __init__(self, engine_path: str, logger):
        TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f:
            runtime = trt.Runtime(TRT_LOGGER)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()

        self.in_name = self.engine.get_tensor_name(0)
        self.out_name = self.engine.get_tensor_name(1)
        in_shape = self.engine.get_tensor_shape(self.in_name)
        out_shape = self.engine.get_tensor_shape(self.out_name)

        self.h_in = cuda.pagelocked_empty(trt.volume(in_shape), dtype=np.float32)
        self.d_in = cuda.mem_alloc(self.h_in.nbytes)
        self.h_out = cuda.pagelocked_empty(trt.volume(out_shape), dtype=np.float32)
        self.d_out = cuda.mem_alloc(self.h_out.nbytes)
        self.stream = cuda.Stream()

        self.context.set_tensor_address(self.in_name, int(self.d_in))
        self.context.set_tensor_address(self.out_name, int(self.d_out))

        logger.info(f'GoStop2 TRT engine loaded  in={in_shape}  out={out_shape}  [{engine_path}]')

    def infer(self, input_tensor: np.ndarray):
        np.copyto(self.h_in, input_tensor.ravel())
        cuda.memcpy_htod_async(self.d_in, self.h_in, self.stream)
        self.context.execute_async_v3(self.stream.handle)
        cuda.memcpy_dtoh_async(self.h_out, self.d_out, self.stream)
        self.stream.synchronize()
        return float(self.h_out[0]), float(self.h_out[1])


class _GoStopTorchModel:
    def __init__(self, pth_path: str, device: str, logger):
        import torch
        import torch.nn as nn
        import torchvision.models as models
        self.device = torch.device(device)
        model = models.resnet18(weights=None)
        model.fc = nn.Linear(model.fc.in_features, 2)
        model.load_state_dict(torch.load(pth_path, map_location=self.device, weights_only=True))
        model.eval()
        self.model = model.to(self.device)
        logger.info(f'GoStop2 PyTorch model loaded  [{pth_path}]')

    def infer(self, input_tensor: np.ndarray):
        import torch
        t = torch.from_numpy(input_tensor).to(self.device)
        with torch.no_grad():
            out = self.model(t)
        vals = out.squeeze(0).cpu().numpy()
        return float(vals[0]), float(vals[1])


def _sigmoid(x: float) -> float:
    return 1.0 / (1.0 + math.exp(-x))


class InferenceGoStop2Node(Node):
    def __init__(self):
        super().__init__('inference_go_stop_2_node')

        self.declare_parameter('engine_dir', '/home/wego/third_impact/src/jetracer_ros2/jetracer_engines')
        self.declare_parameter('linear_x', 1.0)
        self.declare_parameter('max_steer_angle', 0.42)
        self.declare_parameter('crop_ratio', 0.0)
        self.declare_parameter('stop_engine_path', '')
        self.declare_parameter('stop_thresh', 0.85)
        self.declare_parameter('stop_consec', 5)
        self.declare_parameter('stop_min_time', 3.0)
        self.declare_parameter('stop_dry_run', False)
        self.declare_parameter('parking_start_topic', '/parking_start')

        engine_dir = self.get_parameter('engine_dir').value
        self.linear_x = self.get_parameter('linear_x').value
        self.max_steer_angle = self.get_parameter('max_steer_angle').value
        self.crop_ratio = self.get_parameter('crop_ratio').value
        stop_engine_path = self.get_parameter('stop_engine_path').value
        self.stop_thresh = self.get_parameter('stop_thresh').value
        self.stop_consec = self.get_parameter('stop_consec').value
        self.stop_min_time = self.get_parameter('stop_min_time').value
        self.stop_dry_run = self.get_parameter('stop_dry_run').value
        parking_start_topic = self.get_parameter('parking_start_topic').value

        self.model = self._load_model(engine_dir, stop_engine_path)

        self._consec_count = 0
        self._start_time = self.get_clock().now()
        self._driving_enabled = True

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.parking_start_pub = self.create_publisher(Bool, parking_start_topic, 10)
        self.image_sub = self.create_subscription(
            CompressedImage, '/camera/color/image_raw/compressed',
            self.image_callback, 10)

        mode = 'DRY-RUN(로그만)' if self.stop_dry_run else '실제 정지'
        self.get_logger().info(
            f'GoStop2 ready — model_loaded={self.model is not None}  '
            f'thresh={self.stop_thresh}  consec={self.stop_consec}  '
            f'min_time={self.stop_min_time}s  모드={mode}')

    def _load_model(self, engine_dir: str, explicit_path: str):
        if explicit_path:
            engine_path = explicit_path
            pth_path = explicit_path
        else:
            engine_path = os.path.join(engine_dir, 'go_stop_2.engine')
            pth_path = os.path.join(engine_dir, 'go_stop_2.pth')

        if HAS_TRT and os.path.exists(engine_path):
            try:
                return _GoStopTRTModel(engine_path, self.get_logger())
            except Exception as e:
                self.get_logger().error(f'go_stop_2 TRT 로드 실패: {e}')

        if HAS_TORCH and os.path.exists(pth_path):
            import torch
            try:
                device = 'cuda' if torch.cuda.is_available() else 'cpu'
                return _GoStopTorchModel(pth_path, device, self.get_logger())
            except Exception as e:
                self.get_logger().error(f'go_stop_2 PyTorch 로드 실패: {e}')

        self.get_logger().warn(
            f'go_stop_2 모델을 못 찾았습니다 ({engine_path} / {pth_path}).')
        return None

    def image_callback(self, msg: CompressedImage):
        if self.model is None:
            return

        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if image is None:
            return

        h, w = image.shape[:2]
        crop_y = int(h * self.crop_ratio)
        crop = image[crop_y:h, :]
        inp = cv2.resize(crop, (224, 224), interpolation=cv2.INTER_LINEAR)
        inp = inp[:, :, ::-1].astype(np.float32) / 255.0
        inp = (inp - _IMAGENET_MEAN) / _IMAGENET_STD
        inp = inp.transpose(2, 0, 1)[np.newaxis].astype(np.float32)

        try:
            steer_raw, stop_logit = self.model.infer(inp)
        except Exception as e:
            self.get_logger().warn(f'go_stop_2 추론 오류: {e}')
            return

        steer_raw = float(np.clip(steer_raw, -1.0, 1.0))
        inner_angle = steer_raw * self.max_steer_angle
        angular_z = inner_angle_to_omega(inner_angle, self.linear_x)

        cmd = Twist()
        if self._driving_enabled:
            cmd.linear.x = self.linear_x
            cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)

        self._update_stop_decision(_sigmoid(stop_logit))

    def _update_stop_decision(self, prob: float):
        if not self._driving_enabled:
            return

        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if prob >= self.stop_thresh and elapsed >= self.stop_min_time:
            self._consec_count += 1
        else:
            self._consec_count = 0

        if self._consec_count >= self.stop_consec:
            self._on_stop_confirmed(prob)

    def _on_stop_confirmed(self, prob: float):
        if self.stop_dry_run:
            self.get_logger().warn(
                f'[DRY-RUN] STOP 확정됐을 것 (prob={prob:.3f}) — 실제로는 안 멈춤')
            self._consec_count = 0
            return

        self._driving_enabled = False
        self.get_logger().warn(
            f'STOP 확정 (prob={prob:.3f}) — cmd_vel 영구 정지, /parking_start 발행')
        self.cmd_pub.publish(Twist())
        self.parking_start_pub.publish(Bool(data=True))


def main(args=None):
    argv = sys.argv if args is None else args

    pkg = get_package_share_directory('jetracer_ros2')
    params_path = os.path.join(pkg, 'params', 'params.yaml')
    ros_args = ['--ros-args', '--params-file', params_path]

    rclpy.init(args=[argv[0]] + ros_args)
    node = InferenceGoStop2Node()
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
