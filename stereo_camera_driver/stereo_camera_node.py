"""
Universal stereo camera node.

Параметры (задаются снаружи через launch или командную строку):
  left_camera_path  (string) -- путь к левой камере,  напр. /dev/v4l/by-path/...
  right_camera_path (string) -- путь к правой камере, напр. /dev/v4l/by-path/...
  camera_fps        (double) -- желаемый FPS захвата (default: 30.0)
  frame_width       (int)    -- ширина кадра в пикселях (default: 640)
  frame_height      (int)    -- высота кадра в пикселях (default: 480)
  frame_id_left     (string) -- TF frame для левой камеры  (default: "left_camera_optical")
  frame_id_right    (string) -- TF frame для правой камеры (default: "right_camera_optical")

Публикуемые топики (относительно namespace ноды):
  left/image_raw    (sensor_msgs/Image)
  left/camera_info  (sensor_msgs/CameraInfo)
  right/image_raw   (sensor_msgs/Image)
  right/camera_info (sensor_msgs/CameraInfo)
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

import cv2
from cv_bridge import CvBridge

from builtin_interfaces.msg import Time as RosTime


def _make_default_camera_info(frame_id: str, width: int, height: int) -> CameraInfo:
    """Возвращает CameraInfo с нулевой калибровкой — заполните реальными значениями."""
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width = width
    msg.height = height
    msg.distortion_model = 'plumb_bob'
    # K, D, R, P — нули; перекалибруйте с помощью camera_calibration
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.k = [0.0, 0.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    msg.p = [0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0]
    return msg


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__('stereo_camera_node')

        # --- параметры ---
        self.declare_parameter('left_camera_path',  '/dev/video0')
        self.declare_parameter('right_camera_path', '/dev/video1')
        self.declare_parameter('camera_fps',   30.0)
        self.declare_parameter('frame_width',  640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('frame_id_left',  'left_camera_optical')
        self.declare_parameter('frame_id_right', 'right_camera_optical')

        left_path   = self.get_parameter('left_camera_path').value
        right_path  = self.get_parameter('right_camera_path').value
        fps         = self.get_parameter('camera_fps').value
        width       = self.get_parameter('frame_width').value
        height      = self.get_parameter('frame_height').value
        self._frame_id_left  = self.get_parameter('frame_id_left').value
        self._frame_id_right = self.get_parameter('frame_id_right').value

        self.get_logger().info(f'Left  camera: {left_path}')
        self.get_logger().info(f'Right camera: {right_path}')

        # --- cv_bridge ---
        self._bridge = CvBridge()

        # --- открытие захвата ---
        self._cap_left  = self._open_capture(left_path,  width, height, fps)
        self._cap_right = self._open_capture(right_path, width, height, fps)

        # --- публикаторы ---
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub_left_img  = self.create_publisher(Image,      'left/image_raw',   qos)
        self._pub_left_info = self.create_publisher(CameraInfo, 'left/camera_info',  qos)
        self._pub_right_img  = self.create_publisher(Image,      'right/image_raw',  qos)
        self._pub_right_info = self.create_publisher(CameraInfo, 'right/camera_info', qos)

        # --- шаблоны CameraInfo (без калибровки) ---
        actual_w_l = int(self._cap_left.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h_l = int(self._cap_left.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_w_r = int(self._cap_right.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h_r = int(self._cap_right.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self._info_left  = _make_default_camera_info(self._frame_id_left,  actual_w_l, actual_h_l)
        self._info_right = _make_default_camera_info(self._frame_id_right, actual_w_r, actual_h_r)

        # --- таймер захвата ---
        timer_period = 1.0 / fps
        self._timer = self.create_timer(timer_period, self._capture_and_publish)

        self.get_logger().info(
            f'Stereo camera node started  '
            f'left={actual_w_l}x{actual_h_l}  '
            f'right={actual_w_r}x{actual_h_r}  '
            f'@ {fps} fps'
        )

    # ------------------------------------------------------------------
    def _open_capture(self, path: str, width: int, height: int, fps: float) -> cv2.VideoCapture:
        """Открывает V4L2-устройство и задаёт параметры захвата."""
        cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f'Cannot open camera: {path}')
            raise RuntimeError(f'Cannot open camera: {path}')

        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        # Отключить буферизацию — получать свежий кадр без задержки
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    # ------------------------------------------------------------------
    def _now_header(self, frame_id: str) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        return header

    # ------------------------------------------------------------------
    def _capture_and_publish(self):
        # Сначала вычитываем оба кадра, затем берём единый timestamp.
        # Это важно для approx_sync: False в stereo_sync — оба сообщения
        # пары должны иметь одинаковый stamp.
        ret_l, frame_l = self._cap_left.read()
        ret_r, frame_r = self._cap_right.read()

        stamp = self.get_clock().now().to_msg()

        if not ret_l:
            self.get_logger().warn('Failed to grab frame from LEFT camera')
        if not ret_r:
            self.get_logger().warn('Failed to grab frame from RIGHT camera')

        if not (ret_l and ret_r):
            # Публикуем только полную пару — stereo_sync не примет неполную
            return

        # --- левая ---
        header_l = Header()
        header_l.stamp = stamp
        header_l.frame_id = self._frame_id_left

        img_l = self._bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
        img_l.header = header_l
        self._pub_left_img.publish(img_l)

        self._info_left.header = header_l
        self._pub_left_info.publish(self._info_left)

        # --- правая ---
        header_r = Header()
        header_r.stamp = stamp          # тот же stamp!
        header_r.frame_id = self._frame_id_right

        img_r = self._bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')
        img_r.header = header_r
        self._pub_right_img.publish(img_r)

        self._info_right.header = header_r
        self._pub_right_info.publish(self._info_right)

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info('Releasing cameras...')
        if self._cap_left.isOpened():
            self._cap_left.release()
        if self._cap_right.isOpened():
            self._cap_right.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
