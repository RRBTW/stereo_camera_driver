"""
Universal stereo camera node.

Поддерживает два режима:

  1. TWO-DEVICE (default, single_device=False):
       Два отдельных USB-устройства.
       left_camera_path  — путь к левой камере
       right_camera_path — путь к правой камере
       Захват: frame_width x frame_height с каждой.

  2. SINGLE-DEVICE (single_device=True):
       Одно USB-устройство с двойным кадром side-by-side
       (напр. Waveshare IMX219-83, ELP двойная камера).
       left_camera_path — путь к устройству (right_camera_path игнорируется).
       Захват: (frame_width*2) x frame_height, затем кадр разрезается пополам.

Параметры:
  left_camera_path  (string) -- путь к левой камере  (default: /dev/video0)
  right_camera_path (string) -- путь к правой камере (default: /dev/video2)
  single_device     (bool)   -- side-by-side режим    (default: false)
  camera_fps        (double) -- желаемый FPS           (default: 30.0)
  frame_width       (int)    -- ширина одного глаза    (default: 640)
  frame_height      (int)    -- высота кадра           (default: 480)
  frame_id_left     (string) -- TF frame левой камеры  (default: left_camera_optical)
  frame_id_right    (string) -- TF frame правой камеры (default: right_camera_optical)

Публикуемые топики:
  left/image_raw    (sensor_msgs/Image)
  left/camera_info  (sensor_msgs/CameraInfo)
  right/image_raw   (sensor_msgs/Image)
  right/camera_info (sensor_msgs/CameraInfo)
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

import cv2
from cv_bridge import CvBridge


def _make_default_camera_info(frame_id: str, width: int, height: int) -> CameraInfo:
    """Возвращает CameraInfo с нулевой калибровкой — заполните реальными значениями."""
    msg = CameraInfo()
    msg.header.frame_id = frame_id
    msg.width  = width
    msg.height = height
    msg.distortion_model = 'plumb_bob'
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
        self.declare_parameter('right_camera_path', '/dev/video2')
        self.declare_parameter('single_device',  False)
        self.declare_parameter('camera_fps',   30.0)
        self.declare_parameter('frame_width',  640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('frame_id_left',  'left_camera_optical')
        self.declare_parameter('frame_id_right', 'right_camera_optical')

        left_path   = self.get_parameter('left_camera_path').value
        right_path  = self.get_parameter('right_camera_path').value
        self._single = self.get_parameter('single_device').value
        fps          = self.get_parameter('camera_fps').value
        width        = self.get_parameter('frame_width').value
        height       = self.get_parameter('frame_height').value
        self._frame_id_left  = self.get_parameter('frame_id_left').value
        self._frame_id_right = self.get_parameter('frame_id_right').value

        self._bridge = CvBridge()

        # --- открытие захвата ---
        if self._single:
            self.get_logger().info(
                f'Mode: SINGLE-DEVICE side-by-side  path={left_path}')
            # Захватываем двойной кадр: ширина удваивается
            self._cap = self._open_capture(left_path, width * 2, height, fps)
            actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self._mid = actual_w // 2   # точка разреза
            eye_w = self._mid
            eye_h = actual_h
            self.get_logger().info(
                f'Capture: {actual_w}x{actual_h}  each eye: {eye_w}x{eye_h} @ {fps} fps')
        else:
            self.get_logger().info(
                f'Mode: TWO-DEVICE  left={left_path}  right={right_path}')
            self._cap_left  = self._open_capture(left_path,  width, height, fps)
            self._cap_right = self._open_capture(right_path, width, height, fps)
            eye_w = int(self._cap_left.get(cv2.CAP_PROP_FRAME_WIDTH))
            eye_h = int(self._cap_left.get(cv2.CAP_PROP_FRAME_HEIGHT))
            right_w = int(self._cap_right.get(cv2.CAP_PROP_FRAME_WIDTH))
            right_h = int(self._cap_right.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(
                f'Left:  {eye_w}x{eye_h}  Right: {right_w}x{right_h} @ {fps} fps')
            # CameraInfo для правой камеры по её реальному разрешению
            self._info_right = _make_default_camera_info(
                self._frame_id_right, right_w, right_h)

        # --- публикаторы ---
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_left_img   = self.create_publisher(Image,      'left/image_raw',    qos)
        self._pub_left_info  = self.create_publisher(CameraInfo, 'left/camera_info',  qos)
        self._pub_right_img  = self.create_publisher(Image,      'right/image_raw',   qos)
        self._pub_right_info = self.create_publisher(CameraInfo, 'right/camera_info', qos)

        # --- шаблоны CameraInfo ---
        self._info_left = _make_default_camera_info(
            self._frame_id_left, eye_w, eye_h)
        if self._single:
            self._info_right = _make_default_camera_info(
                self._frame_id_right, eye_w, eye_h)

        # --- таймер ---
        self._timer = self.create_timer(1.0 / fps, self._capture_and_publish)
        self.get_logger().info('Stereo camera node ready.')

    # ------------------------------------------------------------------
    def _open_capture(self, path: str, width: int, height: int,
                      fps: float) -> cv2.VideoCapture:
        src = int(path) if path.lstrip('-').isdigit() else path
        cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f'Cannot open camera: {path}')
            raise RuntimeError(f'Cannot open camera: {path}')
        # MJPG даёт 30 fps на большинстве камер, YUYV может быть ограничен 15 fps
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS,          fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
        return cap

    # ------------------------------------------------------------------
    def _capture_and_publish(self):
        stamp = self.get_clock().now().to_msg()

        if self._single:
            ret, frame = self._cap.read()
            if not ret:
                self.get_logger().warn('Failed to grab frame from camera')
                return
            frame_l = frame[:, :self._mid]
            frame_r = frame[:, self._mid:]
        else:
            ret_l, frame_l = self._cap_left.read()
            ret_r, frame_r = self._cap_right.read()
            if not ret_l:
                self.get_logger().warn('Failed to grab frame from LEFT camera')
            if not ret_r:
                self.get_logger().warn('Failed to grab frame from RIGHT camera')
            if not (ret_l and ret_r):
                return

        # --- левая ---
        header_l = Header()
        header_l.stamp    = stamp
        header_l.frame_id = self._frame_id_left

        img_l = self._bridge.cv2_to_imgmsg(frame_l, encoding='bgr8')
        img_l.header = header_l
        self._pub_left_img.publish(img_l)

        self._info_left.header = header_l
        self._pub_left_info.publish(self._info_left)

        # --- правая ---
        header_r = Header()
        header_r.stamp    = stamp          # тот же stamp для синхронизации
        header_r.frame_id = self._frame_id_right

        img_r = self._bridge.cv2_to_imgmsg(frame_r, encoding='bgr8')
        img_r.header = header_r
        self._pub_right_img.publish(img_r)

        self._info_right.header = header_r
        self._pub_right_info.publish(self._info_right)

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.get_logger().info('Releasing cameras...')
        if self._single:
            if self._cap.isOpened():
                self._cap.release()
        else:
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
