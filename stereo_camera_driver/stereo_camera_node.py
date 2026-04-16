"""
Stereo camera node — один USB-девайс, MJPG side-by-side.

Каждая физическая стереокамера выдаёт один широкий кадр (напр. 1280×480),
в котором левая половина — левый объектив, правая — правый.
Нода открывает устройство через прямые V4L2 ioctl (обходит сброс формата
OpenCV), делит кадр пополам и публикует 4 топика:

  left/image_raw    (sensor_msgs/Image)
  left/camera_info  (sensor_msgs/CameraInfo)
  right/image_raw   (sensor_msgs/Image)
  right/camera_info (sensor_msgs/CameraInfo)

При отключении камеры нода продолжает работать и переподключается
автоматически, как только устройство снова появится в системе.

Параметры:
  device_path    (string) -- путь к устройству, напр. /dev/stereo_cam0
  camera_fps     (double) -- FPS (default: 30.0)
  frame_width    (int)    -- ширина широкого кадра (default: 1280)
  frame_height   (int)    -- высота кадра           (default: 480)
  frame_id_left  (string) -- TF frame левой камеры  (default: left_camera_optical)
  frame_id_right (string) -- TF frame правой камеры (default: right_camera_optical)
  reconnect_sec  (double) -- пауза между попытками реконнекта (default: 2.0)
"""

import fcntl
import mmap
import os
import select as _select
import struct
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header


# ---------------------------------------------------------------------------
# Прямой V4L2 MJPG захват
# ---------------------------------------------------------------------------

class MJPGCapture:
    """Захватывает MJPG кадры через прямые V4L2 ioctl + mmap."""

    _BUF_TYPE = 1
    _MEMORY   = 1
    _MJPG     = 0x47504A4D

    _S_FMT    = 0xC0D05605
    _S_PARM   = 0xC0CC5616
    _REQBUFS  = 0xC0145608
    _QUERYBUF = 0xC0585609
    _QBUF     = 0xC058560F
    _DQBUF    = 0xC0585611
    _STREAMON = 0x40045612

    _OFF_WIDTH  = 8
    _OFF_HEIGHT = 12
    _OFF_PIXFMT = 16
    _OFF_FIELD  = 20

    _OFF_IDX       = 0
    _OFF_TYPE_BUF  = 4
    _OFF_BYTESUSED = 8
    _OFF_MEMORY    = 60
    _OFF_M_OFFSET  = 64
    _OFF_LENGTH    = 72

    def __init__(self, device: str, width: int, height: int,
                 fps: float = 30.0, n_bufs: int = 2):
        self.width  = width
        self.height = height
        self._mmaps: list[tuple[mmap.mmap, int]] = []
        self.fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        self._set_fmt(width, height)
        self._set_fps(fps)
        self._init_buffers(n_bufs)
        self._stream_on()

    def _ioctl(self, req: int, buf: bytearray) -> None:
        fcntl.ioctl(self.fd, req, buf)

    def _set_fmt(self, w: int, h: int) -> None:
        buf = bytearray(208)
        struct.pack_into('=I',    buf, 0, self._BUF_TYPE)
        struct.pack_into('=IIII', buf, self._OFF_WIDTH, w, h, self._MJPG, 0)
        self._ioctl(self._S_FMT, buf)
        aw = struct.unpack_from('I', buf, self._OFF_WIDTH)[0]
        ah = struct.unpack_from('I', buf, self._OFF_HEIGHT)[0]
        pf = struct.unpack_from('I', buf, self._OFF_PIXFMT)[0]
        ok = pf == self._MJPG and aw == w and ah == h
        status = 'OK' if ok else f'WARN: got {aw}x{ah} fmt={hex(pf)}'
        print(f'  MJPGCapture {w}x{h}: {status}')

    def _set_fps(self, fps: float) -> None:
        buf = bytearray(204)
        struct.pack_into('=I',  buf, 0,  self._BUF_TYPE)
        struct.pack_into('=II', buf, 12, 1, max(1, round(fps)))
        try:
            self._ioctl(self._S_PARM, buf)
        except OSError:
            pass

    def _init_buffers(self, n: int) -> None:
        rb = bytearray(20)
        struct.pack_into('=III', rb, 0, n, self._BUF_TYPE, self._MEMORY)
        self._ioctl(self._REQBUFS, rb)
        count = struct.unpack_from('I', rb)[0]
        for i in range(count):
            qb = bytearray(88)
            struct.pack_into('=II', qb, self._OFF_IDX, i, self._BUF_TYPE)
            self._ioctl(self._QUERYBUF, qb)
            length = struct.unpack_from('I', qb, self._OFF_LENGTH)[0]
            offset = struct.unpack_from('I', qb, self._OFF_M_OFFSET)[0]
            mm = mmap.mmap(self.fd, length,
                           flags=mmap.MAP_SHARED,
                           prot=mmap.PROT_READ | mmap.PROT_WRITE,
                           offset=offset)
            self._mmaps.append((mm, length))
            self._qbuf(i)

    def _qbuf(self, idx: int) -> None:
        buf = bytearray(88)
        struct.pack_into('=II', buf, self._OFF_IDX, idx, self._BUF_TYPE)
        struct.pack_into('I',   buf, self._OFF_MEMORY, self._MEMORY)
        self._ioctl(self._QBUF, buf)

    def _stream_on(self) -> None:
        self._ioctl(self._STREAMON,
                    bytearray(struct.pack('=I', self._BUF_TYPE)))

    def read(self) -> tuple[bool, np.ndarray | None]:
        """
        Возвращает (True, frame) при успехе.
        Возвращает (False, None) при таймауте select (нормально, не ошибка).
        Поднимает OSError при ошибке устройства (отвал USB и т.п.).
        """
        if not _select.select([self.fd], [], [], 1.0)[0]:
            return False, None          # таймаут — не ошибка
        db = bytearray(88)
        struct.pack_into('=II', db, self._OFF_IDX, 0, self._BUF_TYPE)
        struct.pack_into('I',   db, self._OFF_MEMORY, self._MEMORY)
        self._ioctl(self._DQBUF, db)    # поднимет OSError если устройство пропало
        idx       = struct.unpack_from('I', db, self._OFF_IDX)[0]
        bytesused = struct.unpack_from('I', db, self._OFF_BYTESUSED)[0]
        mm, _     = self._mmaps[idx]
        mm.seek(0)
        frame = cv2.imdecode(
            np.frombuffer(mm.read(bytesused), np.uint8), cv2.IMREAD_COLOR)
        self._qbuf(idx)
        return (True, frame) if frame is not None else (False, None)

    def release(self) -> None:
        for mm, _ in self._mmaps:
            try:
                mm.close()
            except Exception:
                pass
        if self.fd >= 0:
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = -1
        self._mmaps.clear()


# ---------------------------------------------------------------------------
# Фоновый поток захвата с авторекконектом
# ---------------------------------------------------------------------------

class _FrameGrabber:
    """
    Непрерывно читает кадры из MJPGCapture в фоновом потоке.
    При отвале камеры закрывает устройство и пробует переоткрыть его
    каждые `reconnect_sec` секунд.

    Состояние доступно через свойство `connected`.
    При смене connected вызывается `on_connect(True/False)` — необязательный
    callback для логирования из ноды.
    """

    # Сколько подряд таймаутов select считается отвалом
    _TIMEOUT_THRESHOLD = 5

    def __init__(
        self,
        device: str,
        width: int,
        height: int,
        fps: float,
        reconnect_sec: float = 2.0,
        on_connect=None,          # callable(bool) | None
    ) -> None:
        self._device        = device
        self._width         = width
        self._height        = height
        self._fps           = fps
        self._reconnect_sec = reconnect_sec
        self._on_connect    = on_connect

        self._cap: MJPGCapture | None = None
        self._frame: np.ndarray | None = None
        self._lock       = threading.Lock()
        self._stop       = False
        self._connected  = False
        self._timeouts   = 0

        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    # ------------------------------------------------------------------
    def _open(self) -> bool:
        try:
            cap = MJPGCapture(self._device, self._width, self._height, self._fps)
        except OSError:
            return False
        self._cap      = cap
        self._timeouts = 0
        self._set_connected(True)
        return True

    def _close(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        self._timeouts = 0
        self._set_connected(False)

    def _set_connected(self, value: bool) -> None:
        if self._connected == value:
            return
        self._connected = value
        if self._on_connect is not None:
            try:
                self._on_connect(value)
            except Exception:
                pass

    # ------------------------------------------------------------------
    def _loop(self) -> None:
        while not self._stop:
            # Нет устройства — пробуем открыть
            if self._cap is None:
                if not self._open():
                    time.sleep(self._reconnect_sec)
                continue

            # Читаем кадр
            try:
                ret, frame = self._cap.read()
            except OSError:
                # Устройство пропало
                self._close()
                continue

            if ret and frame is not None:
                with self._lock:
                    self._frame = frame
                self._timeouts = 0
            else:
                # Таймаут select — считаем
                self._timeouts += 1
                if self._timeouts >= self._TIMEOUT_THRESHOLD:
                    self._close()

    # ------------------------------------------------------------------
    @property
    def connected(self) -> bool:
        return self._connected

    def read(self) -> tuple[bool, np.ndarray | None]:
        with self._lock:
            return (self._connected and self._frame is not None), self._frame

    def stop(self) -> None:
        self._stop = True
        self._close()


# ---------------------------------------------------------------------------
# Утилита: split_stereo
# ---------------------------------------------------------------------------

def split_stereo(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Разрезает wide side-by-side кадр пополам → (left, right)."""
    mid = frame.shape[1] // 2
    return frame[:, :mid], frame[:, mid:]


# ---------------------------------------------------------------------------
# ROS2-нода
# ---------------------------------------------------------------------------

def _make_default_camera_info(frame_id: str, width: int, height: int) -> CameraInfo:
    """CameraInfo с нулевой калибровкой — заполните реальными значениями."""
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
        self.declare_parameter('device_path',    '/dev/stereo_cam0')
        self.declare_parameter('camera_fps',     30.0)
        self.declare_parameter('frame_width',    1280)
        self.declare_parameter('frame_height',   480)
        self.declare_parameter('frame_id_left',  'left_camera_optical')
        self.declare_parameter('frame_id_right', 'right_camera_optical')
        self.declare_parameter('reconnect_sec',  2.0)

        device        = self.get_parameter('device_path').value
        fps           = self.get_parameter('camera_fps').value
        width         = self.get_parameter('frame_width').value
        height        = self.get_parameter('frame_height').value
        self._fid_left  = self.get_parameter('frame_id_left').value
        self._fid_right = self.get_parameter('frame_id_right').value
        reconnect_sec = self.get_parameter('reconnect_sec').value

        self.get_logger().info(
            f'Stereo device: {device}  {width}x{height} @ {fps} fps  '
            f'reconnect_sec={reconnect_sec}'
        )

        # --- фоновый захват с авторекконектом ---
        self._grabber = _FrameGrabber(
            device=device,
            width=width,
            height=height,
            fps=fps,
            reconnect_sec=reconnect_sec,
            on_connect=self._on_connect_change,
        )

        # --- cv_bridge ---
        self._bridge = CvBridge()

        # --- публикаторы ---
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub_left_img   = self.create_publisher(Image,      'left/image_raw',    qos)
        self._pub_left_info  = self.create_publisher(CameraInfo, 'left/camera_info',  qos)
        self._pub_right_img  = self.create_publisher(Image,      'right/image_raw',   qos)
        self._pub_right_info = self.create_publisher(CameraInfo, 'right/camera_info', qos)

        # --- CameraInfo шаблоны ---
        half_w = width // 2
        self._info_left  = _make_default_camera_info(self._fid_left,  half_w, height)
        self._info_right = _make_default_camera_info(self._fid_right, half_w, height)

        # --- таймер публикации ---
        self._timer = self.create_timer(1.0 / fps, self._capture_and_publish)

        self.get_logger().info(
            f'Ready: {device}  left/right each {half_w}x{height} @ {fps} fps'
        )

    # ------------------------------------------------------------------
    def _on_connect_change(self, connected: bool) -> None:
        """Callback из потока grabber-а при смене состояния соединения."""
        if connected:
            self.get_logger().info(
                f'Camera connected: {self.get_parameter("device_path").value}')
        else:
            self.get_logger().warn(
                f'Camera disconnected: {self.get_parameter("device_path").value}'
                f'  — ожидаю переподключения...')

    # ------------------------------------------------------------------
    def _capture_and_publish(self) -> None:
        ok, frame = self._grabber.read()
        if not ok or frame is None:
            # Камера ещё не подключена / временный сбой — молчим
            return

        left, right = split_stereo(frame)
        stamp = self.get_clock().now().to_msg()

        header_l = Header()
        header_l.stamp    = stamp
        header_l.frame_id = self._fid_left

        header_r = Header()
        header_r.stamp    = stamp       # одинаковый stamp для stereo_sync
        header_r.frame_id = self._fid_right

        img_l = self._bridge.cv2_to_imgmsg(left,  encoding='bgr8')
        img_l.header = header_l
        self._pub_left_img.publish(img_l)

        self._info_left.header = header_l
        self._pub_left_info.publish(self._info_left)

        img_r = self._bridge.cv2_to_imgmsg(right, encoding='bgr8')
        img_r.header = header_r
        self._pub_right_img.publish(img_r)

        self._info_right.header = header_r
        self._pub_right_info.publish(self._info_right)

    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        self.get_logger().info('Stopping grabber...')
        self._grabber.stop()
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
