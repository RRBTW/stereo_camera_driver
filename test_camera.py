#!/usr/bin/env python3
"""
Автономный тест камеры — без ROS, только OpenCV.

Использование:
  # Список устройств:
  python3 test_camera.py --list

  # Одна обычная камера:
  python3 test_camera.py --device 2

  # 1 стереокамера (side-by-side, один USB):
  python3 test_camera.py --device /dev/stereo_cam0 --width 1280 --height 480 --split

  # 2 стереокамеры одновременно:
  python3 test_camera.py --device /dev/stereo_cam0 --device2 /dev/stereo_cam1 --width 1280 --height 480 --split

  # 3 стереокамеры одновременно:
  python3 test_camera.py --device /dev/stereo_cam0 --device2 /dev/stereo_cam1 --device3 /dev/stereo_cam2 --width 1280 --height 480 --split

  # Без окна (headless / SSH):
  python3 test_camera.py --device /dev/stereo_cam0 --split --no-display

Выход: q или Ctrl-C
"""

import argparse
import fcntl
import glob
import mmap
import os
import select
import struct
import subprocess
import sys
import threading
import time

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Прямой V4L2 MJPG захват (обходит OpenCV, который сбрасывает формат)
# ---------------------------------------------------------------------------

class MJPGCapture:
    """
    Захватывает кадры через прямые V4L2 ioctl + mmap.
    Нужен для стереокамер, у которых OpenCV при открытии сбрасывает
    MJPG 1280x480 обратно в YUYV 640x480.
    """
    _BUF_TYPE = 1           # V4L2_BUF_TYPE_VIDEO_CAPTURE
    _MEMORY   = 1           # V4L2_MEMORY_MMAP
    _MJPG     = 0x47504A4D  # V4L2_PIX_FMT_MJPEG

    # ioctl-коды (получены через sizeof на этой платформе)
    _S_FMT    = 0xC0D05605
    _S_PARM   = 0xC0CC5616
    _REQBUFS  = 0xC0145608
    _QUERYBUF = 0xC0585609
    _QBUF     = 0xC058560F
    _DQBUF    = 0xC0585611
    _STREAMON = 0x40045612

    # Смещения в v4l2_format: union fmt начинается с offset 8 (4 байта padding после type)
    _OFF_WIDTH  = 8
    _OFF_HEIGHT = 12
    _OFF_PIXFMT = 16
    _OFF_FIELD  = 20

    # Смещения в v4l2_buffer (88 байт)
    _OFF_IDX      = 0
    _OFF_TYPE_BUF = 4
    _OFF_BYTESUSED= 8
    _OFF_MEMORY   = 60
    _OFF_M_OFFSET = 64
    _OFF_LENGTH   = 72

    def __init__(self, device: str, width: int, height: int,
                 fps: float = 30.0, n_bufs: int = 2):
        dev_path = (f'/dev/video{device}'
                    if str(device).lstrip('-').isdigit() else device)
        self.width  = width
        self.height = height
        self._mmaps: list[tuple[mmap.mmap, int]] = []
        self.fd = os.open(dev_path, os.O_RDWR | os.O_NONBLOCK)
        self._set_fmt(width, height)
        self._set_fps(fps)
        self._init_buffers(n_bufs)
        self._stream_on()

    # ------------------------------------------------------------------
    def _ioctl(self, req: int, buf: bytearray) -> None:
        fcntl.ioctl(self.fd, req, buf)

    def _set_fmt(self, w: int, h: int) -> None:
        buf = bytearray(208)
        struct.pack_into('=I', buf, 0, self._BUF_TYPE)
        struct.pack_into('=IIII', buf, self._OFF_WIDTH, w, h, self._MJPG, 0)
        self._ioctl(self._S_FMT, buf)
        aw = struct.unpack_from('I', buf, self._OFF_WIDTH)[0]
        ah = struct.unpack_from('I', buf, self._OFF_HEIGHT)[0]
        pf = struct.unpack_from('I', buf, self._OFF_PIXFMT)[0]
        ok = pf == self._MJPG and aw == w and ah == h
        print(f'  MJPGCapture: {aw}x{ah}  fmt={"MJPG" if pf==self._MJPG else hex(pf)}'
              f'  {"OK" if ok else "WARN: не то разрешение/формат"}')

    def _set_fps(self, fps: float) -> None:
        # VIDIOC_S_PARM: struct v4l2_streamparm (204 байта)
        # type@0, timeperframe.numerator@12, timeperframe.denominator@16
        buf = bytearray(204)
        struct.pack_into('=I',  buf, 0,  self._BUF_TYPE)
        struct.pack_into('=II', buf, 12, 1, max(1, round(fps)))
        try:
            self._ioctl(self._S_PARM, buf)
            actual = struct.unpack_from('I', buf, 16)[0]
            print(f'  FPS: {actual}')
        except OSError:
            pass  # некоторые драйверы не поддерживают S_PARM

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

    # ------------------------------------------------------------------
    def read(self) -> tuple[bool, np.ndarray | None]:
        if not select.select([self.fd], [], [], 2.0)[0]:
            return False, None
        db = bytearray(88)
        struct.pack_into('=II', db, self._OFF_IDX, 0, self._BUF_TYPE)
        struct.pack_into('I',   db, self._OFF_MEMORY, self._MEMORY)
        try:
            self._ioctl(self._DQBUF, db)
        except OSError:
            return False, None
        idx       = struct.unpack_from('I', db, self._OFF_IDX)[0]
        bytesused = struct.unpack_from('I', db, self._OFF_BYTESUSED)[0]
        mm, _     = self._mmaps[idx]
        mm.seek(0)
        frame = cv2.imdecode(
            np.frombuffer(mm.read(bytesused), np.uint8), cv2.IMREAD_COLOR)
        self._qbuf(idx)
        return (True, frame) if frame is not None else (False, None)

    def get(self, prop: int) -> float:
        if prop == cv2.CAP_PROP_FRAME_WIDTH:  return float(self.width)
        if prop == cv2.CAP_PROP_FRAME_HEIGHT: return float(self.height)
        return 0.0

    def isOpened(self) -> bool:
        return self.fd >= 0

    def release(self) -> None:
        for mm, _ in self._mmaps:
            mm.close()
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1
        self._mmaps.clear()


# ---------------------------------------------------------------------------
# Вспомогательные функции
# ---------------------------------------------------------------------------

def list_cameras() -> None:
    """Выводит все /dev/video* и соответствующие by-path симлинки."""
    videos = sorted(glob.glob('/dev/video*'))
    if not videos:
        print('Устройства /dev/video* не найдены.')
        return

    by_path_map: dict[str, list[str]] = {}
    for link in sorted(glob.glob('/dev/v4l/by-path/*')):
        real = os.path.realpath(link)
        by_path_map.setdefault(real, []).append(link)

    print(f'{"УСТРОЙСТВО":<20}  {"DRIVER/NAME":<30}  BY-PATH')
    print('-' * 90)
    for dev in videos:
        name = ''
        try:
            out = subprocess.check_output(
                ['v4l2-ctl', '--device', dev, '--info'],
                stderr=subprocess.DEVNULL,
                timeout=2,
            ).decode()
            for line in out.splitlines():
                if 'Card type' in line:
                    name = line.split(':', 1)[-1].strip()
                    break
        except Exception:
            pass

        real = os.path.realpath(dev)
        paths = by_path_map.get(real, [])
        first = paths[0] if paths else '—'
        extra = ('\n' + ' ' * 52).join(paths[1:])
        extra_str = ('\n' + ' ' * 52 + extra) if extra else ''
        print(f'{dev:<20}  {name:<30}  {first}{extra_str}')


def open_camera(device: str, width: int, height: int, fps: float) -> cv2.VideoCapture:
    """Открывает камеру через V4L2 и задаёт параметры."""
    src = int(device) if device.lstrip('-').isdigit() else device
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'[ERROR] Не удалось открыть камеру: {device}', file=sys.stderr)
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS,          fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    return cap


def open_mjpg(device: str, width: int, height: int, fps: float = 30.0) -> MJPGCapture:
    """Открывает стереокамеру в MJPG через прямые V4L2 ioctl."""
    try:
        cap = MJPGCapture(device, width, height, fps=fps)
    except OSError as e:
        print(f'[ERROR] MJPGCapture: {e}', file=sys.stderr)
        sys.exit(1)
    return cap


def print_camera_info(cap: cv2.VideoCapture, device: str, label: str = '') -> None:
    """Выводит реальные параметры захвата после открытия."""
    actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    fmt_raw    = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc     = ''.join(chr((fmt_raw >> i) & 0xFF) for i in (0, 8, 16, 24))
    tag = f' [{label}]' if label else ''
    print(f'  Устройство{tag}: {device}')
    print(f'  Разрешение : {actual_w} x {actual_h}')
    print(f'  FPS        : {actual_fps}')
    print(f'  Формат     : {fourcc}')


def _annotate(frame: np.ndarray, label: str, fps: float, count: int) -> np.ndarray:
    """Рисует метку и FPS поверх кадра."""
    out = frame.copy()
    cv2.putText(out, f'{label}  FPS:{fps:.1f}  f:{count}',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return out


def split_stereo(frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Разрезает широкий side-by-side кадр стереокамеры пополам.

    Стереокамера выдаёт один кадр NxW (напр. 480x1280), где
    левая камера — левая половина, правая — правая половина.

    Возвращает (left, right) — два отдельных изображения Nx(W/2).
    """
    mid = frame.shape[1] // 2
    return frame[:, :mid], frame[:, mid:]


# ---------------------------------------------------------------------------
# Режим одной камеры
# ---------------------------------------------------------------------------

def run_single(cap: cv2.VideoCapture, label: str, no_display: bool) -> None:
    window = f'Camera {label}  [q - выход]'
    if not no_display:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    frame_count  = 0
    fps_measured = 0.0
    t_start      = time.monotonic()

    print(f'\nЗахват [{label}] запущен. q или Ctrl-C для выхода.\n')
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print('[WARN] Кадр не получен')
                time.sleep(0.05)
                continue

            frame_count += 1
            elapsed = time.monotonic() - t_start
            fps_measured = frame_count / elapsed

            print(f'\r  Кадров: {frame_count:6d}  FPS: {fps_measured:5.1f}  '
                  f'Время: {elapsed:6.1f} с  ', end='', flush=True)

            if not no_display:
                cv2.imshow(window, _annotate(frame, label, fps_measured, frame_count))
                if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - t_start
        print(f'\n\nИтого: {frame_count} кадров за {elapsed:.1f} с  '
              f'(средний FPS: {frame_count / elapsed:.1f})')
        if not no_display:
            cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Фоновый захват кадров (один поток на камеру)
# ---------------------------------------------------------------------------

class _FrameGrabber:
    """Захватывает кадры из MJPGCapture в фоновом потоке."""

    def __init__(self, cap: MJPGCapture) -> None:
        self._cap     = cap
        self._frame: np.ndarray | None = None
        self._lock    = threading.Lock()
        self._stop    = False
        self._grabbed = 0
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def _loop(self) -> None:
        while not self._stop:
            ret, frame = self._cap.read()
            if ret and frame is not None:
                with self._lock:
                    self._frame = frame
                    self._grabbed += 1

    def read(self) -> tuple[bool, np.ndarray | None]:
        with self._lock:
            return (self._frame is not None), self._frame

    @property
    def grabbed(self) -> int:
        with self._lock:
            return self._grabbed

    def stop(self) -> None:
        self._stop = True


# ---------------------------------------------------------------------------
# Режим side-by-side: 1-3 стереокамеры одновременно
# ---------------------------------------------------------------------------

def run_multi_split(
    caps: list[MJPGCapture],
    devices: list[str],
    no_display: bool,
) -> None:
    """
    Запускает 1-3 стереокамеры (каждая — один USB, wide-кадр).
    Каждая камера захватывается в своём потоке.
    Кадр разрезается на LEFT/RIGHT через split_stereo().
    Все камеры выводятся вертикально в одном окне:

        ┌──────────────┬──────────────┐
        │  CAM0  LEFT  │  CAM0  RIGHT │
        ├──────────────┼──────────────┤
        │  CAM1  LEFT  │  CAM1  RIGHT │
        ├──────────────┼──────────────┤
        │  CAM2  LEFT  │  CAM2  RIGHT │
        └──────────────┴──────────────┘
    """
    n   = len(caps)
    W   = caps[0].width
    H   = caps[0].height
    half = W // 2   # ширина одной половины (LEFT или RIGHT)
    SEP  = 4        # ширина вертикального разделителя между LEFT и RIGHT

    print(f'\n  Камер: {n}  кадр {W}x{H}  left/right по {half}px')

    # Один буфер для всего окна (n строк, каждая half+SEP+half пикселей)
    canvas_w = half * 2 + SEP
    canvas = np.zeros((H * n, canvas_w, 3), dtype=np.uint8)
    canvas[:, half:half + SEP] = 64   # вертикальный разделитель LEFT|RIGHT

    grabbers = [_FrameGrabber(cap) for cap in caps]

    window = f'Stereo x{n}  [q - выход]'
    if not no_display:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    frame_count  = 0
    fps_measured = 0.0
    t_start      = time.monotonic()

    print('Захват запущен. q или Ctrl-C для выхода.\n')
    try:
        while True:
            # Читаем последний кадр от каждой камеры
            frames: list[np.ndarray | None] = []
            all_ready = True
            for g in grabbers:
                ok, fr = g.read()
                if not ok:
                    all_ready = False
                    break
                frames.append(fr)

            if not all_ready:
                time.sleep(0.005)
                continue

            frame_count += 1
            elapsed      = time.monotonic() - t_start
            fps_measured = frame_count / elapsed

            print(f'\r  Кадров: {frame_count:6d}  FPS: {fps_measured:5.1f}  '
                  f'Время: {elapsed:6.1f} с  ', end='', flush=True)

            if not no_display:
                for i, frame in enumerate(frames):
                    left, right = split_stereo(frame)
                    row = canvas[i * H:(i + 1) * H]

                    rh = min(frame.shape[0], H)
                    row[:rh, :half]              = left[:rh]
                    row[:rh, half + SEP:]        = right[:rh]

                    # аннотации
                    cv2.putText(row,
                                f'CAM{i} LEFT  {devices[i]}  FPS:{fps_measured:.1f}  f:{frame_count}',
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(row,
                                f'CAM{i} RIGHT',
                                (half + SEP + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                # горизонтальные разделители между камерами
                for i in range(1, n):
                    canvas[i * H - 1:i * H + 1] = 128

                cv2.imshow(window, canvas)
                if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        for g in grabbers:
            g.stop()
        elapsed = time.monotonic() - t_start
        print(f'\n\nИтого: {frame_count} кадров за {elapsed:.1f} с  '
              f'(средний FPS: {frame_count / elapsed:.1f})')
        if not no_display:
            cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Режим стереопары
# ---------------------------------------------------------------------------

def run_stereo(
    cap_l: cv2.VideoCapture,
    cap_r: cv2.VideoCapture,
    dev_l: str,
    dev_r: str,
    no_display: bool,
) -> None:
    """
    Захватывает кадры с обеих камер синхронно (как в stereo_camera_node).
    Показывает их рядом в одном окне.
    Печатает в терминал: FPS пары и счётчик пропущенных кадров.
    """
    window = 'Stereo pair  [q - выход]'
    if not no_display:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    frame_count  = 0
    drop_count   = 0
    fps_measured = 0.0
    t_start      = time.monotonic()

    print(f'\nСтереозахват  LEFT={dev_l}  RIGHT={dev_r}')
    print('q или Ctrl-C для выхода.\n')

    try:
        while True:
            ret_l, frame_l = cap_l.read()
            ret_r, frame_r = cap_r.read()

            if not ret_l or not ret_r:
                drop_count += 1
                print(f'\r[WARN] пропуск #{drop_count}  '
                      f'(left={ret_l}, right={ret_r})          ', flush=True)
                time.sleep(0.02)
                continue

            frame_count += 1
            elapsed      = time.monotonic() - t_start
            fps_measured = frame_count / elapsed

            print(f'\r  Кадров: {frame_count:6d}  FPS: {fps_measured:5.1f}  '
                  f'Пропусков: {drop_count:3d}  Время: {elapsed:6.1f} с  ',
                  end='', flush=True)

            if not no_display:
                ann_l = _annotate(frame_l, f'LEFT  {dev_l}',  fps_measured, frame_count)
                ann_r = _annotate(frame_r, f'RIGHT {dev_r}', fps_measured, frame_count)

                # Привести к одинаковой высоте на случай разных разрешений
                h = min(ann_l.shape[0], ann_r.shape[0])
                combined = np.hstack([
                    ann_l[:h],
                    np.full((h, 4, 3), 64, dtype=np.uint8),  # разделитель
                    ann_r[:h],
                ])
                cv2.imshow(window, combined)
                if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.monotonic() - t_start
        print(f'\n\nИтого: {frame_count} кадров, {drop_count} пропусков, '
              f'{elapsed:.1f} с  (средний FPS: {frame_count / elapsed:.1f})')
        if not no_display:
            cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Тест захвата камеры/стереопары (без ROS)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument('--list',       action='store_true',
                        help='Показать все V4L2-устройства и выйти')
    parser.add_argument('--device',     default='0',
                        help='Камера 0 (путь или номер, default: 0)')
    parser.add_argument('--device2',    default=None,
                        help='Камера 1 (для --split: вторая стереокамера)')
    parser.add_argument('--device3',    default=None,
                        help='Камера 2 (для --split: третья стереокамера)')
    parser.add_argument('--split',      action='store_true',
                        help='Side-by-side: wide кадр разрезается пополам (до 3 камер)')
    parser.add_argument('--width',      type=int,   default=640,
                        help='Ширина захвата (для --split: 1280)')
    parser.add_argument('--height',     type=int,   default=480)
    parser.add_argument('--fps',        type=float, default=30.0)
    parser.add_argument('--no-display', action='store_true',
                        help='Не открывать окно (headless / SSH)')

    args = parser.parse_args()

    if args.list:
        list_cameras()
        return

    w, h, fps = args.width, args.height, args.fps

    if args.split:
        # --- режим side-by-side: 1-3 стереокамеры ---
        devices = [args.device]
        if args.device2:
            devices.append(args.device2)
        if args.device3:
            devices.append(args.device3)
        print(f'\n=== Stereo split x{len(devices)}: {", ".join(devices)}  {w}x{h} ===')
        caps = []
        try:
            for dev in devices:
                caps.append(open_mjpg(dev, w, h, fps))
            run_multi_split(caps, devices, args.no_display)
        finally:
            for cap in caps:
                cap.release()

    elif args.device2 is not None:
        # --- режим двух отдельных камер ---
        print(f'\n=== Стереопара: LEFT={args.device}  RIGHT={args.device2} ===')
        cap_l = open_camera(args.device,  w, h, fps)
        cap_r = open_camera(args.device2, w, h, fps)
        print('LEFT:')
        print_camera_info(cap_l, args.device,  'LEFT')
        print('RIGHT:')
        print_camera_info(cap_r, args.device2, 'RIGHT')
        try:
            run_stereo(cap_l, cap_r, args.device, args.device2, args.no_display)
        finally:
            cap_l.release()
            cap_r.release()

    else:
        # --- режим одной камеры ---
        print(f'\n=== Открываем камеру: {args.device} ===')
        cap = open_camera(args.device, w, h, fps)
        print_camera_info(cap, args.device)
        try:
            run_single(cap, args.device, args.no_display)
        finally:
            cap.release()


if __name__ == '__main__':
    main()
