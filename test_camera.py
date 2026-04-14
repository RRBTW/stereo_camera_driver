#!/usr/bin/env python3
"""
Автономный тест камеры — без ROS, только OpenCV.

Использование:
  # Список устройств:
  python3 test_camera.py --list

  # Одна камера:
  python3 test_camera.py --device 2

  # Одно USB-устройство с двойным кадром side-by-side (1280x480 → два 640x480):
  python3 test_camera.py --device 2 --width 1280 --height 480 --split

  # Два отдельных USB-устройства:
  python3 test_camera.py --device 0 --device2 2

  # Без окна (headless / SSH):
  python3 test_camera.py --device 2 --no-display

  # SBS 30fps (MJPG 2560x720 = два глаза 1280x720):
  python3 test_camera.py --device 2 --width 2560 --height 720 --split --mjpg

Выход: q или Ctrl-C
"""

import argparse
import glob
import os
import subprocess
import sys
import threading
import time

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Захват в отдельном потоке
# ---------------------------------------------------------------------------

class FrameGrabber:
    """
    Читает кадры из cap.read() в фоновом потоке непрерывно.
    Главный поток забирает последний готовый кадр без ожидания камеры.

    Это разделяет скорость захвата и скорость отображения:
    камера работает на своём максимальном FPS независимо от дисплея.
    """

    def __init__(self, cap: cv2.VideoCapture):
        self._cap      = cap
        self._frame    = None
        self._ret      = False
        self._lock     = threading.Lock()
        self._stopped  = False
        self._grabbed  = 0   # счётчик захваченных кадров
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def _loop(self) -> None:
        while not self._stopped:
            ret, frame = self._cap.read()
            with self._lock:
                self._ret   = ret
                self._frame = frame
                if ret:
                    self._grabbed += 1

    def read(self) -> tuple[bool, np.ndarray | None]:
        """Возвращает последний захваченный кадр (без ожидания)."""
        with self._lock:
            return self._ret, self._frame

    @property
    def grabbed(self) -> int:
        with self._lock:
            return self._grabbed

    def stop(self) -> None:
        self._stopped = True


# ---------------------------------------------------------------------------
# Вспомогательные функции
# ---------------------------------------------------------------------------

def list_cameras() -> None:
    videos = sorted(glob.glob('/dev/video*'))
    if not videos:
        print('Устройства /dev/video* не найдены.')
        return
    by_path_map: dict[str, list[str]] = {}
    for link in sorted(glob.glob('/dev/v4l/by-path/*')):
        real = os.path.realpath(link)
        by_path_map.setdefault(real, []).append(link)
    print(f'{"УСТРОЙСТВО":<20}  {"NAME":<30}  BY-PATH')
    print('-' * 90)
    for dev in videos:
        name = ''
        try:
            out = subprocess.check_output(
                ['v4l2-ctl', '--device', dev, '--info'],
                stderr=subprocess.DEVNULL, timeout=2).decode()
            for line in out.splitlines():
                if 'Card type' in line:
                    name = line.split(':', 1)[-1].strip()
                    break
        except Exception:
            pass
        real = os.path.realpath(dev)
        paths = by_path_map.get(real, [])
        first = paths[0] if paths else '—'
        rest = ('\n' + ' ' * 52).join(paths[1:])
        extra = ('\n' + ' ' * 52 + rest) if rest else ''
        print(f'{dev:<20}  {name:<30}  {first}{extra}')


def open_cap(device: str, width: int, height: int, fps: float,
             mjpg: bool = True) -> cv2.VideoCapture:
    src = int(device) if device.lstrip('-').isdigit() else device
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'[ERROR] Не удалось открыть: {device}', file=sys.stderr)
        sys.exit(1)
    if mjpg:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS,          fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    return cap


def print_info(cap: cv2.VideoCapture, device: str, label: str = '') -> None:
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    f = cap.get(cv2.CAP_PROP_FPS)
    r = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = ''.join(chr((r >> i) & 0xFF) for i in (0, 8, 16, 24))
    tag = f' [{label}]' if label else ''
    print(f'  Устройство{tag}: {device}')
    print(f'  Захват     : {w}x{h}  {fourcc}  {f} fps')


def annotate(frame: np.ndarray, label: str, fps: float, n: int) -> None:
    """Рисует текст прямо в кадр (без копии)."""
    cv2.putText(frame, f'{label}  FPS:{fps:.1f}  f:{n}',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)




# ---------------------------------------------------------------------------
# Режим одной камеры
# ---------------------------------------------------------------------------

def run_single(cap: cv2.VideoCapture, label: str, no_display: bool,
               show_every: int = 1) -> None:
    grabber = FrameGrabber(cap)
    win = f'Camera {label}  [q - выход]'
    if not no_display:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    disp_n, fps, t = 0, 0.0, time.monotonic()
    print(f'\nЗахват [{label}]. q / Ctrl-C для выхода.\n')
    try:
        while True:
            ret, frame = grabber.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue
            cap_n = grabber.grabbed
            el    = time.monotonic() - t
            fps   = cap_n / el if el > 0 else 0.0
            print(f'\r  Захват:{cap_n:6d}  FPS:{fps:5.1f}  Время:{el:6.1f}с  ',
                  end='', flush=True)
            disp_n += 1
            if not no_display and disp_n % show_every == 0:
                f = frame.copy()
                annotate(f, label, fps, cap_n)
                cv2.imshow(win, f)
                if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                    break
            else:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        grabber.stop()
        el = time.monotonic() - t
        print(f'\n\nИтого: {grabber.grabbed} кадров за {el:.1f}с  '
              f'(FPS: {grabber.grabbed / el:.1f})')
        if not no_display:
            cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Режим side-by-side (одно USB-устройство, двойной кадр)
# ---------------------------------------------------------------------------

def run_split(cap: cv2.VideoCapture, device: str, no_display: bool,
              show_every: int = 1) -> None:
    """
    Захватывает широкий кадр (напр. 1280x480) в фоновом потоке.
    W/H берутся из первого реального кадра — не из cap.get() —
    чтобы корректно работать при любом фактическом разрешении.
    """
    grabber = FrameGrabber(cap)
    win = f'Stereo split  {device}  [q - выход]'
    if not no_display:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    display_buf: np.ndarray | None = None
    mid = 0
    disp_n, fps, t = 0, 0.0, time.monotonic()
    print('\nОжидаем первый кадр...\n')
    try:
        while True:
            ret, frame = grabber.read()
            if not ret or frame is None:
                time.sleep(0.005)
                continue

            # Инициализируем размеры по первому реальному кадру
            if display_buf is None:
                H, W = frame.shape[:2]
                mid = W // 2
                display_buf = np.empty((H, W + 4, 3), dtype=np.uint8)
                display_buf[:, mid:mid + 4] = 64
                print(f'  Реальный кадр: {W}x{H}  '
                      f'→ LEFT=[0:{mid}]  RIGHT=[{mid}:{W}]  '
                      f'каждый глаз: {mid}x{H}')
                if W <= H * 2:  # подозрительно: не похоже на side-by-side
                    print(f'  [WARN] Ширина {W} мала для side-by-side — '
                          f'возможно камера отдаёт одиночный кадр. '
                          f'Попробуйте --no-mjpg или --width 2560 --height 720')
                print()

            cap_n = grabber.grabbed
            el    = time.monotonic() - t
            fps   = cap_n / el if el > 0 else 0.0
            print(f'\r  Захват:{cap_n:6d}  FPS:{fps:5.1f}  Время:{el:6.1f}с  ',
                  end='', flush=True)

            disp_n += 1
            if not no_display and disp_n % show_every == 0:
                H, W = frame.shape[:2]
                display_buf[:H, :mid]      = frame[:H, :mid]
                display_buf[:H, mid + 4:]  = frame[:H, mid:]
                annotate(display_buf[:H, :mid],     'LEFT',  fps, cap_n)
                annotate(display_buf[:H, mid + 4:], 'RIGHT', fps, cap_n)
                cv2.imshow(win, display_buf[:H])
                if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                    break
            else:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        grabber.stop()
        el = time.monotonic() - t
        print(f'\n\nИтого: {grabber.grabbed} кадров за {el:.1f}с  '
              f'(FPS: {grabber.grabbed / el:.1f})')
        if not no_display:
            cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Режим двух отдельных USB-устройств
# ---------------------------------------------------------------------------

def run_stereo(cap_l: cv2.VideoCapture, cap_r: cv2.VideoCapture,
               dev_l: str, dev_r: str, no_display: bool,
               show_every: int = 1) -> None:
    grabber_l = FrameGrabber(cap_l)
    grabber_r = FrameGrabber(cap_r)
    win = f'Stereo pair  LEFT={dev_l}  RIGHT={dev_r}  [q - выход]'
    if not no_display:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    disp_n, fps, t = 0, 0.0, time.monotonic()
    display_buf: np.ndarray | None = None
    print(f'\nСтереозахват LEFT={dev_l}  RIGHT={dev_r}. q / Ctrl-C.\n')
    try:
        while True:
            rl, fl = grabber_l.read()
            rr, fr = grabber_r.read()
            if not rl or fl is None or not rr or fr is None:
                time.sleep(0.005)
                continue

            cap_n = min(grabber_l.grabbed, grabber_r.grabbed)
            el    = time.monotonic() - t
            fps   = cap_n / el if el > 0 else 0.0
            print(f'\r  Захват:{cap_n:6d}  FPS:{fps:5.1f}  Время:{el:6.1f}с  ',
                  end='', flush=True)

            disp_n += 1
            if not no_display and disp_n % show_every == 0:
                h = min(fl.shape[0], fr.shape[0])
                w = fl.shape[1]
                if display_buf is None or display_buf.shape != (h, w * 2 + 4, 3):
                    display_buf = np.empty((h, w * 2 + 4, 3), dtype=np.uint8)
                    display_buf[:, w:w + 4] = 64
                display_buf[:h, :w]       = fl[:h]
                display_buf[:h, w + 4:]   = fr[:h]
                annotate(display_buf[:h, :w],     f'LEFT  {dev_l}', fps, cap_n)
                annotate(display_buf[:h, w + 4:], f'RIGHT {dev_r}', fps, cap_n)
                cv2.imshow(win, display_buf)
                if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                    break
            else:
                time.sleep(0.005)
    except KeyboardInterrupt:
        pass
    finally:
        grabber_l.stop()
        grabber_r.stop()
        el = time.monotonic() - t
        n = min(grabber_l.grabbed, grabber_r.grabbed)
        print(f'\n\nИтого: {n} кадров за {el:.1f}с  (FPS: {n / el:.1f})')
        if not no_display:
            cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> None:
    p = argparse.ArgumentParser(
        description='Тест камеры/стереопары (без ROS)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument('--list',       action='store_true',
                   help='Показать все V4L2-устройства и выйти')
    p.add_argument('--device',     default='0',
                   help='Камера (путь или номер, default: 0)')
    p.add_argument('--device2',    default=None,
                   help='Вторая камера — режим двух отдельных USB-устройств')
    p.add_argument('--split',      action='store_true',
                   help='Side-by-side: разрезать широкий кадр пополам')
    p.add_argument('--width',      type=int,   default=640,
                   help='Ширина захвата (для --split: полная, напр. 1280)')
    p.add_argument('--height',     type=int,   default=480)
    p.add_argument('--fps',        type=float, default=30.0)
    p.add_argument('--mjpg',        action='store_true',
                   help='Форсировать MJPG. По умолчанию YUYV.\n'
                        'Для SBS: YUYV 1280x480 = 15fps | MJPG 2560x720 = 30fps')
    p.add_argument('--no-display',  action='store_true',
                   help='Не открывать окно (headless / SSH)')
    p.add_argument('--show-every',  type=int, default=1,
                   help='Отображать каждый N-й кадр (захват идёт на полном FPS)')
    a = p.parse_args()

    if a.list:
        list_cameras()
        return

    mjpg = a.mjpg
    show_every = max(1, a.show_every)

    if a.split:
        print(f'\n=== Side-by-side: {a.device}  {a.width}x{a.height} @ {a.fps} fps'
              f'  MJPG={mjpg}  show_every={show_every} ===')
        cap = open_cap(a.device, a.width, a.height, a.fps, mjpg)
        print_info(cap, a.device, 'SBS')
        try:
            run_split(cap, a.device, a.no_display, show_every)
        finally:
            cap.release()

    elif a.device2 is not None:
        print(f'\n=== Стереопара: LEFT={a.device}  RIGHT={a.device2} ===')
        cl = open_cap(a.device,  a.width, a.height, a.fps, mjpg)
        cr = open_cap(a.device2, a.width, a.height, a.fps, mjpg)
        print('LEFT:');  print_info(cl, a.device,  'LEFT')
        print('RIGHT:'); print_info(cr, a.device2, 'RIGHT')
        try:
            run_stereo(cl, cr, a.device, a.device2, a.no_display, show_every)
        finally:
            cl.release()
            cr.release()

    else:
        print(f'\n=== Камера: {a.device} ===')
        cap = open_cap(a.device, a.width, a.height, a.fps, mjpg)
        print_info(cap, a.device)
        try:
            run_single(cap, a.device, a.no_display, show_every)
        finally:
            cap.release()


if __name__ == '__main__':
    main()
