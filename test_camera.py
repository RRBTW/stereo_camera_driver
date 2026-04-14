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

  # Принудительно YUYV вместо MJPG:
  python3 test_camera.py --device 2 --width 1280 --height 480 --split --no-mjpg

Выход: q или Ctrl-C
"""

import argparse
import glob
import os
import subprocess
import sys
import time

import cv2
import numpy as np


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


def annotate(frame: np.ndarray, label: str, fps: float, n: int) -> np.ndarray:
    out = frame.copy()
    cv2.putText(out, f'{label}  FPS:{fps:.1f}  f:{n}',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return out


def separator(h: int) -> np.ndarray:
    return np.full((h, 4, 3), 64, dtype=np.uint8)


def print_stats(t_start: float, n: int, d: int = 0) -> tuple[float, float]:
    el = time.monotonic() - t_start
    fps = n / el if el > 0 else 0.0
    print(f'\r  Кадров:{n:6d}  FPS:{fps:5.1f}  Пропусков:{d:3d}  Время:{el:6.1f}с  ',
          end='', flush=True)
    return fps, el


# ---------------------------------------------------------------------------
# Режим одной камеры
# ---------------------------------------------------------------------------

def run_single(cap: cv2.VideoCapture, label: str, no_display: bool) -> None:
    win = f'Camera {label}  [q - выход]'
    if not no_display:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    n, fps, t = 0, 0.0, time.monotonic()
    print(f'\nЗахват [{label}]. q / Ctrl-C для выхода.\n')
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue
            n += 1
            fps, _ = print_stats(t, n)
            if not no_display:
                cv2.imshow(win, annotate(frame, label, fps, n))
                if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        _, el = print_stats(t, n)
        print(f'\n\nИтого: {n} кадров за {el:.1f}с')
        if not no_display:
            cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Режим side-by-side (одно USB-устройство, двойной кадр)
# ---------------------------------------------------------------------------

def run_split(cap: cv2.VideoCapture, device: str, no_display: bool) -> None:
    """
    Захватывает широкий кадр (напр. 1280x480) и разрезает пополам:
      левая половина  → LEFT  камера
      правая половина → RIGHT камера
    """
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    mid = W // 2
    print(f'  Разрезаем: LEFT=[0:{mid}]  RIGHT=[{mid}:{W}]  каждый глаз: {mid}x{H}')
    win = f'Stereo split  {device}  [q - выход]'
    if not no_display:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    n, d, fps, t = 0, 0, 0.0, time.monotonic()
    print('\nЗахват (split). q / Ctrl-C для выхода.\n')
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                d += 1
                time.sleep(0.02)
                continue
            n += 1
            fps, _ = print_stats(t, n, d)
            if not no_display:
                L = annotate(frame[:, :mid],  'LEFT',  fps, n)
                R = annotate(frame[:, mid:], 'RIGHT', fps, n)
                cv2.imshow(win, np.hstack([L, separator(H), R]))
                if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        _, el = print_stats(t, n, d)
        print(f'\n\nИтого: {n} кадров, {d} пропусков, {el:.1f}с')
        if not no_display:
            cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Режим двух отдельных USB-устройств
# ---------------------------------------------------------------------------

def run_stereo(cap_l: cv2.VideoCapture, cap_r: cv2.VideoCapture,
               dev_l: str, dev_r: str, no_display: bool) -> None:
    win = f'Stereo pair  LEFT={dev_l}  RIGHT={dev_r}  [q - выход]'
    if not no_display:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    n, d, fps, t = 0, 0, 0.0, time.monotonic()
    print(f'\nСтереозахват LEFT={dev_l}  RIGHT={dev_r}. q / Ctrl-C.\n')
    try:
        while True:
            rl, fl = cap_l.read()
            rr, fr = cap_r.read()
            if not rl or not rr:
                d += 1
                time.sleep(0.02)
                continue
            n += 1
            fps, _ = print_stats(t, n, d)
            if not no_display:
                h = min(fl.shape[0], fr.shape[0])
                L = annotate(fl[:h], f'LEFT  {dev_l}', fps, n)
                R = annotate(fr[:h], f'RIGHT {dev_r}', fps, n)
                cv2.imshow(win, np.hstack([L, separator(h), R]))
                if cv2.waitKey(1) & 0xFF in (ord('q'), ord('Q'), 27):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        _, el = print_stats(t, n, d)
        print(f'\n\nИтого: {n} кадров, {d} пропусков, {el:.1f}с')
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
    p.add_argument('--no-mjpg',    action='store_true',
                   help='Не форсировать MJPG (использовать YUYV)')
    p.add_argument('--no-display', action='store_true',
                   help='Не открывать окно (headless / SSH)')
    a = p.parse_args()

    if a.list:
        list_cameras()
        return

    mjpg = not a.no_mjpg

    if a.split:
        print(f'\n=== Side-by-side: {a.device}  {a.width}x{a.height} @ {a.fps} fps'
              f'  MJPG={mjpg} ===')
        cap = open_cap(a.device, a.width, a.height, a.fps, mjpg)
        print_info(cap, a.device, 'SBS')
        try:
            run_split(cap, a.device, a.no_display)
        finally:
            cap.release()

    elif a.device2 is not None:
        print(f'\n=== Стереопара: LEFT={a.device}  RIGHT={a.device2} ===')
        cl = open_cap(a.device,  a.width, a.height, a.fps, mjpg)
        cr = open_cap(a.device2, a.width, a.height, a.fps, mjpg)
        print('LEFT:');  print_info(cl, a.device,  'LEFT')
        print('RIGHT:'); print_info(cr, a.device2, 'RIGHT')
        try:
            run_stereo(cl, cr, a.device, a.device2, a.no_display)
        finally:
            cl.release()
            cr.release()

    else:
        print(f'\n=== Камера: {a.device} ===')
        cap = open_cap(a.device, a.width, a.height, a.fps, mjpg)
        print_info(cap, a.device)
        try:
            run_single(cap, a.device, a.no_display)
        finally:
            cap.release()


if __name__ == '__main__':
    main()
