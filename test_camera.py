#!/usr/bin/env python3
"""
Автономный тест камеры — без ROS, только OpenCV.

Использование:
  # 1. Показать все доступные V4L2-устройства и по-path имена:
  python3 test_camera.py --list

  # 2. Тест одной камеры (по номеру или по пути):
  python3 test_camera.py --device 2
  python3 test_camera.py --device /dev/v4l/by-path/pci-...-video-index0

  # 3. Тест стереопары — две камеры одновременно:
  python3 test_camera.py --device 0 --device2 2

  # 4. Без окна (headless / SSH без -X):
  python3 test_camera.py --device 2 --no-display

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
                        help='Левая/единственная камера (путь или номер, default: 0)')
    parser.add_argument('--device2',    default=None,
                        help='Правая камера — включает режим стереопары')
    parser.add_argument('--width',      type=int,   default=640)
    parser.add_argument('--height',     type=int,   default=480)
    parser.add_argument('--fps',        type=float, default=30.0)
    parser.add_argument('--no-display', action='store_true',
                        help='Не открывать окно (headless / SSH)')

    args = parser.parse_args()

    if args.list:
        list_cameras()
        return

    w, h, fps = args.width, args.height, args.fps

    if args.device2 is not None:
        # --- режим стереопары ---
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
