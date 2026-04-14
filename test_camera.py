#!/usr/bin/env python3
"""
Автономный тест камеры — без ROS, только OpenCV.

Использование:
  # 1. Показать все доступные V4L2-устройства и по-path имена:
  python3 test_camera.py --list

  # 2. Тест конкретного устройства (по номеру или по пути):
  python3 test_camera.py --device /dev/video2
  python3 test_camera.py --device /dev/v4l/by-path/platform-xhci-...
  python3 test_camera.py --device 2          # эквивалентно /dev/video2

  # 3. Тест без окна (headless / SSH без -X):
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


# ---------------------------------------------------------------------------
# Вспомогательные функции
# ---------------------------------------------------------------------------

def list_cameras() -> None:
    """Выводит все /dev/video* и соответствующие by-path симлинки."""
    videos = sorted(glob.glob('/dev/video*'))
    if not videos:
        print('Устройства /dev/video* не найдены.')
        return

    # Собираем карту realpath → by-path
    by_path_map: dict[str, list[str]] = {}
    for link in sorted(glob.glob('/dev/v4l/by-path/*')):
        real = os.path.realpath(link)
        by_path_map.setdefault(real, []).append(link)

    print(f'{"УСТРОЙСТВО":<20}  {"DRIVER/NAME":<30}  BY-PATH')
    print('-' * 90)
    for dev in videos:
        # Читаем имя камеры через v4l2-ctl (если установлен)
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
        paths_str = '\n' + ' ' * 52 + ('\n' + ' ' * 52).join(paths) if paths else ''
        print(f'{dev:<20}  {name:<30}  {paths[0] if paths else "—"}{paths_str}')


def open_camera(device: str, width: int, height: int, fps: float) -> cv2.VideoCapture:
    """Открывает камеру через V4L2 и задаёт параметры."""
    # Принимаем как путь, так и целое число
    if device.lstrip('-').isdigit():
        src = int(device)
        cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
    else:
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f'[ERROR] Не удалось открыть камеру: {device}', file=sys.stderr)
        sys.exit(1)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS,          fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)
    return cap


def print_camera_info(cap: cv2.VideoCapture, device: str) -> None:
    """Выводит реальные параметры захвата после открытия."""
    actual_w   = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h   = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    fmt_raw    = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc     = ''.join(chr((fmt_raw >> i) & 0xFF) for i in (0, 8, 16, 24))

    print(f'  Устройство : {device}')
    print(f'  Разрешение : {actual_w} x {actual_h}')
    print(f'  FPS        : {actual_fps}')
    print(f'  Формат     : {fourcc}')


# ---------------------------------------------------------------------------
# Захват и отображение
# ---------------------------------------------------------------------------

def run_capture(cap: cv2.VideoCapture, no_display: bool) -> None:
    """Основной цикл захвата. Считает реальный FPS и показывает кадры."""
    window = 'Camera test  [q - выход]'
    if not no_display:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    frame_count  = 0
    fps_measured = 0.0
    t_start      = time.monotonic()
    t_fps        = t_start

    print('\nЗахват запущен. Нажмите q для выхода (или Ctrl-C).\n')

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print('[WARN] Кадр не получен')
                time.sleep(0.05)
                continue

            frame_count += 1
            now = time.monotonic()

            # Пересчитываем FPS раз в секунду
            if now - t_fps >= 1.0:
                fps_measured = frame_count / (now - t_start)
                elapsed = now - t_start
                print(
                    f'\r  Кадров: {frame_count:6d}  |  '
                    f'FPS: {fps_measured:5.1f}  |  '
                    f'Время: {elapsed:6.1f} с  ',
                    end='', flush=True,
                )

            if not no_display:
                # Рисуем FPS поверх кадра
                cv2.putText(
                    frame,
                    f'FPS: {fps_measured:.1f}  frames: {frame_count}',
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (0, 255, 0), 2,
                )
                cv2.imshow(window, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), ord('Q'), 27):
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
# main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description='Тест захвата одной камеры (без ROS)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--list',       action='store_true',
                        help='Показать все доступные V4L2-устройства и выйти')
    parser.add_argument('--device',     default='0',
                        help='Путь к камере или номер (default: 0)')
    parser.add_argument('--width',      type=int,   default=640,
                        help='Ширина кадра (default: 640)')
    parser.add_argument('--height',     type=int,   default=480,
                        help='Высота кадра (default: 480)')
    parser.add_argument('--fps',        type=float, default=30.0,
                        help='Желаемый FPS (default: 30)')
    parser.add_argument('--no-display', action='store_true',
                        help='Не открывать окно (для headless / SSH)')

    args = parser.parse_args()

    if args.list:
        list_cameras()
        return

    print(f'\n=== Открываем камеру: {args.device} ===')
    cap = open_camera(args.device, args.width, args.height, args.fps)
    print('Параметры захвата:')
    print_camera_info(cap, args.device)

    run_capture(cap, args.no_display)
    cap.release()


if __name__ == '__main__':
    main()
