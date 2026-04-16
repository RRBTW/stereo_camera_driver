# stereo_camera_driver

ROS 2 драйвер для трёх USB стереокамер (side-by-side, один USB на камеру).

Каждая камера выдаёт один широкий кадр (1280×480 MJPG), в котором
левая половина — левый объектив, правая — правый. Нода захватывает кадр
через прямые V4L2 ioctl (обходит сброс формата OpenCV), делит пополам и
публикует 4 топика на камеру — итого **12 топиков**.

---

## Топики

| Namespace | Устройство | Топики |
|---|---|---|
| `/front_cams` | `/dev/stereo_cam0` | `f_left_camera/image`, `f_left_camera/camera_info`, `f_right_camera/image`, `f_right_camera/camera_info` |
| `/left_cams`  | `/dev/stereo_cam1` | `l_left_camera/image`, `l_left_camera/camera_info`, `l_right_camera/image`, `l_right_camera/camera_info` |
| `/right_cams` | `/dev/stereo_cam2` | `r_left_camera/image`, `r_left_camera/camera_info`, `r_right_camera/image`, `r_right_camera/camera_info` |

Типы сообщений: `sensor_msgs/Image` (encoding `bgr8`) и `sensor_msgs/CameraInfo`.

---

## Структура пакета

```
stereo_camera_driver/
├── stereo_camera_driver/
│   └── stereo_camera_node.py   # ROS 2 нода
├── launch/
│   └── stereo_cameras.launch.py
├── test_camera.py              # автономный тест без ROS
├── 123.py                      # launch для rtabmap + stereo_sync
├── package.xml
└── setup.py
```

---

## Установка

### Зависимости

```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge \
                 ros-$ROS_DISTRO-image-transport \
                 ros-$ROS_DISTRO-sensor-msgs \
                 python3-opencv
```

### Сборка

```bash
cd ~/ros2_ws/src
ln -s /home/d-grape/Downloads/stereo_camera_driver .   # или скопируйте
cd ~/ros2_ws
colcon build --packages-select stereo_camera_driver
source install/setup.bash
```

---

## Стабильные симлинки устройств

Камеры привязаны к физическим USB-портам через udev. Симлинки не меняются
при переподключении.

Правила: `/etc/udev/rules.d/99-stereo-camera.rules`

| Симлинк | Порт | Vendor:Model |
|---|---|---|
| `/dev/stereo_cam0` | хаб, порт 2.2 | `1bcf:c001` |
| `/dev/stereo_cam1` | прямой порт 3 | `1bcf:c001` |
| `/dev/stereo_cam2` | прямой порт 1 | `1bcf:c001` |

Применить правила:
```bash
sudo cp /tmp/99-stereo-camera.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux
ls -la /dev/stereo_cam*
```

Найти путь новой камеры:
```bash
udevadm info /dev/videoN | grep ID_PATH
```

---

## Запуск

### Все три камеры (ROS 2)

```bash
ros2 launch stereo_camera_driver stereo_cameras.launch.py
```

Параметры командной строки:

| Параметр | По умолчанию | Описание |
|---|---|---|
| `camera_fps` | `30.0` | FPS захвата |
| `frame_width` | `1280` | Ширина широкого кадра |
| `frame_height` | `480` | Высота кадра |

Пример с другими параметрами:
```bash
ros2 launch stereo_camera_driver stereo_cameras.launch.py camera_fps:=15.0
```

### Тест без ROS

Скрипт `test_camera.py` работает автономно — без ROS, только OpenCV.

```bash
# Список устройств
python3 test_camera.py --list

# Одна стереокамера
python3 test_camera.py --device /dev/stereo_cam0 --width 1280 --height 480 --split

# Две стереокамеры
python3 test_camera.py --device /dev/stereo_cam0 --device2 /dev/stereo_cam1 \
    --width 1280 --height 480 --split

# Три стереокамеры
python3 test_camera.py --device /dev/stereo_cam0 --device2 /dev/stereo_cam1 \
    --device3 /dev/stereo_cam2 --width 1280 --height 480 --split

# Без окна (headless / SSH)
python3 test_camera.py --device /dev/stereo_cam0 --width 1280 --height 480 \
    --split --no-display
```

Окно отображает все камеры вертикально, каждая строка — одна пара LEFT/RIGHT:

```
┌──────────────┬──────────────┐
│  CAM0  LEFT  │  CAM0  RIGHT │
├──────────────┼──────────────┤
│  CAM1  LEFT  │  CAM1  RIGHT │
├──────────────┼──────────────┤
│  CAM2  LEFT  │  CAM2  RIGHT │
└──────────────┴──────────────┘
```

Выход: `q` или `Ctrl-C`.

---

## Параметры ноды

| Параметр | Тип | По умолчанию | Описание |
|---|---|---|---|
| `device_path` | string | `/dev/stereo_cam0` | Путь к устройству |
| `camera_fps` | double | `30.0` | Желаемый FPS |
| `frame_width` | int | `1280` | Ширина широкого кадра |
| `frame_height` | int | `480` | Высота кадра |
| `frame_id_left` | string | `left_camera_optical` | TF frame левой камеры |
| `frame_id_right` | string | `right_camera_optical` | TF frame правой камеры |

---

## Интеграция с rtabmap

Файл `123.py` — launch для `rtabmap_sync/stereo_sync` + `rtabmap_slam` + `rtabmap_odom`.
Ожидает ровно те топики, которые публикует этот драйвер.

```bash
ros2 launch <your_package> 123.py
```

Схема данных:
```
stereo_camera_driver  →  stereo_sync (×3)  →  rgbd_image (×3)
                                                     ↓
                                          stereo_odometry + rtabmap
```

---

## Архитектура

```
MJPGCapture (V4L2 ioctl)
    │  прямой mmap захват MJPG 1280×480
    ▼
_FrameGrabber (фоновый поток)
    │  последний кадр без блокировки
    ▼
split_stereo()
    │  frame[:, :640]   frame[:, 640:]
    ▼                        ▼
left 640×480            right 640×480
    │                        │
cv2_to_imgmsg           cv2_to_imgmsg
    │                        │
left/image_raw          right/image_raw
left/camera_info        right/camera_info
```

`CameraInfo` публикуется с нулевой калибровкой. Для работы глубины
необходимо откалибровать камеры через `camera_calibration` и заменить
значения `K`, `D`, `R`, `P` в `_make_default_camera_info()`.

---

## Ограничения

- Все камеры на USB 2.0 (480 Мбит/с). Три камеры одновременно работают
  только если хотя бы две подключены к **разным** физическим контроллерам
  (прямые порты ПК, а не через один хаб). При нехватке полосы ядро
  возвращает `ENOSPC` (errno 28) при `VIDIOC_STREAMON`.
- `CameraInfo` содержит нулевую калибровку — необходима калибровка для
  получения карт глубины.
