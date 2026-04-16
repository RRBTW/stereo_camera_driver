"""
Launch-файл: запускает три экземпляра stereo_camera_node.

Каждая нода открывает ОДИН USB-девайс (/dev/stereo_camN),
захватывает MJPG кадр 1280×480, делит пополам и публикует:

  namespace front_cams  (камера 0, /dev/stereo_cam0):
    /front_cams/f_left_camera/image         /front_cams/f_left_camera/camera_info
    /front_cams/f_right_camera/image        /front_cams/f_right_camera/camera_info

  namespace left_cams   (камера 1, /dev/stereo_cam1):
    /left_cams/l_left_camera/image          /left_cams/l_left_camera/camera_info
    /left_cams/l_right_camera/image         /left_cams/l_right_camera/camera_info

  namespace right_cams  (камера 2, /dev/stereo_cam2):
    /right_cams/r_left_camera/image         /right_cams/r_left_camera/camera_info
    /right_cams/r_right_camera/image        /right_cams/r_right_camera/camera_info

Итого 12 топиков — 4 на камеру (image + camera_info для left и right).
Совместимо с remappings в 123.py (rtabmap_sync/stereo_sync).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# =============================================================================
# Привязка физических портов к namespace-ам
# Симлинки создаются udev-правилами из /etc/udev/rules.d/99-stereo-camera.rules
# =============================================================================
CAMERA_CONFIG = [
    # (namespace,     device_path,          prefix, TF-prefix)
    ('front_cams',  '/dev/stereo_cam0',    'f'),   # хаб, порт 2.2
    ('left_cams',   '/dev/stereo_cam1',    'l'),   # прямой порт 3
    ('right_cams',  '/dev/stereo_cam2',    'r'),   # прямой порт 1
]
# =============================================================================


def _make_stereo_node(namespace, device_path, prefix, fps, width, height) -> Node:
    """
    Один экземпляр stereo_camera_node.

    Нода читает широкий кадр (frame_width × frame_height), делит пополам,
    публикует left/image_raw и right/image_raw в своём namespace.

    Ремапинги приводят топики к виду, который ожидает stereo_sync:
      left/image_raw   → <prefix>_left_camera/image
      left/camera_info → <prefix>_left_camera/camera_info
      (аналогично для right)
    Итоговые абсолютные пути:
      /<namespace>/<prefix>_left_camera/image   и т.д.
    """
    p = prefix
    return Node(
        package='stereo_camera_driver',
        executable='stereo_camera_node',
        name=f'stereo_camera_node_{namespace}',
        namespace=namespace,
        output='screen',
        parameters=[{
            'device_path':    device_path,
            'camera_fps':     fps,
            'frame_width':    width,
            'frame_height':   height,
            'frame_id_left':  f'{namespace}/{p}_left_camera_optical',
            'frame_id_right': f'{namespace}/{p}_right_camera_optical',
        }],
        remappings=[
            ('left/image_raw',   f'{p}_left_camera/image'),
            ('left/camera_info', f'{p}_left_camera/camera_info'),
            ('right/image_raw',  f'{p}_right_camera/image'),
            ('right/camera_info',f'{p}_right_camera/camera_info'),
        ],
    )


def generate_launch_description():
    fps_arg    = DeclareLaunchArgument(
        'camera_fps',   default_value='30.0',
        description='FPS захвата для всех камер')
    width_arg  = DeclareLaunchArgument(
        'frame_width',  default_value='1280',
        description='Ширина широкого кадра (side-by-side), напр. 1280')
    height_arg = DeclareLaunchArgument(
        'frame_height', default_value='480',
        description='Высота кадра, напр. 480')

    fps    = ParameterValue(LaunchConfiguration('camera_fps'),   value_type=float)
    width  = ParameterValue(LaunchConfiguration('frame_width'),  value_type=int)
    height = ParameterValue(LaunchConfiguration('frame_height'), value_type=int)

    nodes = [
        _make_stereo_node(ns, dev, prefix, fps, width, height)
        for ns, dev, prefix in CAMERA_CONFIG
    ]

    return LaunchDescription([fps_arg, width_arg, height_arg, *nodes])
