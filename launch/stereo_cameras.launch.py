"""
Launch-файл: запускает три экземпляра stereo_camera_node.

Топики, которые ожидает потребитель (rtabmap_sync/stereo_sync):

  namespace front_cams  (центральная пара):
    /front_cams/f_left_camera/image         /front_cams/f_left_camera/camera_info
    /front_cams/f_right_camera/image        /front_cams/f_right_camera/camera_info

  namespace left_cams   (левая пара):
    /left_cams/l_left_camera/image          /left_cams/l_left_camera/camera_info
    /left_cams/l_right_camera/image         /left_cams/l_right_camera/camera_info

  namespace right_cams  (правая пара):
    /right_cams/r_left_camera/image         /right_cams/r_left_camera/camera_info
    /right_cams/r_right_camera/image        /right_cams/r_right_camera/camera_info

КАК НАЙТИ ПУТИ КАМЕР:
  1. Подключите все 6 камер.
  2. ls -la /dev/v4l/by-path/
  3. Определите каждый порт (отключая по одной камере).
  4. Подставьте пути в CAMERA_PATHS ниже.

ПЕРЕОПРЕДЕЛЕНИЕ ПАРАМЕТРОВ ИЗ КОМАНДНОЙ СТРОКИ:
  ros2 launch stereo_camera_driver stereo_cameras.launch.py \
      camera_fps:=15.0 frame_width:=1280 frame_height:=720
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument  # type: ignore[import-untyped]
from launch.substitutions import LaunchConfiguration  # type: ignore[import-untyped]
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # type: ignore[import-untyped]

# =============================================================================
# ЗАПОЛНИТЕ РЕАЛЬНЫМИ ПУТЯМИ из /dev/v4l/by-path/
# =============================================================================
CAMERA_PATHS = {
    # --- Центральная стереопара → namespace front_cams ---
    'front_cams': {
        'left':  '/dev/v4l/by-path/PLACEHOLDER_CENTER_PAIR_LEFT_CAM',   # TODO
        'right': '/dev/v4l/by-path/PLACEHOLDER_CENTER_PAIR_RIGHT_CAM',  # TODO
        'prefix': 'f',  # топики будут f_left_camera/... и f_right_camera/...
    },
    # --- Левая стереопара → namespace left_cams ---
    'left_cams': {
        'left':  '/dev/v4l/by-path/PLACEHOLDER_LEFT_PAIR_LEFT_CAM',     # TODO
        'right': '/dev/v4l/by-path/PLACEHOLDER_LEFT_PAIR_RIGHT_CAM',    # TODO
        'prefix': 'l',
    },
    # --- Правая стереопара → namespace right_cams ---
    'right_cams': {
        'left':  '/dev/v4l/by-path/PLACEHOLDER_RIGHT_PAIR_LEFT_CAM',    # TODO
        'right': '/dev/v4l/by-path/PLACEHOLDER_RIGHT_PAIR_RIGHT_CAM',   # TODO
        'prefix': 'r',
    },
}
# =============================================================================


def _make_stereo_node(
    namespace: str,
    left_path: str,
    right_path: str,
    prefix: str,
    fps,
    width,
    height,
) -> Node:
    """
    Создаёт Node для одного экземпляра stereo_camera_node.

    Нода публикует left/image_raw и right/image_raw в своём namespace.
    Ремапинги переименовывают топики так, как ожидает stereo_sync:
      left/image_raw  → <prefix>_left_camera/image
      right/image_raw → <prefix>_right_camera/image
      (аналогично camera_info)
    Итоговые абсолютные пути:
      /<namespace>/<prefix>_left_camera/image
      /<namespace>/<prefix>_right_camera/image
    """
    p = prefix
    return Node(
        package='stereo_camera_driver',
        executable='stereo_camera_node',
        name='stereo_camera_node',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'left_camera_path':  left_path,
                'right_camera_path': right_path,
                'frame_id_left':  f'{namespace}/{p}_left_camera_optical',
                'frame_id_right': f'{namespace}/{p}_right_camera_optical',
                'camera_fps':    fps,
                'frame_width':   width,
                'frame_height':  height,
            },
        ],
        remappings=[
            # image — тип Image; stereo_sync ремапит свой left/image_rect на этот топик
            ('left/image_raw',    f'{p}_left_camera/image'),
            ('left/camera_info',  f'{p}_left_camera/camera_info'),
            ('right/image_raw',   f'{p}_right_camera/image'),
            ('right/camera_info', f'{p}_right_camera/camera_info'),
        ],
    )


def generate_launch_description():
    # --- аргументы командной строки ---
    fps_arg    = DeclareLaunchArgument(
        'camera_fps',   default_value='30.0',
        description='Желаемый FPS захвата для всех камер')
    width_arg  = DeclareLaunchArgument(
        'frame_width',  default_value='640',
        description='Ширина кадра в пикселях')
    height_arg = DeclareLaunchArgument(
        'frame_height', default_value='480',
        description='Высота кадра в пикселях')

    fps    = ParameterValue(LaunchConfiguration('camera_fps'),    value_type=float)
    width  = ParameterValue(LaunchConfiguration('frame_width'),   value_type=int)
    height = ParameterValue(LaunchConfiguration('frame_height'),  value_type=int)

    camera_nodes = [
        _make_stereo_node(
            namespace=ns,
            left_path=cfg['left'],
            right_path=cfg['right'],
            prefix=cfg['prefix'],
            fps=fps,
            width=width,
            height=height,
        )
        for ns, cfg in CAMERA_PATHS.items()
    ]

    return LaunchDescription([fps_arg, width_arg, height_arg, *camera_nodes])
