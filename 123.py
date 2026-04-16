from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # --- Arguments ---
    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        default_value='demeter',
        description='Unique robot name'
    )
    use_namespace_arg = DeclareLaunchArgument(
        name='use_namespace',
        default_value='false',
        choices=['true', 'false'],
        description='Use robot name as namespace'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Use simulation (Gazebo) clock if true'
    )
    localization_arg = DeclareLaunchArgument(
        'localization', 
        default_value='false',
        choices=['true', 'false'],
        description='Apply localozaton mode in rtabmap'
        )
    rtabmap_viz_arg = DeclareLaunchArgument(
        'rtabmap_viz', 
        default_value='false',
        choices=['true', 'false'],
        description='Apply rtabmap vizualization')

    robot_name = LaunchConfiguration('robot_name')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')

    stereo_to_rgbd = GroupAction(
        actions=[
            PushRosNamespace(
                condition=IfCondition(use_namespace), 
                namespace=robot_name
            ),
            Node(
                package='rtabmap_sync',
                executable='stereo_sync',
                name='stereo_sync_front',
                namespace='front_cams',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'approx_sync': False,
                    'qos': 2,
                    'qos_camera_info': 2,
                    'queue_size': 10,
                    'sync_queue_size': 10,
                    'approx_sync_max_interval': 0.1  # с
                }],
                remappings=[
                    ('left/camera_info', 'f_left_camera/camera_info'),
                    ('left/image_rect', 'f_left_camera/image'),
                    ('right/camera_info', 'f_right_camera/camera_info'),
                    ('right/image_rect', 'f_right_camera/image'),
                ]
            ),
            Node(
                package='rtabmap_sync',
                executable='stereo_sync',
                name='stereo_sync_left',
                namespace='left_cams',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'approx_sync': False,
                    'qos': 2,
                    'qos_camera_info': 2,
                    'queue_size': 10,
                    'sync_queue_size': 10,
                    'approx_sync_max_interval': 0.1  # с
                }],
                remappings=[
                    ('left/camera_info', 'l_left_camera/camera_info'),
                    ('left/image_rect', 'l_left_camera/image'),
                    ('right/camera_info', 'l_right_camera/camera_info'),
                    ('right/image_rect', 'l_right_camera/image'),
                ]
            ),
            Node(
                package='rtabmap_sync',
                executable='stereo_sync',
                name='stereo_sync_right',
                namespace='right_cams',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'approx_sync': False,
                    'qos': 2,
                    'qos_camera_info': 2,
                    'queue_size': 10,

                    'sync_queue_size': 10,
                    'approx_sync_max_interval': 0.1  # с
                }],
                remappings=[
                    ('left/camera_info', 'r_left_camera/camera_info'),
                    ('left/image_rect', 'r_left_camera/image'),
                    ('right/camera_info', 'r_right_camera/camera_info'),
                    ('right/image_rect', 'r_right_camera/image'),
                ]
            ),
        ]
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_mag': False,       # без магнетометра
            'publish_tf': False,    # TF публикует rtabmap, не фильтр
            'world_frame': 'enu',
            'frequency': 500.0,
        }],
        remappings=[
            ('imu/data_raw', 'imu/data'),        # ваш сырой топик
            ('imu/data',     'imu/data_filtered'),
        ]
    )

    rtabmap_bringup = GroupAction(
        actions=[
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                output='screen',
                parameters=[{
                    'use_sim_time' : use_sim_time,
                    'rgbd_cameras':3,
                    'frame_id': 'base_link',
                    'odom_frame_id': 'odom',  # Фрейм одометрии
                    'map_frame_id': 'map',
                    'subscribe_rgbd': True,
                    'subscribe_odom': True,
                    #'odom_topic': '/diff_drive_controller/odom',  # ПРАВИЛЬНЫЙ ТОПИК!
                    'approx_sync': False,
                    'subscribe_odom_info': False,
                    'queue_size': 100,
                    'sync_queue_size': 100,
                    'Mem/IncrementalMemory': 'true',
                    'RGBD/ProximityBySpace': 'true',
                    'RGBD/AngularUpdate': '0.01',
                    'RGBD/LinearUpdate': '0.01',
                    # 'Rtabmap/DetectionRate': '30',
                    'Grid/FromDepth': 'true',
                }],
                remappings=[
                    ("rgbd_image0", 'front_cams/rgbd_image'),
                    ("rgbd_image1", 'left_cams/rgbd_image'),
                    ("rgbd_image2", 'right_cams/rgbd_image'),
                    ('odom', 'odom'),
                    # ('odom', 'diff_drive_controller/odom'),
                    # ('gps/fix', 'navsat/fix'),
                    ('imu',         'imu/data_filtered'),   # фильтрованный IMU!
                ],
                arguments=['-d'],  # Удалить старую БД
                condition=UnlessCondition(localization)
            ),
            Node(
                package='rtabmap_odom',
                executable='stereo_odometry',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'frame_id': 'base_link',
                    'odom_frame_id': 'odom',
                    'subscribe_rgbd': True,
                    'rgbd_cameras': 3,
                    'approx_sync': False,
                    'wait_imu_to_init': True,    # ждать IMU перед стартом
                    'Vis/FeatureType': "8",          # GFTT/Harris - лучше для однородных поверхностей
                    'Vis/GridRows': "4",             # Делить кадр на 4 строки
                    'Vis/GridCols': "4",             # и 4 столбца -> 16 ячеек, признаки в каждой
                    'Vis/MaxFeatures': "600",        # Увеличить кол-во признаков
                    'OdomF2M/MaxSize': "2000",       # Больший локальный map
                    'Odom/Holonomic': "false",       # Твой робот не-голономный (гусеницы!)
                    # 'Imu/FilteringStrategy': '1',
                }],
                remappings=[
                    ("rgbd_image0", 'front_cams/rgbd_image'),
                    ("rgbd_image1", 'left_cams/rgbd_image'),

                    ("rgbd_image2", 'right_cams/rgbd_image'),
                    ('imu',         'imu/data_filtered'),   # фильтрованный IMU!
                    ('odom', 'odom'),
                ],
                condition=UnlessCondition(localization)
            ),
            Node(
                package='rtabmap_viz',
                executable='rtabmap_viz',
                output='screen',
                condition=IfCondition(rtabmap_viz),
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'frame_id': 'base_link',
                    'approx_sync': False,
                    'subscribe_odom_info': False,
                    'subscribe_stereo': False,
                    'queue_size': 100,
                    'sync_queue_size': 100,
                }],
                remappings=[
                    ('left/image_rect', 'front_cams/front_left_camera/image'),
                    ('right/image_rect', 'front_cams/front_right_camera/image'),
                    ('left/camera_info', 'front_cams/front_left_camera/camera_info'),
                    ('right/camera_info', 'front_cams/front_right_camera/camera_info'),
                    ('odom', 'odom'),
                    # ('odom', '/diff_drive_controller/odom'),
                ]
            ),
        ]
    )

    return LaunchDescription([
        robot_name_arg,
        use_namespace_arg,
        use_sim_time_arg,
        localization_arg,
        rtabmap_viz_arg,

        stereo_to_rgbd,
        imu_filter,
        rtabmap_bringup,
    ])