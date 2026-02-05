from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    database_path_arg = DeclareLaunchArgument(
        'database_path',
        default_value=os.path.join(
            get_package_share_directory('rtabmap_localization'),
            'maps',
            'rtabmap.db'
        ),
        description='Chemin vers la base de données RTAB-Map'
    )

    database_path = LaunchConfiguration('database_path')

    # RTAB-Map en mode LOCALISATION
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'database_path': database_path,

            # --- Mode localisation ---
            'Mem/IncrementalMemory': 'false',
            'Mem/InitWMWithAllNodes': 'true',

            # --- Frames ---
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'publish_tf': True,

            # --- Capteurs ---
            'subscribe_scan': False, # à modifier plus tard
            'subscribe_rgb': False,
            'subscribe_depth': False,
        }]
    )

    # TF statique map -> odom
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # TF statique odom -> base_footprint
    static_odom_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_odom_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )

    map_node = Node(
        package='rtabmap_util',
        executable='map_assembler',
        name='map_assembler',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    fake_odom = Node(
        package='rtabmap_localization',
        executable='fake_odom',
        name='fake_odom',
        output='screen'
    )


    return LaunchDescription([
        database_path_arg,
        rtabmap_node,
        static_map_to_odom,
        #static_odom_to_base,
        map_node,
        fake_odom,
    ])
