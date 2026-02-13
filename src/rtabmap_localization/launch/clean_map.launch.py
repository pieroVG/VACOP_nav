from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    map_yaml_path = os.path.join(
        get_package_share_directory('rtabmap_localization'),
        'maps',
        'carte_nettoyee.yaml'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'use_sim_time': False
        }]
    )

    # TF statique map -> odom
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    fake_odom = Node(
        package='rtabmap_localization',
        executable='fake_odom',
        name='fake_odom',
        output='screen'
    )

    return LaunchDescription([
        map_server,
        lifecycle_manager,
        fake_odom,
        static_map_to_odom,
    ])
