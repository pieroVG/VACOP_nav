from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_dir = get_package_share_directory('gps_localization')

    return LaunchDescription([

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom',
            parameters=[os.path.join(pkg_dir, 'config/ekf_odom.yaml')]
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=[os.path.join(pkg_dir, 'config/navsat.yaml')],
            remappings=[
                ('gps/fix', '/vacop/gps/fix'),
                ('imu/data', '/imu/data'),
                ('odometry/filtered', '/odometry/filtered')
            ]
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_map',
            parameters=[os.path.join(pkg_dir, 'config/ekf_map.yaml')]
        ),
    ])

