from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='planif_locale',
            executable='odometry_node',
            name='wheel_odometry',
            output='screen',
        )

    ])
