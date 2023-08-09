import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='two_nodes',
            executable='pub',
            # name='pub',
            parameters=[os.path.join(get_package_share_directory
            ('two_nodes'), 'config', 'pub_params.yaml')],
            output="screen",
            emulate_tty=True
        ),
        launch_ros.actions.Node(
            package='two_nodes',
            executable='sub',
            # name='sub',
            parameters=[os.path.join(get_package_share_directory
            ('two_nodes'), 'config', 'sub_params.yaml')],
            output="screen",
            emulate_tty=True
        )
    ])
