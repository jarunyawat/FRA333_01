import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnExecutionComplete
from launch.actions import RegisterEventHandler

def generate_launch_description():
    entity_to_run = LaunchDescription()
    sentinel_spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sentinel_description'),
                    'launch',
                    'sentinel_gazebo.launch.py'
                ])
            ])
    )

    controller_config = os.path.join(
        get_package_share_directory('sentinel_control'),
        'config',
        'tracker_config.yaml'
    )

    scheduler_config = os.path.join(
        get_package_share_directory('sentinel_control'),
        'config',
        'viapoint.yaml'
    )

    scheduler_node = Node(
        package="sentinel_control",
        executable="scheduler.py",
        # parameters = [scheduler_config]
    )

    jog_node = Node(
        package="sentinel_control",
        executable="tracker.py",
        parameters = [controller_config]
    )

    generate_node = Node(
        package="sentinel_control",
        executable="generator.py",
    )

    entity_to_run.add_action(sentinel_spawn)
    entity_to_run.add_action(jog_node)
    entity_to_run.add_action(generate_node)
    return entity_to_run