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
                    FindPackageShare('fra333_lab4_01_description'),
                    'launch',
                    'sentinel_gazebo.launch.py'
                ])
            ])
    )

    controller_config = os.path.join(
        get_package_share_directory('fra333_lab4_01'),
        'config',
        'controller_parameters.yaml'
    )

    scheduler_config = os.path.join(
        get_package_share_directory('fra333_lab4_01'),
        'config',
        'viapoint.yaml'
    )

    scheduler_node = Node(
        package="fra333_lab4_01",
        executable="scheduler.py",
        parameters = [scheduler_config]
    )

    # event_handler = RegisterEventHandler(
    #         OnExecutionComplete(
    #             target_action=scheduler_node,
    #             on_completion=[
    #                 jog_node
    #             ]
    #         )
    # ),
    # jog_node = Node(
    #     package="fra333_lab4_01",
    #     executable="controller.py",
    #     parameters = [controller_config]
    # )
    # entity_to_run.add_action(sentinel_spawn)
    # entity_to_run.add_action(event_handler)
    entity_to_run.add_action(scheduler_node)
    return entity_to_run