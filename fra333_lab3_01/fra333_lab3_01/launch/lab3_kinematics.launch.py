import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    entity_to_run = LaunchDescription()
    sentinel_spawn = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('fra333_lab3_01_description'),
                    'launch',
                    'sentinel_gazebo.launch.py'
                ])
            ])
    )
    imu_calibrate_node = Node(
        package="fra333_lab3_01",
        executable="imu_calibration.py"
    )
    jog_node = Node(
        package="fra333_lab3_01",
        executable="trajectory_gen.py"
    )
    imu_filter = Node(
        package="imu_complementary_filter",
        executable="complementary_filter_node"
    )
    entity_to_run.add_action(sentinel_spawn)
    entity_to_run.add_action(imu_calibrate_node)
    entity_to_run.add_action(imu_filter)
    entity_to_run.add_action(jog_node)
    return entity_to_run