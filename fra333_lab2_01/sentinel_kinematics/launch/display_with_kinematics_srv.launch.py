#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

import sys

# from sentinel_kinematics.scripts.kinematics_server import Kinematics_server


def generate_launch_description():

    ### How to launch another launch file ###
    #
    # You must specify the package, folder, and the name
    #
    # display = IncludeLaunchDescription(
    #
    #      PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('package_name'),
    #             'folder_name',
    #             'launch_file.launch.py'
    #         ])
    #     ])
    # )
    
    # Launch Description
    launch_description = LaunchDescription()
    display = IncludeLaunchDescription(
         PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('sentinel_description'),
                'launch',
                'display.launch.py'
            ])
        ])
    )
    kinematics_server = Node(
        package="sentinel_kinematics",
        executable="kinematics_server.py"
    )
    
    launch_description.add_action(display)
    launch_description.add_action(kinematics_server)

    return launch_description

def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()

if __name__ == '__main__':
    main()
    