#!usr/bin/python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess,DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
def generate_launch_description():
    # create a place holder for launch description
    launch_description = LaunchDescription()

    ### Example for adding launch argument ###
    # v_max = LaunchConfiguration('v_max')
    # v_max_launch_arg = DeclareLaunchArgument('v_max',default_value='1.0')
    # launch_description.add_action(v_max_launch_arg)
    
    ### Example for adding a node ###
    # node = Node(
    #     package='my_package',
    #     executable='my_executable',
    #     namespace= 'this_namespace',
    #     arguments=[v_max],
    #     remappings=[
    #         ('/topic_1','/topic_a'),
    #         ('/topic_2','/topic_b'),
    #     ]
    # )
    # launch_description.add_action(node)

    ### Example for execute a shell command in python script ###
    # vx = 1.0
    # wz = 1.0
    # pub_cmd_vel = ExecuteProcess(
    #     cmd = [[f'ros2 topic pub -r 10 /turtle1/cmd_vel geometry_msgs/msg/Twist "{{linear: {{x: {vx}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {wz}}}}}"']],
    #     shell=True
    # )
    # launch_description.add_action(pub_cmd_vel)
    rate_config = LaunchConfiguration("/rate")
    rate_config_arg = DeclareLaunchArgument(
        "/rate",
        default_value="5.0"
    )
    turtle = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    linear_noise_gen = Node(
        namespace="/linear",
        package="fra333_lab1_01",
        executable="noise_generator.py",
        arguments=[rate_config],
    )
    angular_noise_gen = Node(
        namespace="/angular",
        package="fra333_lab1_01",
        executable="noise_generator.py",
        arguments=[rate_config],
    )
    velocity_mux = Node(
        package="fra333_lab1_01",
        executable="velocity_mux.py",
                remappings=[
            ("/linear/velocity","/linear/noise"),
            ("/angular/velocity","/angular/noise")
        ],
        arguments=[rate_config],
    )
    entity_to_run = [rate_config_arg,turtle,linear_noise_gen,angular_noise_gen,velocity_mux]
    
    return LaunchDescription(entity_to_run)