from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    action_cmd = Node(
        package='hello_moveit',
        executable='pick_place_repeat_action_server',
        parameters=[PathJoinSubstitution(
            [FindPackageShare('hello_moveit'), 'config', 'pick_place_repeat_params.yaml']),
            {"waypoints_file": PathJoinSubstitution([FindPackageShare('hello_moveit'), 'config', 'waypoints.yaml'])},
            ],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(action_cmd)

    return ld
