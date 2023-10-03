from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    action_cmd = Node(
        package="pdf_beamtime",
        executable="pick_place_action_server",
        parameters=[
            PathJoinSubstitution([FindPackageShare("pdf_beamtime"), "config", "pick_place_repeat_params.yaml"]),
            {"waypoints_file": PathJoinSubstitution([FindPackageShare("pdf_beamtime"), "config", "waypoints.yaml"])},
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(action_cmd)

    return ld
