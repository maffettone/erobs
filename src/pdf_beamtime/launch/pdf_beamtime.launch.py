from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the node obstacle_builder with a parameter file."""
    action_cmd = Node(
        package="pdf_beamtime",
        executable="simple_server",
        parameters=[
            PathJoinSubstitution([FindPackageShare("pdf_beamtime"), "config", "obstacles.yaml"]),
            PathJoinSubstitution([FindPackageShare("pdf_beamtime"), "config", "joint_poses.yaml"]),
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(action_cmd)

    return ld
