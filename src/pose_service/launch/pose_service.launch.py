from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the node pose_service with parameter files."""
    return LaunchDescription(
        [
            Node(
                package="pose_service",
                executable="pose_service",
                name="pose_service",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("pose_service"), "config", "camera_param.yaml"]),
                    PathJoinSubstitution([FindPackageShare("pose_service"), "config", "fiducial_marker_param.yaml"]),
                ],
            )
        ]
    )
