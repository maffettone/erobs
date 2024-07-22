from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch the node aruco_pose with parameter files."""
    return LaunchDescription(
        [
            Node(
                package="aruco_pose",
                executable="aruco_pose",
                name="aruco_pose",
                output="screen",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("aruco_pose"), "config", "camera_param.yaml"]),
                    PathJoinSubstitution([FindPackageShare("aruco_pose"), "config", "fiducial_marker_param.yaml"]),
                ],
            )
        ]
    )
