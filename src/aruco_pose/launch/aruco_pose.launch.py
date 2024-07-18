from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import pdb

def generate_launch_description():
    """Launch the node aruco_pose with parameter files."""
    action_cmd = Node(
        package="aruco_pose",
        executable="aruco_pose",
                parameters=[
                    PathJoinSubstitution([FindPackageShare("aruco_pose"), "config", "camera_param.yaml"]),
                    PathJoinSubstitution([FindPackageShare("aruco_pose"), "config", "fiducial_marker_param.yaml"]),
                ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(action_cmd)

    cam_par = PathJoinSubstitution([FindPackageShare("aruco_pose"), "config", "camera_param.yaml"])
    
    pdb.set_trace()
    return ld
