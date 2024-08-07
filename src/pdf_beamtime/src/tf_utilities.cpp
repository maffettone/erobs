/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pdf_beamtime/tf_utilities.hpp>


TFUtilities::TFUtilities(const rclcpp::Node::SharedPtr node)
: node_(node), tf_util_logger_(node_->get_logger().get_child("tf_util"))
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "aruco_pose");

  while (!parameters_client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(tf_util_logger_, "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(tf_util_logger_, "Waiting for the parameters service to be available...");
  }

  sample_frame = parameters_client_->get_parameter<std::string>("sample_name");
  pre_pickup_approach_point_frame = parameters_client_->get_parameter<std::string>(
    "pre_pickup_location.name");

  world_frame = "world";
  grasping_point_on_gripper_frame = "grasping_point_link";
  wrist_2_frame = "wrist_2_link";
}


double TFUtilities::degreesToRadians(double degrees)
{
  return degrees * M_PI / 180.0;
}

std::pair<double, double> TFUtilities::get_wrist_elbow_alignment(
  moveit::planning_interface::MoveGroupInterface & mgi)
{
  tf2::Quaternion tf2_wrist_2_quaternion;
  tf2::Quaternion tf2_sample_quaternion;

  geometry_msgs::msg::TransformStamped transform_world_to_sample;
  geometry_msgs::msg::TransformStamped transform_world_to_wrist_2;

  double wrist_2_roll, wrist_2_pitch, wrist_2_yaw;
  double sample_roll, sample_pitch, sample_yaw;

  while (rclcpp::ok()) {
    try {
      transform_world_to_sample =
        tf_buffer_->lookupTransform(world_frame, sample_frame, tf2::TimePointZero);
      transform_world_to_wrist_2 =
        tf_buffer_->lookupTransform(world_frame, wrist_2_frame, tf2::TimePointZero);

      tf2::fromMsg(transform_world_to_wrist_2.transform.rotation, tf2_wrist_2_quaternion);
      tf2::fromMsg(transform_world_to_sample.transform.rotation, tf2_sample_quaternion);

      // // Convert tf2::Quaternion to RPY
      tf2::Matrix3x3(tf2_wrist_2_quaternion).getRPY(wrist_2_roll, wrist_2_pitch, wrist_2_yaw);
      tf2::Matrix3x3(tf2_sample_quaternion).getRPY(sample_roll, sample_pitch, sample_yaw);

      break;
    } catch (tf2::TransformException & ex) {
    }
  }

  // Get current joint values
  std::vector<double> joint_group_positions;
  mgi.getCurrentState()->copyJointGroupPositions(
    mgi.getCurrentState()->getRobotModel()->getJointModelGroup("ur_arm"),
    joint_group_positions
  );

  return std::make_pair(
    (joint_group_positions[4] + wrist_2_yaw - sample_yaw),
    sample_yaw * 180 / M_PI);
}

std::vector<geometry_msgs::msg::Pose> TFUtilities::get_pickup_action_z_adj(
  moveit::planning_interface::MoveGroupInterface & mgi)
{
  // Define waypoints for Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::TransformStamped transform_world_to_grasping_point;
  geometry_msgs::msg::TransformStamped transform_world_to_pickup_approach_point;

  double z_dist_to_pickup_approach = 0.0;

  while (rclcpp::ok()) {
    try {
      transform_world_to_grasping_point = tf_buffer_->lookupTransform(
        world_frame,
        grasping_point_on_gripper_frame,
        tf2::TimePointZero);

      transform_world_to_pickup_approach_point = tf_buffer_->lookupTransform(
        world_frame,
        pre_pickup_approach_point_frame,
        tf2::TimePointZero);

      z_dist_to_pickup_approach =
        transform_world_to_pickup_approach_point.transform.translation.z -
        (transform_world_to_grasping_point.transform.translation.z);

      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        tf_util_logger_, "Could not transform %s to %s: %s",
        grasping_point_on_gripper_frame.c_str(),
        sample_frame.c_str(), ex.what());
    }
  }

  geometry_msgs::msg::Pose target_pose = mgi.getCurrentPose().pose;
  // Move 2 cm down in the z direction
  target_pose.position.z += z_dist_to_pickup_approach;
  target_pose.position.z += -0.02;

  waypoints.push_back(target_pose);

  return waypoints;
}

std::vector<geometry_msgs::msg::Pose> TFUtilities::get_pickup_action_pre_pickup(
  moveit::planning_interface::MoveGroupInterface & mgi)
{
  // Define waypoints for Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::TransformStamped transform_world_to_grasping_point;
  geometry_msgs::msg::TransformStamped transform_world_to_pickup_approach_point;

  double x_dist_to_pickup_approach = 0.0, y_dist_to_pickup_approach = 0.0;

  while (rclcpp::ok()) {
    try {
      transform_world_to_grasping_point = tf_buffer_->lookupTransform(
        world_frame,
        grasping_point_on_gripper_frame,
        tf2::TimePointZero);

      transform_world_to_pickup_approach_point = tf_buffer_->lookupTransform(
        world_frame,
        pre_pickup_approach_point_frame,
        tf2::TimePointZero);

      x_dist_to_pickup_approach =
        transform_world_to_pickup_approach_point.transform.translation.x -
        transform_world_to_grasping_point.transform.translation.x;
      y_dist_to_pickup_approach =
        transform_world_to_pickup_approach_point.transform.translation.y -
        transform_world_to_grasping_point.transform.translation.y;

      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        tf_util_logger_, "Could not transform %s to %s: %s",
        grasping_point_on_gripper_frame.c_str(),
        sample_frame.c_str(), ex.what());
    }
  }

  geometry_msgs::msg::Pose target_pose = mgi.getCurrentPose().pose;

  int N = 10;
  // Calculate incremental distances
  double x_increment = x_dist_to_pickup_approach / N;
  double y_increment = y_dist_to_pickup_approach / N;

  // Loop to move in segments
  for (int i = 0; i < N; ++i) {
    target_pose.position.x += x_increment;
    target_pose.position.y += y_increment;

    // Set the new target pose
    waypoints.push_back(target_pose);
  }

  return waypoints;
}

std::vector<geometry_msgs::msg::Pose> TFUtilities::get_pickup_action_pickup(
  moveit::planning_interface::MoveGroupInterface & mgi)
{
  // Define waypoints for Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::TransformStamped transform_world_to_sample;
  geometry_msgs::msg::TransformStamped transform_world_to_pickup_approach_point;

  double x_dist_to_sample = 0.0, y_dist_to_sample = 0.0;

  while (rclcpp::ok()) {
    try {
      transform_world_to_sample =
        tf_buffer_->lookupTransform(world_frame, sample_frame, tf2::TimePointZero);
      transform_world_to_pickup_approach_point = tf_buffer_->lookupTransform(
        world_frame,
        pre_pickup_approach_point_frame,
        tf2::TimePointZero);

      x_dist_to_sample = transform_world_to_sample.transform.translation.x -
        transform_world_to_pickup_approach_point.transform.translation.x;
      y_dist_to_sample = transform_world_to_sample.transform.translation.y -
        transform_world_to_pickup_approach_point.transform.translation.y;

      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        tf_util_logger_, "Could not transform %s to %s: %s",
        grasping_point_on_gripper_frame.c_str(),
        sample_frame.c_str(), ex.what());
    }
  }

  geometry_msgs::msg::Pose target_pose = mgi.getCurrentPose().pose;

  target_pose.position.x += x_dist_to_sample;
  target_pose.position.y += y_dist_to_sample;
  waypoints.push_back(target_pose);

  return waypoints;
}
