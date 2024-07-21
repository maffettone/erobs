/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <chrono>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
/*
This needs robot_description and robot_description_semantic parameters somehow.
*/

// Function to convert degrees to radians
double degreesToRadians(double degrees)
{
  return degrees * M_PI / 180.0;
}

void doCartesianMovement(
  std::vector<geometry_msgs::msg::Pose> waypoints,
  moveit::planning_interface::MoveGroupInterface & move_group_interface, rclcpp::Logger logger)
{
  // Plan the Cartesian path
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
  double fraction = move_group_interface.computeCartesianPath(
    waypoints, 0.001, 0.0,
    cartesian_plan.trajectory_);

  if (fraction > 0.95) {
    RCLCPP_INFO(
      logger, "Cartesian path (%.2f%% acheived), moving the arm", fraction * 100.0);
    move_group_interface.execute(cartesian_plan);
  } else {
    RCLCPP_WARN(logger, "Cartesian path planning failed with %.2f%%", fraction * 100.0);
    rclcpp::shutdown();
  }
}

void doJointMovement(
  std::vector<double> joint_goal_degrees,
  moveit::planning_interface::MoveGroupInterface & move_group_interface, rclcpp::Logger logger)
{
  // Vector to hold the converted angles in radians
  std::vector<double> joint_goal_radians(joint_goal_degrees.size());

  // Convert each element from degrees to radians using std::transform
  std::transform(
    joint_goal_degrees.begin(), joint_goal_degrees.end(),
    joint_goal_radians.begin(), degreesToRadians);

  move_group_interface.setJointValueTarget(joint_goal_radians);
  // Create a plan to that target pose
  auto const [planing_success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface.plan(msg));
      return std::make_pair(ok, msg);
    }();
  if (planing_success) {

    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
    rclcpp::shutdown();         // Stop the node if planning fails
  }
}

int main(int argc, char * argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create a node for synchronously grabbing params
  auto parameter_client_node = rclcpp::Node::make_shared("param_client");
  auto parent_parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(parameter_client_node, "move_group");
  // Boiler plate wait block
  while (!parent_parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        logger, "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "move_group service not available, waiting again...");
  }

  // Get robot config parameters from parameter server
  auto parameters = parent_parameters_client->get_parameters(
    {"robot_description_semantic",
      "robot_description"});

  // We spin up a SingleThreadedExecutor for the current state monitor to get
  // information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(parameter_client_node);
  auto spinner = std::thread([&executor]() {executor.spin();});

  std::string parameter_value = parameters[0].value_to_string();
  parameter_client_node->declare_parameter<std::string>(
    "robot_description_semantic",
    parameter_value);
  parameter_value = parameters[1].value_to_string();
  parameter_client_node->declare_parameter<std::string>("robot_description", parameter_value);

  // Next step goes here
  // Create the MoveIt MoveGroup Interface
  RCLCPP_INFO(logger, "assembling move_group_interface");
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(parameter_client_node, "ur_arm");
  move_group_interface.setEndEffectorLink("wrist_3_link");

// Initialize the planning scene interface
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_ =
    std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

  // Define the collision object (box)
  std::vector<moveit_msgs::msg::CollisionObject> all_obstacles;

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = "table";

  // Define the shape and size of the box
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 2;    // x
  primitive.dimensions[1] = 2;    // y
  primitive.dimensions[2] = 0.01;    // z

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.01;

  // Add the primitive and pose to the collision object
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);

  // Add the collision object to the planning scene
  all_obstacles.push_back(collision_object);

  // Add the collision object to the planning scene
  planning_scene_interface_->applyCollisionObjects(all_obstacles);

  // ####################### REST
  // while (true) {
  std::vector<double> joint_goal_degrees = {293.24, -77.05, 119.62, -43.57, 199.87, 180.0};
  // std::vector<double> joint_goal_degrees = {235.14, -77.05, 119.62, -43.57, 148.03, 180.0};


  doJointMovement(joint_goal_degrees, move_group_interface, logger);

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // ####################### PRE PICK UP HEAD TURN

  // TF2 buffer and listener
  tf2_ros::Buffer tf_buffer(parameter_client_node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Lookup the transformation between two links
  std::string world_frame = "world";
  std::string to_sample = "sample_1";
  std::string grasping_point = "grasping_point_link";
  std::string to_wrist_2 = "wrist_2_link";
  std::string pickup_approach_point = "pre_pickup";

  geometry_msgs::msg::TransformStamped transform_world_to_sample;
  geometry_msgs::msg::TransformStamped transform_world_to_wrist_2;
  geometry_msgs::msg::TransformStamped transform_world_to_grasping_point;
  geometry_msgs::msg::TransformStamped transform_world_to_pickup_approach_point;

  tf2::Quaternion tf2_wrist_2_quaternion;
  tf2::Quaternion tf2_sample_quaternion;

  double x_dist_to_sample, y_dist_to_sample, z_dist_to_sample;
  double x_dist_to_pickup_approach, y_dist_to_pickup_approach, z_dist_to_pickup_approach;
  double wrist_2_roll, wrist_2_pitch, wrist_2_yaw;
  double sample_roll, sample_pitch, sample_yaw;

  while (rclcpp::ok()) {
    try {
      transform_world_to_sample =
        tf_buffer.lookupTransform(world_frame, to_sample, tf2::TimePointZero);
      transform_world_to_wrist_2 =
        tf_buffer.lookupTransform(world_frame, to_wrist_2, tf2::TimePointZero);

      tf2::fromMsg(transform_world_to_wrist_2.transform.rotation, tf2_wrist_2_quaternion);
      tf2::fromMsg(transform_world_to_sample.transform.rotation, tf2_sample_quaternion);

      // Convert tf2::Quaternion to RPY
      tf2::Matrix3x3(tf2_wrist_2_quaternion).getRPY(wrist_2_roll, wrist_2_pitch, wrist_2_yaw);
      tf2::Matrix3x3(tf2_sample_quaternion).getRPY(sample_roll, sample_pitch, sample_yaw);

      break;
    } catch (tf2::TransformException & ex) {
      // RCLCPP_ERROR(
      //   logger, "Could not transform %s to %s: %s", to_wrist.c_str(),
      //   to_sample.c_str(), ex.what());
    }
  }

  // Get current joint values
  std::vector<double> joint_group_positions;
  move_group_interface.getCurrentState()->copyJointGroupPositions(
    move_group_interface.getCurrentState()->getRobotModel()->getJointModelGroup("ur_arm"),
    joint_group_positions
  );

  // RCLCPP_ERROR(logger, "joint_group_positions[1]: %f", joint_group_positions[1] * 180 / M_PI);
  // RCLCPP_ERROR(logger, "joint_group_positions[2]: %f", joint_group_positions[2] * 180 / M_PI);
  // RCLCPP_ERROR(logger, "joint_group_positions[3]: %f", joint_group_positions[3] * 180 / M_PI);
  RCLCPP_ERROR(logger, "joint_group_positions[4]: %f", joint_group_positions[4] * 180 / M_PI);
  RCLCPP_ERROR(logger, "wrist_2_yaw: %f", (wrist_2_yaw) * 180 / M_PI);
  RCLCPP_ERROR(logger, "wrist_2_roll: %f", (wrist_2_roll) * 180 / M_PI);
  RCLCPP_ERROR(logger, "wrist_2_pitch: %f", (wrist_2_pitch) * 180 / M_PI);

  RCLCPP_ERROR(logger, "sample_yaw: %f", (sample_yaw) * 180 / M_PI);

  double adj = joint_group_positions[4] + wrist_2_yaw - sample_yaw;
  RCLCPP_ERROR(
    logger, "new joint_goal_degrees[4]: %f",
    (adj) * 180 / M_PI);
  // RCLCPP_ERROR(logger, "joint_group_positions[5]: %f", joint_group_positions[5] * 180 / M_PI);

  joint_goal_degrees[4] = (joint_group_positions[4] + wrist_2_yaw - sample_yaw ) * 180 / M_PI;
  // joint_goal_degrees[5] = joint_group_positions[5] + wrist_2_pitch - sample_pitch;
  doJointMovement(joint_goal_degrees, move_group_interface, logger);

  while (rclcpp::ok()) {
    try {
      transform_world_to_sample = tf_buffer.lookupTransform(
        world_frame,
        to_sample,
        tf2::TimePointZero);

      transform_world_to_grasping_point = tf_buffer.lookupTransform(
        world_frame,
        grasping_point,
        tf2::TimePointZero);

      transform_world_to_pickup_approach_point = tf_buffer.lookupTransform(
        world_frame,
        pickup_approach_point,
        tf2::TimePointZero);

      x_dist_to_pickup_approach =
        transform_world_to_pickup_approach_point.transform.translation.x -
        transform_world_to_grasping_point.transform.translation.x;
      y_dist_to_pickup_approach =
        transform_world_to_pickup_approach_point.transform.translation.y -
        transform_world_to_grasping_point.transform.translation.y;
      z_dist_to_pickup_approach =
        transform_world_to_pickup_approach_point.transform.translation.z -
        (transform_world_to_grasping_point.transform.translation.z);   // 0.1 is an offset to grab low

      RCLCPP_WARN(
        logger,
        "transform_world_to_pickup_approach_point.transform.translation.x: %f   transform_world_to_pickup_approach_point.transform.translation.y: %f  transform_world_to_pickup_approach_point.transform.translation.z: %f",
        transform_world_to_pickup_approach_point.transform.translation.x,
        transform_world_to_pickup_approach_point.transform.translation.y,
        transform_world_to_pickup_approach_point.transform.translation.z);

      RCLCPP_WARN(
        logger,
        "transform_world_to_grasping_point.transform.translation.x: %f   transform_world_to_grasping_point.transform.translation.y: %f  transform_world_to_grasping_point.transform.translation.z: %f",
        transform_world_to_grasping_point.transform.translation.x,
        transform_world_to_grasping_point.transform.translation.y,
        transform_world_to_grasping_point.transform.translation.z);

      x_dist_to_sample = transform_world_to_sample.transform.translation.x -
        transform_world_to_pickup_approach_point.transform.translation.x;
      y_dist_to_sample = transform_world_to_sample.transform.translation.y -
        transform_world_to_pickup_approach_point.transform.translation.y;
      z_dist_to_sample = transform_world_to_sample.transform.translation.z -
        transform_world_to_pickup_approach_point.transform.translation.z;

      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(
        logger, "Could not transform %s to %s: %s", grasping_point.c_str(),
        to_sample.c_str(), ex.what());
    }
  }

  geometry_msgs::msg::PoseStamped current_pose_after_direction =
    move_group_interface.getCurrentPose();

  // Define waypoints for Cartesian path
  std::vector<geometry_msgs::msg::Pose> waypoints;

  geometry_msgs::msg::Pose target_pose = current_pose_after_direction.pose;

  // Move 2 cm down in the z direction
  target_pose.position.z += z_dist_to_pickup_approach;
  target_pose.position.z += -0.025;

  waypoints.push_back(target_pose);
  doCartesianMovement(waypoints, move_group_interface, logger);
  waypoints.clear();

  RCLCPP_WARN(
    logger,
    "x_dist_to_pickup_approach: %f   y_dist_to_pickup_approach: %f  z_dist_to_pickup_approach: %f",
    x_dist_to_pickup_approach,
    y_dist_to_pickup_approach,
    z_dist_to_pickup_approach);

  // Get current pose

  target_pose = move_group_interface.getCurrentPose().pose;
  // Move to pre-pickup lineup.

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

  doCartesianMovement(waypoints, move_group_interface, logger);
  waypoints.clear();
  // ####################### PICK UP

  target_pose = move_group_interface.getCurrentPose().pose;

  target_pose.position.x += x_dist_to_sample;
  target_pose.position.y += y_dist_to_sample;
  waypoints.push_back(target_pose);
  doCartesianMovement(waypoints, move_group_interface, logger);

  std::this_thread::sleep_for(std::chrono::seconds(10));

  // ####################### Move Back


  target_pose = move_group_interface.getCurrentPose().pose;

  target_pose.position.x -= x_dist_to_sample;
  target_pose.position.y -= y_dist_to_sample;
  waypoints.push_back(target_pose);
  doCartesianMovement(waypoints, move_group_interface, logger);

  // Go home
  joint_goal_degrees = {293.24, -77.05, 119.62, -43.57, 199.87, 180.0};

  doJointMovement(joint_goal_degrees, move_group_interface, logger);
  // Shutdown ROS
  planning_scene_interface_->removeCollisionObjects({"table"});

  rclcpp::shutdown();
  return 0;
}
