/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <utility>

class TFUtilities
{
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger tf_util_logger_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client_;

  std::string world_frame, sample_frame, grasping_point_on_gripper_frame, wrist_2_frame,
    pre_pickup_approach_point_frame;

public:
  explicit TFUtilities(const rclcpp::Node::SharedPtr node);

  /// @brief converts degrees to radians
  double degreesToRadians(double degrees);

  std::pair<double, double> get_wrist_elbow_alignment(
    moveit::planning_interface::MoveGroupInterface & mgi);

  std::vector<geometry_msgs::msg::Pose> get_pickup_action_waypoints(
    moveit::planning_interface::MoveGroupInterface & mgi);
};
