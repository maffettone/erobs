/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/node.hpp>

class TFUtilities
{
private:
  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

public:
  explicit TFUtilities(const rclcpp::Node::SharedPtr node);

  /// @brief converts degrees to radians
  double degreesToRadians(double degrees);
  double get_elbow_alignment();
};
