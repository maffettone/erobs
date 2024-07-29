/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pdf_beamtime/tf_utilities.hpp>


TFUtilities::TFUtilities(
  const rclcpp::Node::SharedPtr node)
: node_(node)
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


double TFUtilities::degreesToRadians(double degrees)
{
  return degrees * M_PI / 180.0;
}

double TFUtilities::get_elbow_alignment()
{
  return 0.0;
}
