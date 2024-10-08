/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <cmath>
#include <fstream>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <filters/median.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class ArucoPose : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;

  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::thread flange_tracker_thread_;
  double flange_x = 0.0, flange_y = 0.0, flange_z = 0.0, flange_qx = 0.0, flange_qy = 0.0,
    flange_qz = 0.0, flange_qw = 0.0;

  rclcpp::Logger LOGGER;

  /// @brief Callback function to
  /// @param rgb_msg
  void image_raw_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg);

  cv::Mat cameraMatrix_;
  cv::Mat distCoeffs_;

  tf2::Quaternion camera_quaternion_;

  // Parameters for Aruco maker detection
  std::vector<int> markerIds_;
  std::vector<std::vector<cv::Point2f>> markerCorners_, rejectedCandidates_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;  // Params for ArUco detetcion
  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  // This is the actual length/width of the printed tag
  double physical_marker_size_;   // height/width in meters

  // Median filters related vars
  std::unordered_map<int,
    std::shared_ptr<filters::MultiChannelFilterBase<double>>> median_filters_map_;
  std::unordered_map<int, std::vector<double>> median_filtered_rpyxyz_map_;

  /// @brief converts a rpy to a quaternion
  geometry_msgs::msg::Quaternion toQuaternion(double roll, double pitch, double yaw);

  std::vector<std::string> double_params_ = {
    "intrinsics.fx", "intrinsics.fy", "intrinsics.cx", "intrinsics.cy",
    "dist_coeffs.k1", "dist_coeffs.k2", "dist_coeffs.k3", "dist_coeffs.k4",
    "dist_coeffs.k5", "dist_coeffs.k6", "dist_coeffs.p1", "dist_coeffs.p2",
    "cam_translation.x", "cam_translation.y", "cam_translation.z",
    "cam_rotation.cam_alpha", "cam_rotation.cam_beta", "cam_rotation.cam_gamma",
    "pre_pickup_location.x_adj", "pre_pickup_location.y_adj", "pre_pickup_location.z_adj",
    "offset_on_marker_x", "offset_on_marker_y", "physical_marker_size"
  };

  std::vector<std::string> string_params_ = {
    "camera_tf_frame", "sample_name", "pre_pickup_location.name", "fiducial_marker_family",
    "image_topic"
  };

  // Map string values to corresponding ArUco dictionary enums
  std::map<std::string, cv::aruco::PREDEFINED_DICTIONARY_NAME> dictionary_map_ = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250", cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000", cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100", cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250", cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000", cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100", cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250", cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000", cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50", cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100", cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250", cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000", cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL", cv::aruco::DICT_ARUCO_ORIGINAL},
    {"DICT_APRILTAG_16h5", cv::aruco::DICT_APRILTAG_16h5},
    {"DICT_APRILTAG_25h9", cv::aruco::DICT_APRILTAG_25h9},
    {"DICT_APRILTAG_36h10", cv::aruco::DICT_APRILTAG_36h10},
    {"DICT_APRILTAG_36h11", cv::aruco::DICT_APRILTAG_36h11}
  };

public:
  ArucoPose();
};
