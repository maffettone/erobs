/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <iostream>
#include <thread>
#include <memory>
#include <string>
#include <map>
#include <cmath>
#include <fstream>

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <filters/median.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class PoseService : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;

  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  rclcpp::Logger LOGGER;

  /// @brief Callback function to
  /// @param rgb_msg
  void image_raw_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg);

  // // In OpenCV, the distortion coefficients are usually represented as a 1x8 matrix:
  // // distCoeffs_= [k1, k2, p1, p2, k3, k4, k5, k6] where
  // // k1, k2, k3, k3, k4, k5, k6 = radial distortion coefficients
  // // p1, p2 = tangential distortion coefficients
  // // For Azure kinect, they can be found when running the ROS2 camera node and explained in the following:
  // // https://microsoft.github.io/Azure-Kinect-Sensor-SDK/master/structk4a__calibration__intrinsic__parameters__t_1_1__param.html

  // cv::Mat cameraMatrix_ =
  //   (cv::Mat_<double>(3, 3) << 974.724, 0.0, 1024.82, 0.0, 974.456, 773.291, 0, 0, 1.0, 2.87826,
  //   2.23121, 0.364764 );
  cv::Mat cameraMatrix_;
  // cv::Mat distCoeffs_ =
  //   (cv::Mat_<double>(8, 1) << 0.602113, -2.92716, 0.000402061, -0.000392198, 1.63343, 0.468872,
  //   -2.72304, 1.54828);
  cv::Mat distCoeffs_;

  tf2::Quaternion camera_quaternion_;

  // Parameters for Aruco maker detection
  std::vector<int> markerIds_;
  std::vector<std::vector<cv::Point2f>> markerCorners_, rejectedCandidates_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;  // Params for ArUco detetcion
  cv::Ptr<cv::aruco::Dictionary> dictionary_;

  // This is the actual length/width of the printed tag
  double physical_marker_size_;   // height/width in meters

  // Median filter related vars
  std::shared_ptr<filters::MultiChannelFilterBase<double>> median_filter_ =
    std::make_shared<filters::MultiChannelMedianFilter<double>>();
  int moving_window_median_ = 10;
  std::vector<double> median_filtered_rpyxyz;

  /// @brief converts a rpy to a quaternion
  geometry_msgs::msg::Quaternion toQuaternion(double roll, double pitch, double yaw);

public:
  PoseService(const rclcpp::NodeOptions options);
};
