/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pose_service/pose_service.hpp>

using namespace std::placeholders;

PoseService::PoseService(const rclcpp::NodeOptions options)
: Node("pose_service", options),
  static_broadcaster_(this), tf_broadcaster_(this),
  LOGGER(this->get_logger())
{
  // Construct the camera K matrix and the distortion matrix
  cameraMatrix_ =
    (cv::Mat_<double>(3, 3) << this->get_parameter("intrinsics.fx").as_double(), 0.0,
    this->get_parameter("intrinsics.cx").as_double(), 0.0,
    this->get_parameter("intrinsics.fy").as_double(),
    this->get_parameter("intrinsics.cy").as_double(), 0, 0, 1.0);

  distCoeffs_ =
    (cv::Mat_<double>(8, 1) << this->get_parameter("dist_coeffs.k1").as_double(),
    this->get_parameter("dist_coeffs.k2").as_double(),
    this->get_parameter("dist_coeffs.p1").as_double(),
    this->get_parameter("dist_coeffs.p2").as_double(),
    this->get_parameter("dist_coeffs.k3").as_double(),
    this->get_parameter("dist_coeffs.k4").as_double(),
    this->get_parameter("dist_coeffs.k5").as_double(),
    this->get_parameter("dist_coeffs.k6").as_double());

  physical_marker_size_ = this->get_parameter("physical_marker_size").as_double();
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Setting allow_undeclared_parameters(true) makes you not re-declare params
  const rclcpp::NodeOptions & options = (
    rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true)
  );
  auto pose_service_node = std::make_shared<PoseService>(options);

  rclcpp::spin(pose_service_node);
  rclcpp::shutdown();

  return 0;
}
