/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pose_service/pose_service.hpp>

using std::placeholders::_1;

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

  // Construct the rotation matrix
  auto rotations_from_params = this->get_parameter("camera_rotation_matrix").as_double_array();
  tf2::Matrix3x3 rotation_matrix_(
    rotations_from_params[0], rotations_from_params[1], rotations_from_params[2],
    rotations_from_params[3], rotations_from_params[4], rotations_from_params[5],
    rotations_from_params[6], rotations_from_params[7], rotations_from_params[8]);

  double alpha = this->get_parameter("cam_rotation.alpha").as_double() / 180 * M_PI;
  double gamma = this->get_parameter("cam_rotation.beta").as_double() / 180 * M_PI;
  double beta = this->get_parameter("cam_rotation.gamma").as_double() / 180 * M_PI;

  tf2::Matrix3x3 rotation_gen_(
    cos(beta) * cos(gamma), sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma), cos(
      alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma),
    cos(beta) * sin(gamma), sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma), cos(
      alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma),
    -1 * sin(beta), sin(alpha) * cos(beta), cos(alpha) * cos(beta) );

  // tf2::Matrix3x3 rotation_gen_(0, 0, -1, 1, 0, 0, 0, -1, 0);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      std::cout << rotation_gen_[i][j] << " ";
    }
    std::cout << std::endl;
  }

  // Add the camera to tf server
  rotation_matrix_.getRotation(this->camera_quaternion_);
  // Define the transform
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = this->get_parameter("camera_base").as_string();
  transformStamped.transform.translation.x = this->get_parameter("cam_translation.x").as_double();  //0.516;
  transformStamped.transform.translation.y = this->get_parameter("cam_translation.y").as_double();
  transformStamped.transform.translation.z = this->get_parameter("cam_translation.z").as_double();
  transformStamped.transform.rotation.x = this->camera_quaternion_.x();
  transformStamped.transform.rotation.y = this->camera_quaternion_.y();
  transformStamped.transform.rotation.z = this->camera_quaternion_.z();
  transformStamped.transform.rotation.w = this->camera_quaternion_.w();

  static_broadcaster_.sendTransform(transformStamped);

  camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/rgb/image_raw", 5,
    std::bind(&PoseService::image_raw_callback, this, std::placeholders::_1));

}


void PoseService::image_raw_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg)
{

  // Convert ROS image message to cv::Mat
  cv_bridge::CvImagePtr cv_ptr_rgb =
    cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);

  // Detect the markers from the incoming image
  cv::aruco::detectMarkers(
    cv_ptr_rgb->image, dictionary_,
    markerCorners_, markerIds_, parameters_,
    rejectedCandidates_);

  // Exclude instances where no markers are detected
  try {

    if (!markerIds_.empty()) {

      // rvecs: rotational vector
      // tvecs: translation vector
      std::vector<cv::Vec3d> rvecs, tvecs;

      // Pose estimation happens here
      cv::aruco::estimatePoseSingleMarkers(
        markerCorners_, physical_marker_size_, cameraMatrix_, distCoeffs_, rvecs, tvecs);

      cv::Mat R;
      //  Convert the rvecs to a rotation matrix
      for (size_t i = 0; i < rvecs.size(); ++i) {
        cv::Rodrigues(rvecs[i], R);
      }

      // Access each elements of the R matrix for easy rpy calculations
      double r11 = R.at<double>(0, 0), r21 = R.at<double>(1, 0), r31 = R.at<double>(2, 0),
        r32 = R.at<double>(2, 1), r33 = R.at<double>(2, 2);

      double roll, pitch, yaw;

      // rpy calculation
      roll = std::atan2(r32, r33);
      pitch = std::asin(-1 * r31);
      yaw = std::atan2(r21, r11);

      auto tranlsation = tvecs[0];

      // Construct the raw rpy_xyz of the marker
      std::vector<double> raw_rpyxyz =
      {roll, pitch, yaw, tranlsation[0], tranlsation[1], tranlsation[2]};

      // Median filter gets applied
      median_filter_->update(raw_rpyxyz, median_filtered_rpyxyz);

      // Add to the tf frame here for the sample
      geometry_msgs::msg::TransformStamped transformStamped_tag;

      transformStamped_tag.header.stamp = this->now();
      transformStamped_tag.header.frame_id = this->get_parameter("camera_base").as_string();
      transformStamped_tag.child_frame_id = this->get_parameter("sample_name").as_string();

      transformStamped_tag.transform.translation.x = median_filtered_rpyxyz[3] +
        this->get_parameter("offset_on_marker_x").as_double();
      transformStamped_tag.transform.translation.y = median_filtered_rpyxyz[4];
      transformStamped_tag.transform.translation.z = median_filtered_rpyxyz[5];
      transformStamped_tag.transform.rotation = toQuaternion(roll, pitch, yaw);

      // Add a pre-pickup tf
      geometry_msgs::msg::TransformStamped transformStamped_pre_pickup;
      transformStamped_pre_pickup.header.stamp = this->now();
      transformStamped_pre_pickup.header.frame_id = this->get_parameter("sample_name").as_string();
      transformStamped_pre_pickup.child_frame_id =
        this->get_parameter("pre_pickup_location.name").as_string();

      transformStamped_pre_pickup.transform.translation.x = this->get_parameter(
        "pre_pickup_location.x_adj").as_double();
      transformStamped_pre_pickup.transform.translation.y = this->get_parameter(
        "pre_pickup_location.y_adj").as_double();
      transformStamped_pre_pickup.transform.translation.z = this->get_parameter(
        "pre_pickup_location.z_adj").as_double();
      transformStamped_pre_pickup.transform.rotation = toQuaternion(0, 0, 0);

      static_broadcaster_.sendTransform(transformStamped_pre_pickup);
      static_broadcaster_.sendTransform(transformStamped_tag);

      // RCLCPP_INFO(
      //   this->get_logger(), "Camera RPY XYZ: %f %f %f  \t %f %f %f ",
      //   median_filtered_rpyxyz[0] * (180.0 / 3.141592653589793238463),
      //   median_filtered_rpyxyz[1] * (180.0 / 3.141592653589793238463),
      //   median_filtered_rpyxyz[2] * (180.0 / 3.141592653589793238463),
      //   median_filtered_rpyxyz[3],
      //   median_filtered_rpyxyz[4],
      //   median_filtered_rpyxyz[5]);
    }

    // Inner try-catch
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(this->LOGGER, "Invalid argument in inner try: %s ", e.what() );
    throw;
  }
}

geometry_msgs::msg::Quaternion PoseService::toQuaternion(double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);

  geometry_msgs::msg::Quaternion msg_quat;
  msg_quat.x = quaternion.x();
  msg_quat.y = quaternion.y();
  msg_quat.z = quaternion.z();
  msg_quat.w = quaternion.w();

  return msg_quat;
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
