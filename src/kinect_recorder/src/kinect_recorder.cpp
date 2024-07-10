#include <kinect_recorder/kinect_recorder.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

KinectRecorder::KinectRecorder()
: Node("KinectRecorder")
{
  subscription_rgb_.subscribe(this, "/rgb/image_raw");
  subscription_depth_.subscribe(this, "/depth/image_raw");

  // Synchronize messages from the two topics
  sync_ =
    std::make_shared<message_filters::Synchronizer<sync_policy>>(
    sync_policy(
      5), subscription_rgb_, subscription_depth_);
  sync_->registerCallback(&KinectRecorder::image_raw_callback, this);

}

void KinectRecorder::image_raw_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
{
  // Convert ROS image message to cv::Mat
  cv_bridge::CvImagePtr cv_ptr_rgb =
    cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);

  // Convert ROS image message to cv::Mat
  cv_bridge::CvImagePtr cv_ptr_depth = cv_bridge::toCvCopy(
    depth_msg,
    sensor_msgs::image_encodings::TYPE_32FC1);

  // Convert 32FC1 image to CV_16UC1
  cv_ptr_depth->image.convertTo(cv_ptr_depth->image, CV_16UC1, 65535.0);

  // Save the image as PNG
  std::string filename_rgb = "data/rgb/" + std::to_string(rgb_msg->header.stamp.sec) + ".png";
  cv::imwrite(filename_rgb, cv_ptr_rgb->image);

  // Save the image as PNG
  std::string filename_depth = "data/depth/" + std::to_string(depth_msg->header.stamp.sec) + ".png";
  cv::imwrite(filename_depth, cv_ptr_depth->image);

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto gripper_server_node = std::make_shared<KinectRecorder>();

  rclcpp::spin(gripper_server_node);
  rclcpp::shutdown();

  return 0;
}
