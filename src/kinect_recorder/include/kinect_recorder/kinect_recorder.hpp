/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <iostream>
#include <thread>
#include <memory>
#include <string>
#include <map>

#include <rclcpp/rclcpp.hpp>

#include <cv_bridge/cv_bridge.h>
// #include <op/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv4/opencv2/imgcodecs.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class KinectRecorder : public rclcpp::Node
{
private:
  message_filters::Subscriber<sensor_msgs::msg::Image> subscription_rgb_;
  message_filters::Subscriber<sensor_msgs::msg::Image> subscription_depth_;

  void image_raw_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg);

  using sync_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image,
      sensor_msgs::msg::Image>;

  // Define synchronizer
  std::shared_ptr<message_filters::Synchronizer<sync_policy>> sync_;

public:
  KinectRecorder();
};
