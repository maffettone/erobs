/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#ifndef PDF_BEAMTIME__PDF_BEAM_ENV_HPP_
#define PDF_BEAMTIME__PDF_BEAM_ENV_HPP_

#include <chrono>
#include <fstream>
#include <iostream>
#include <functional>
#include <memory>
#include <thread>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <pdf_beamtime_interfaces/srv/new_obstacle_msg.hpp>
#include <pdf_beamtime_interfaces/srv/update_obstacle_msg.hpp>

/// @brief Create the obstacle environment and an simple action server for the robot to move.
class PdfBeamEnvironment
{
public:
  /// @brief A structure to hold on to all the necessary fields for an obstacle.
  struct Obstacle
  {
    std::string name;
    std::string type;
    double x, y, z, w, h, d, r;
  };
  using NewObstacleMsg = pdf_beamtime_interfaces::srv::NewObstacleMsg;
  using UpdateObstacleMsg = pdf_beamtime_interfaces::srv::UpdateObstacleMsg;

  explicit PdfBeamEnvironment(const rclcpp::NodeOptions & options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<NewObstacleMsg>::SharedPtr new_obstacle_client_;
  rclcpp::Client<UpdateObstacleMsg>::SharedPtr update_obstacle_client_;

  Obstacle new_obstacle_;
  /// @brief Changes a property of an obstacle.
  void change_obstacle(std::string obstacle_name, std::string property, double value);

  /// @brief Adds a new obstacle. This invokes a service call to new_obstacle_client_.
  void add_new_obstacle(const Obstacle & new_obstacle_);
};

#endif  // PDF_BEAMTIME__PDF_BEAM_ENV_HPP_
