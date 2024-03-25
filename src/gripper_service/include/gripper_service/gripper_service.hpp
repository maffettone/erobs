/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"

#include "pdf_beamtime_interfaces/srv/gripper_control_msg.hpp"

#include <robotiq_driver/robotiq_gripper_interface.hpp>

class GripperService : public rclcpp::Node
{

private:
  /// @brief your serial port goes here
  const char * kComPort = "/tmp/ttyUR";
  const int kSlaveID = 0x09;

  // rclcpp::Node::SharedPtr node_;
  RobotiqGripperInterface gripper_;

/// @brief Gripper commands as enums for ease of use
  enum class Gripper_Command {OPEN, CLOSE, PARTIAL, ACTIVE, DEACTIVE};

  std::map<std::string, Gripper_Command> gripper_command_map_ = {
    {"ACTIVE", Gripper_Command::ACTIVE},
    {"DEACTIVE", Gripper_Command::DEACTIVE},
    {"OPEN", Gripper_Command::OPEN},
    {"CLOSE", Gripper_Command::CLOSE},
    {"PARTIAL", Gripper_Command::PARTIAL}
  };

  /// @brief callback function for the gripper service
  void gripper_controller(
    const std::shared_ptr<pdf_beamtime_interfaces::srv::GripperControlMsg::Request> request,
    std::shared_ptr<pdf_beamtime_interfaces::srv::GripperControlMsg::Response> response);

public:
  explicit GripperService();
};
