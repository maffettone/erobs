/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"

#include "pdf_beamtime_interfaces/srv/gripper_control_msg.hpp"

// #include "robotiq_driver/robotiq_gripper_interface.hpp"
#include <robotiq_driver/robotiq_gripper_interface.hpp>

class GripperService
{

private:
  const char * kComPort = "/tmp/ttyUR";
  const int kSlaveID = 0x09;

  rclcpp::Node::SharedPtr node_;
  RobotiqGripperInterface gripper_;

/// @brief Gripper states
  enum class Gripper_State {OPEN, CLOSE, PARTIAL, MOVING, ACTIVE, DEACTIVE};

  std::map<std::string, Gripper_State> gripper_command_map_ = {
    {"ACTIVE", Gripper_State::ACTIVE},
    {"DEACTIVE", Gripper_State::DEACTIVE},
    {"OPEN", Gripper_State::OPEN},
    {"CLOSE", Gripper_State::CLOSE},
    {"PARTIAL", Gripper_State::PARTIAL}
  };

  void gripper_controller(
    const std::shared_ptr<pdf_beamtime_interfaces::srv::GripperControlMsg::Request> request,
    std::shared_ptr<pdf_beamtime_interfaces::srv::GripperControlMsg::Response> response);

public:
  explicit GripperService();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

};
