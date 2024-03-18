/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <iostream>
#include <thread>
#include "rclcpp/rclcpp.hpp"

#include <robotiq_driver/srv/gripper_control_msg.hpp>
// #include "pdf_beamtime_interface/srv/gripper_cmd.hpp"

#include <robotiq_driver/robotiq_gripper_interface.hpp>

class GripperService
{

private:
  const char * kComPort = "/tmp/ttyUR";
  const int kSlaveID = 0x09;
  RobotiqGripperInterface gripper_;

public:
  GripperService();
};
