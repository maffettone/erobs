/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <pdf_beamtime/pdf_beamtime_server.hpp>
#include <pdf_beamtime/tf_utilities.hpp>

/// @brief Create the obstacle environment and an simple action server for the robot to move
class PdfBeamtimeFidPoseServer : public PdfBeamtimeServer
{
public:
  explicit PdfBeamtimeFidPoseServer(
    const std::string & move_group_name, const rclcpp::NodeOptions & options,
    std::string action_name);

protected:
  moveit::core::MoveItErrorCode run_fsm(
    std::shared_ptr<const pdf_beamtime_interfaces::action::PickPlaceControlMsg_Goal> goal);

private:
  /// @brief Pointer to the inner state machine object
  TFUtilities * tf_utilities_;
  std::vector<double, std::allocator<double>> adjusted_pickup_;
};
