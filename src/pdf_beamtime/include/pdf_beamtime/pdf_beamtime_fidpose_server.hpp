/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include <pdf_beamtime/pdf_beamtime_server.hpp>
#include <pdf_beamtime/tf_utilities.hpp>
#include <pdf_beamtime_interfaces/action/fid_pose_control_msg.hpp>

/// @brief Create the obstacle environment and an simple action server for the robot to move
class PdfBeamtimeFidPoseServer : public PdfBeamtimeServer
{
public:
  using FidPoseControlMsg = pdf_beamtime_interfaces::action::FidPoseControlMsg;

  explicit PdfBeamtimeFidPoseServer(
    const std::string & move_group_name, const rclcpp::NodeOptions & options,
    std::string action_name);

private:
  moveit::core::MoveItErrorCode run_fsm(
    std::shared_ptr<const pdf_beamtime_interfaces::action::FidPoseControlMsg_Goal> goal);

  moveit::core::MoveItErrorCode run_return_fsm(
    std::shared_ptr<const pdf_beamtime_interfaces::action::FidPoseControlMsg_Goal> goal);

  /// @brief Pointer to the inner state machine object
  TFUtilities * tf_utilities_;
  std::vector<double, std::allocator<double>> adjusted_pickup_;
  std::vector<double, std::allocator<double>> adjusted_place_;

  /// @brief holds the current goal to be used by return_sample() function
  std::shared_ptr<const pdf_beamtime_interfaces::action::FidPoseControlMsg_Goal> fidpose_goal;

  /// @brief records pickup approach state
  std::vector<double, std::allocator<double>> pickup_approach_;

  rclcpp_action::Server<FidPoseControlMsg>::SharedPtr fidpose_action_server_;

  void fidpose_handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FidPoseControlMsg>> goal_handle);
  rclcpp_action::GoalResponse fidpose_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FidPoseControlMsg::Goal> goal);
  rclcpp_action::CancelResponse fidpose_handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FidPoseControlMsg>> goal_handle);

  virtual void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<FidPoseControlMsg>> goal_handle);

  moveit::core::MoveItErrorCode return_sample();
  void execute_cleanup();

  bool pickup_pose_saved = false;
  std::vector<double> pre_pickup_approach_joints_;
  std::vector<double> pickup_joints_;
  int sample_id = 0;

  std::unordered_map<int, std::vector<double>> pickup_storage_map_;
  std::unordered_map<int, std::vector<double>> pre_pickup_approach_storage_map_;

};
