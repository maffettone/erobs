#pragma once

#include <rclcpp/node.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <pdf_beamtime/state_enum.hpp>
#include <future>

class InnerStateMachine
{

private:
  rclcpp::Node::SharedPtr node_;
  State external_state_enum_;
  Internal_State internal_state_enum_;
  /// @brief Holds the passed joint target
  std::vector<double> joint_goal_;

  std::future<moveit::core::MoveItErrorCode> move_future_;
  moveit::core::MoveItErrorCode execute_future_;
  /// @brief true if this outer state machine works on this particular state, false otherwise
  bool state_active_ = false;

  std::vector<std::string> external_state_names_ =
  {"HOME", "PICKUP_APPROACH", "PICKUP", "GRASP_SUCCESS", "GRASP_FAILURE", "PICKUP_RETREAT",
    "PLACE_APPROACH", "PLACE", "RELEASE_SUCCESS", "RELEASE_FAILURE", "PLACE_RETREAT"};

  std::vector<std::string> internal_state_names =
  {"RESTING", "MOVING", "PAUSED", "ABORT", "HALT", "STOP"};

public:
  InnerStateMachine(const rclcpp::Node::SharedPtr node, State external_state_enum);

  /// @brief move the robot to the passed joint angles
  /// @param mgi move_group_interface_
  /// @return next outer state
  moveit::core::MoveItErrorCode move_robot(
    moveit::planning_interface::MoveGroupInterface & mgi,
    std::vector<double> joint_goal);

  /// @brief state change if paused command was issues
  void pause(moveit::planning_interface::MoveGroupInterface & mgi);
  // State resume(moveit::planning_interface::MoveGroupInterface & mgi);
  /// @brief TODO: rewind the state back to RESTING
  void rewind();
  void abort();
  void halt();
  void stop();

  void set_internal_state(Internal_State state);
  Internal_State get_internal_state();

  void set_external_state(State state);

  moveit::core::MoveItErrorCode open_gripper();
  moveit::core::MoveItErrorCode close_gripper();

  ~InnerStateMachine();
};
