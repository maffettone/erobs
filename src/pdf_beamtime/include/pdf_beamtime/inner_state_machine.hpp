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

  // moveit::planning_interface::MoveGroupInterface move_group_interface_;

  std::vector<std::string> external_state_names_ =
  {"HOME", "PICKUP_APPROACH", "PICKUP", "GRASP_SUCCESS", "PICKUP_RETREAT",
    "PLACE_APPROACH", "PLACE", "RELEASE_SUCCESS", "PLACE_RETREAT"};

  std::vector<std::string> internal_state_names =
  {"RESTING", "MOVING", "PAUSED", "ABORT", "HALT", "STOP"};

public:
  InnerStateMachine(const rclcpp::Node::SharedPtr node, State external_state_enum);

  /// @brief move the robot to the passed joint angles
  /// @param mgi move_group_interface_
  /// @return next outer state
  State move_robot(moveit::planning_interface::MoveGroupInterface & mgi);

  /// @brief state change if paused command was issues
  void pause();
  State resume(moveit::planning_interface::MoveGroupInterface & mgi);
  /// @brief TODO: rewind the state back to RESTING
  void rewind();
  State abort();
  State halt();
  void stop();

  State open_gripper();

  State close_gripper();

  /// @brief sets the joint target in a variable
  /// @param joint_goal joint target
  void set_joint_goal(std::vector<double> joint_goal);

  /// @brief setters and getter for the active state
  void set_active_true();
  void set_active_false();
  bool is_active();

  ~InnerStateMachine();
};
