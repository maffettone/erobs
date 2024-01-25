#pragma once

#include <rclcpp/node.hpp>
#include <pdf_beamtime/internal_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <pdf_beamtime/state_enum.hpp>

class InternalState;

class FiniteStateMachine
{
  friend class InternalState;

private:
  InternalState * state_;
  rclcpp::Node::SharedPtr node_;

  State external_state_enum_;

public:
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  std::vector<double> joint_goal_;

  FiniteStateMachine(const rclcpp::Node::SharedPtr node);
  FiniteStateMachine(const rclcpp::Node::SharedPtr node, State external_state_enum);
  void move_robot();
  void robot_moved_successfully();
  void pause();
  void resume();
  void rewind();
  void abort();
  void halt();
  void stop();
  void transition_to_next_external_state();
  void set_joint_goal(std::vector<double> joint_goal);
  ~FiniteStateMachine();
};
