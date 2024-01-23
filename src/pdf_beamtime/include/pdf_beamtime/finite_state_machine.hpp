#pragma once

#include <rclcpp/node.hpp>
#include <pdf_beamtime/internal_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class InternalState;

class FiniteStateMachine
{
  friend class InternalState;

private:
  InternalState * state_;
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;

public:
  FiniteStateMachine(const rclcpp::Node::SharedPtr node);
  void robot_moving();
  void robot_moved_successfully();
  void pause();
  void resume();
  void rewind();
  void abort();
  void halt();
  void stop();
  void transition_to_next_external_state();
  ~FiniteStateMachine();
};
