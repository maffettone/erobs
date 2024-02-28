/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pdf_beamtime/inner_state_machine.hpp>

InnerStateMachine::InnerStateMachine(
  const rclcpp::Node::SharedPtr node)
: node_(node)
{
  internal_state_enum_ = Internal_State::RESTING;
}

moveit::core::MoveItErrorCode InnerStateMachine::move_robot(
  moveit::planning_interface::MoveGroupInterface & mgi, std::vector<double> joint_goal)
{
  moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;
  joint_goal_ = joint_goal;

  switch (internal_state_enum_) {
    case Internal_State::RESTING: {
        mgi.setJointValueTarget(joint_goal_);
        // Create a plan to that target pose
        auto const [planing_success, plan] = [&mgi] {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(mgi.plan(msg));
            return std::make_pair(ok, msg);
          }();
        if (planing_success) {
          // Change inner state to Moving if the robot is ready to move
          set_internal_state(Internal_State::MOVING);
          auto exec_results = mgi.execute(plan);
          return_error_code = exec_results;
        } else {
          return_error_code = moveit::core::MoveItErrorCode::FAILURE;
        }
      }
      break;

    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Robot's current internal state is %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return_error_code = moveit::core::MoveItErrorCode::FAILURE;
      break;
  }

  return return_error_code;
}

moveit::core::MoveItErrorCode InnerStateMachine::close_gripper()
{
  return moveit::core::MoveItErrorCode::SUCCESS;
}

moveit::core::MoveItErrorCode InnerStateMachine::open_gripper()
{
  return moveit::core::MoveItErrorCode::SUCCESS;
}

void InnerStateMachine::pause(moveit::planning_interface::MoveGroupInterface & mgi)
{
  switch (internal_state_enum_) {
    case Internal_State::RESTING:
      RCLCPP_INFO(
        node_->get_logger(), "Paused while at internal state %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      set_internal_state(Internal_State::PAUSED);
      break;

    case Internal_State::MOVING:
      mgi.stop();
      RCLCPP_INFO(
        node_->get_logger(), "Paused while at internal state %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      set_internal_state(Internal_State::PAUSED);
      break;
    default:
      break;
  }
}

void InnerStateMachine::abort(moveit::planning_interface::MoveGroupInterface & mgi)
{
  mgi.stop();
  RCLCPP_INFO(
    node_->get_logger(), "Stopped while at internal state %s ",
    internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
  set_internal_state(Internal_State::ABORT);
}

void InnerStateMachine::halt(moveit::planning_interface::MoveGroupInterface & mgi)
{
  mgi.stop();
  RCLCPP_INFO(
    node_->get_logger(), "Halted while at internal state %s ",
    internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
  set_internal_state(Internal_State::HALT);
}

void InnerStateMachine::rewind()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    set_internal_state(Internal_State::RESTING);
  }
}

void InnerStateMachine::set_internal_state(Internal_State state)
{
  RCLCPP_INFO(
    node_->get_logger(), "Internal state changed from %s to %s ",
    internal_state_names[static_cast<int>(internal_state_enum_)].c_str(),
    internal_state_names[static_cast<int>(state)].c_str());
  internal_state_enum_ = state;
}

Internal_State InnerStateMachine::get_internal_state()
{
  return internal_state_enum_;
}
