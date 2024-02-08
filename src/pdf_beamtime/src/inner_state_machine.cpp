#include <pdf_beamtime/inner_state_machine.hpp>

InnerStateMachine::InnerStateMachine(
  const rclcpp::Node::SharedPtr node, State external_state_enum)
: node_(node), external_state_enum_(external_state_enum)
{
  internal_state_enum_ = Internal_State::RESTING;

}

InnerStateMachine::~InnerStateMachine()
{
}

void InnerStateMachine::set_joint_goal(std::vector<double> joint_goal)
{
  joint_goal_ = joint_goal;
}

moveit::core::MoveItErrorCode InnerStateMachine::move_robot(
  moveit::planning_interface::MoveGroupInterface & mgi)
{

  if (internal_state_enum_ == Internal_State::RESTING) {
    RCLCPP_INFO(
      node_->get_logger(), "[%s] Robot's current internal state is %s ",
      external_state_names_[static_cast<int>(external_state_enum_)].c_str(),
      internal_state_names[static_cast<int>(internal_state_enum_)].c_str());

    mgi.setJointValueTarget(joint_goal_);

    // Create a plan to that target pose
    auto const [success, plan] = [&mgi] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(mgi.plan(msg));
        return std::make_pair(ok, msg);
      }();

    if (success) {
      internal_state_enum_ = Internal_State::MOVING;
      auto exec_results = mgi.execute(plan);

      if (exec_results == moveit::core::MoveItErrorCode::SUCCESS) {
        internal_state_enum_ = Internal_State::RESTING;
        return exec_results;
      } else {
        internal_state_enum_ = Internal_State::STOP;
        return exec_results;
      }
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), "Inner State %s : Planning failed",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return moveit::core::MoveItErrorCode::FAILURE;

    }
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "Robot's current internal state is %s ",
      internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
    return moveit::core::MoveItErrorCode::FAILURE;
  }

}

void InnerStateMachine::pause()
{
  if (internal_state_enum_ == Internal_State::RESTING) {
    RCLCPP_INFO(
      node_->get_logger(), "[%s] Paused while at internal state %s ",
      external_state_names_[static_cast<int>(external_state_enum_)].c_str(),
      internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
    internal_state_enum_ = Internal_State::PAUSED;
  }

  if (internal_state_enum_ == Internal_State::MOVING) {
    RCLCPP_INFO(
      node_->get_logger(), "[%s] Paused while at internal state %s ",
      external_state_names_[static_cast<int>(external_state_enum_)].c_str(),
      internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
    internal_state_enum_ = Internal_State::PAUSED;
  }
}

void InnerStateMachine::abort()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ = Internal_State::ABORT;
  }

}

void InnerStateMachine::halt()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ = Internal_State::HALT;
  }

}

void InnerStateMachine::stop()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ = Internal_State::STOP;
  }

}

void InnerStateMachine::resume()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ = Internal_State::MOVING;
  }

}

void InnerStateMachine::rewind()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ = Internal_State::RESTING;
  }

}

void InnerStateMachine::set_active_true() {state_active_ = true;}

void InnerStateMachine::set_active_false() {state_active_ = false;}

bool InnerStateMachine::is_active() {return state_active_;}
