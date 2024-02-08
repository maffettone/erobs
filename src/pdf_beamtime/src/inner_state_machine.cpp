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

State InnerStateMachine::move_robot(
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
      // Change inner state to Moving if the robot is ready to move
      internal_state_enum_ = Internal_State::MOVING;
      RCLCPP_INFO(
        node_->get_logger(), "[%s] Robot's current internal state is %s ",
        external_state_names_[static_cast<int>(external_state_enum_)].c_str(),
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      auto exec_results = mgi.execute(plan);

      if (exec_results == moveit::core::MoveItErrorCode::SUCCESS) {
        // Change inner state to Resting upon successfully moving the robot
        internal_state_enum_ = Internal_State::RESTING;
        return static_cast<State>(static_cast<int>(external_state_enum_) + 1);
      } else {
        // Change inner state to Stop if moving failed or Paused
        internal_state_enum_ = Internal_State::STOP;
        return external_state_enum_;
      }
    } else {
      RCLCPP_ERROR(
        node_->get_logger(), "Inner State %s : Planning failed",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return external_state_enum_;

    }
  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "Robot's current internal state is %s ",
      internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
    if (internal_state_enum_ == Internal_State::STOP) {
      // Handle the rewind here
    }
    return external_state_enum_;
  }

}

State InnerStateMachine::close_gripper()
{
  return static_cast<State>(static_cast<int>(external_state_enum_) + 1);
}

State InnerStateMachine::open_gripper()
{
  return static_cast<State>(static_cast<int>(external_state_enum_) + 1);
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

State InnerStateMachine::abort()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ = Internal_State::ABORT;
    RCLCPP_INFO(
      node_->get_logger(), "[%s] Abort while at internal state %s ",
      external_state_names_[static_cast<int>(external_state_enum_)].c_str(),
      internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
  }
  return State::HOME;

}

State InnerStateMachine::halt()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ = Internal_State::HALT;
    RCLCPP_INFO(
      node_->get_logger(), "[%s] Halt while at internal state %s ",
      external_state_names_[static_cast<int>(external_state_enum_)].c_str(),
      internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
  }
  return State::HOME;
}

void InnerStateMachine::stop()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    RCLCPP_INFO(
      node_->get_logger(), "[%s] Stop while at internal state %s ",
      external_state_names_[static_cast<int>(external_state_enum_)].c_str(),
      internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
    internal_state_enum_ = Internal_State::STOP;
  }

}

State InnerStateMachine::resume(moveit::planning_interface::MoveGroupInterface & mgi)
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ = Internal_State::MOVING;
    RCLCPP_INFO(
      node_->get_logger(), "[%s] Resume while at internal state %s ",
      external_state_names_[static_cast<int>(external_state_enum_)].c_str(),
      internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
  }

  return move_robot(mgi);
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
