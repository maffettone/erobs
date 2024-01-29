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

std::future<moveit::core::MoveItErrorCode> InnerStateMachine::move_robot(
  moveit::planning_interface::MoveGroupInterface & mgi)
{
  state_active_ = true;
  // Check if the current state is Resting before moving. if not throw an error
  if (internal_state_enum_ == Internal_State::RESTING) {
    RCLCPP_INFO(
      node_->get_logger(), "[%s] Robot's current internal state is %s ",
      external_state_names_[static_cast<int>(external_state_enum_)].c_str(),
      internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
    set_joint_value_target(mgi);

  } else {
    RCLCPP_ERROR(
      node_->get_logger(), "Robot's current internal state is %s ",
      internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
    // return std::make_pair(execute_future_, false);
  }

  return std::move(execute_future_);

  // execute_future_.wait()

}

void InnerStateMachine::set_joint_goal(std::vector<double> joint_goal)
{
  joint_goal_ = joint_goal;
}

// std::pair<std::future<bool>, bool> InnerStateMachine::set_joint_value_target(
bool InnerStateMachine::set_joint_value_target(
  moveit::planning_interface::MoveGroupInterface & mgi)
{
  // Create a plan to that target pose
  auto const [success, plan] = [&mgi] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(mgi.plan(msg));
      return std::make_pair(ok, msg);
    }();
  if (success) {
    //execute(plan) is a blocking call
    execute_future_ = std::async(
      std::launch::async, [&mgi, &plan] {
        // Shoooot, I just realized that move_group_interface already has an async_execution(). Below code will change
        auto execution_results = mgi.execute(plan);
        return execution_results;
      });
    internal_state_enum_ = Internal_State::MOVING;
    return true;
    // return std::make_pair(execute_future_, success);
  } else {
    return false;
  }

}

void InnerStateMachine::pause()
{
  if (internal_state_enum_ == Internal_State::RESTING) {
    internal_state_enum_ == Internal_State::PAUSED;
  }

  if (internal_state_enum_ == Internal_State::MOVING) {
    internal_state_enum_ == Internal_State::PAUSED;
  }
}

void InnerStateMachine::abort()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ == Internal_State::ABORT;
  }

}

void InnerStateMachine::halt()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ == Internal_State::HALT;
  }

}

void InnerStateMachine::stop()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ == Internal_State::STOP;
  }

}

void InnerStateMachine::resume()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ == Internal_State::MOVING;
  }

}

void InnerStateMachine::rewind()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    internal_state_enum_ == Internal_State::RESTING;
  }

}

void InnerStateMachine::set_active_true() {state_active_ = true;}

void InnerStateMachine::set_active_false() {state_active_ = false;}

bool InnerStateMachine::is_active() {return state_active_;}
