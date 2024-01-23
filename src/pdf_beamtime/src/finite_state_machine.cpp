#include <pdf_beamtime/finite_state_machine.hpp>

FiniteStateMachine::FiniteStateMachine(
  const rclcpp::Node::SharedPtr node)
: node_(node), move_group_interface_(node, "a")
{
  state_ = static_cast<InternalState *>(new Resting());
}

FiniteStateMachine::~FiniteStateMachine()
{
  delete state_;
}

void FiniteStateMachine::robot_moving()
{
  state_->robot_moving(*this);
}

void FiniteStateMachine::robot_moved_successfully()
{
  state_->robot_moved_successfully(*this);
}
void FiniteStateMachine::pause()
{
  state_->pause(*this);
}

void FiniteStateMachine::resume()
{
  state_->resume(*this);
}

void FiniteStateMachine::rewind()
{
  state_->rewind(*this);
}

void FiniteStateMachine::abort()
{
  state_->abort(*this);
}

void FiniteStateMachine::halt()
{
  state_->halt(*this);
}

void FiniteStateMachine::stop()
{
  state_->stop(*this);
}

void FiniteStateMachine::transition_to_next_external_state()
{

}
