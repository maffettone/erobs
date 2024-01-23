#include <pdf_beamtime/internal_state.hpp>

InternalState::InternalState(/* args */)
{

}

InternalState::~InternalState()
{

}

void InternalState::setState(FiniteStateMachine & fsm, InternalState * state)
{
  // InternalState * aux = fsm.state_;
  fsm.state_ = state;
  delete state;

}


void InternalState::robot_moving(FiniteStateMachine & fsm)
{

}

void InternalState::robot_moved_successfully(FiniteStateMachine & fsm)
{

}

void InternalState::pause(FiniteStateMachine & fsm)
{

}

void InternalState::resume(FiniteStateMachine & fsm)
{

}

void InternalState::rewind(FiniteStateMachine & fsm)
{

}

void InternalState::abort(FiniteStateMachine & fsm)
{

}

void InternalState::halt(FiniteStateMachine & fsm)
{

}

void InternalState::stop(FiniteStateMachine & fsm)
{

}


// ********** Class Resting ************

void Resting::robot_moving(FiniteStateMachine & fsm)
{
  setState(fsm, new Moving());
}

void Resting::pause(FiniteStateMachine & fsm)
{
  setState(fsm, new Paused());
}

Resting::~Resting() {}


// ********** Class Moving ************

Moving::~Moving() {}

void Moving::robot_moved_successfully(FiniteStateMachine & fsm)
{


}

void Moving::pause(FiniteStateMachine & fsm)
{
  setState(fsm, new Paused());
}

// ********** Class Paused ************

void Paused::resume(FiniteStateMachine & fsm)
{
  setState(fsm, new Moving());
}

void Paused::rewind(FiniteStateMachine & fsm)
{
  setState(fsm, new Resting());
}

void Paused::abort(FiniteStateMachine & fsm)
{
  setState(fsm, new Abort());
}

void Paused::halt(FiniteStateMachine & fsm)
{
  setState(fsm, new Halt());
}

void Paused::stop(FiniteStateMachine & fsm)
{
  setState(fsm, new Stop());
}

Paused::~Paused() {}

// ********** Class Abort ************

Abort::~Abort() {}

// ********** Class Halt ************

Halt::~Halt() {}

// ********** Class Stop ************

Stop::~Stop() {}
