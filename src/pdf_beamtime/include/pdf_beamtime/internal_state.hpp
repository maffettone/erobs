#pragma once

#include <pdf_beamtime/finite_state_machine.hpp>

class FiniteStateMachine;

class InternalState
{
private:
  /* data */

public:
  InternalState(/* args */);
  virtual ~InternalState();
  virtual void move_robot(FiniteStateMachine & fsm);
  virtual void robot_moved_successfully(FiniteStateMachine & fsm);
  virtual void pause(FiniteStateMachine & fsm);
  virtual void resume(FiniteStateMachine & fsm);
  virtual void rewind(FiniteStateMachine & fsm);
  virtual void abort(FiniteStateMachine & fsm);
  virtual void halt(FiniteStateMachine & fsm);
  virtual void stop(FiniteStateMachine & fsm);

protected:
  void setState(FiniteStateMachine & fsm, InternalState * state);
};


// ********** Class Resting ************

class Resting : public InternalState
{
public:
  void move_robot(FiniteStateMachine & fsm) override;
  void pause(FiniteStateMachine & fsm) override;
  ~Resting();

};

// ********** Class Moving ************

class Moving : public InternalState
{
  void pause(FiniteStateMachine & fsm) override;
  void robot_moved_successfully(FiniteStateMachine & fsm) override;
  ~Moving();
};

// ********** Class Paused ************

class Paused : public InternalState
{
  void resume(FiniteStateMachine & fsm) override;
  void rewind(FiniteStateMachine & fsm) override;
  void abort(FiniteStateMachine & fsm) override;
  void halt(FiniteStateMachine & fsm) override;
  void stop(FiniteStateMachine & fsm) override;
  ~Paused();

};

class Abort : public InternalState
{
  ~Abort();

};

class Halt : public InternalState
{
  ~Halt();

};

class Stop : public InternalState
{
  ~Stop();

};
