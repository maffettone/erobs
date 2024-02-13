#pragma once

/// @brief States of the robot transitions
enum class State {HOME, PICKUP_APPROACH, PICKUP, GRASP_SUCCESS, GRASP_FAILURE, PICKUP_RETREAT,
  PLACE_APPROACH, PLACE, RELEASE_SUCCESS, RELEASE_FAILURE, PLACE_RETREAT};

/// @brief Internal states of the state machine
enum class Internal_State {RESTING, MOVING, PAUSED, ABORT, HALT, STOP};
