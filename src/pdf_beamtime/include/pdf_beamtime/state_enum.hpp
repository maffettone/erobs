#pragma once

/// @brief States of the robot transitions
enum class State {HOME, PICKUP_APPROACH, PICKUP, GRASP_SUCCESS, PICKUP_RETREAT,
  PLACE_APPROACH, PLACE, RELEASE_SUCCESS, PLACE_RETREAT};

enum class Internal_State {RESTING, MOVING, PAUSED, ABORT, HALT, STOP};
