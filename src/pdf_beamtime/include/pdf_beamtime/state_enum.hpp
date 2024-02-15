/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

/// @brief States of the robot transitions
enum class State {HOME, PICKUP_APPROACH, PICKUP, GRASP_SUCCESS, GRASP_FAILURE, PICKUP_RETREAT,
  PLACE_APPROACH, PLACE, RELEASE_SUCCESS, RELEASE_FAILURE, PLACE_RETREAT, RETRY_PICKUP};

/// @brief Internal states of the state machine
enum class Internal_State {RESTING, MOVING, PAUSED, ABORT, HALT, STOP};
