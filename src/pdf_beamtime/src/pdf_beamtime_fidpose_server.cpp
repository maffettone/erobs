/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pdf_beamtime/pdf_beamtime_fidpose_server.hpp>

using namespace std::placeholders;

PdfBeamtimeFidPoseServer::PdfBeamtimeFidPoseServer(
  const std::string & move_group_name = "ur_manipulator",
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
  std::string action_name = "pdf_beamtime_fidpose_action_server")
: PdfBeamtimeServer(move_group_name, options, "pdf_beamtime_action_server")
{
  tf_utilities_ = new TFUtilities(node_);

  // Create the action server
  fidpose_action_server_ = rclcpp_action::create_server<FidPoseControlMsg>(
    this->node_,
    action_name,
    std::bind(&PdfBeamtimeFidPoseServer::fidpose_handle_goal, this, _1, _2),
    std::bind(&PdfBeamtimeFidPoseServer::fidpose_handle_cancel, this, _1),
    std::bind(&PdfBeamtimeFidPoseServer::fidpose_handle_accepted, this, _1));
}

rclcpp_action::GoalResponse PdfBeamtimeFidPoseServer::fidpose_handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FidPoseControlMsg::Goal> goal)
{
  RCLCPP_INFO(node_->get_logger(), "Goal handle inside");
  (void)uuid;
  if (goal->sample_return && !pickup_pose_saved) {
    RCLCPP_INFO(node_->get_logger(), "Goal reject");
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accept and execute");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
}

void PdfBeamtimeFidPoseServer::fidpose_handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FidPoseControlMsg>> goal_handle)
{
  using namespace std::placeholders;

  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&PdfBeamtimeFidPoseServer::execute, this, _1), goal_handle}.detach();
}

// Receiving the cancel request.
rclcpp_action::CancelResponse PdfBeamtimeFidPoseServer::fidpose_handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FidPoseControlMsg>> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PdfBeamtimeFidPoseServer::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<FidPoseControlMsg>> goal_handle)
{
  fidpose_goal = goal_handle->get_goal();
  //  Variables for feedback and results
  auto feedback = std::make_shared<FidPoseControlMsg::Feedback>();
  auto results = std::make_shared<FidPoseControlMsg::Result>();
  moveit::core::MoveItErrorCode fsm_results;
  progress_ = 0.0;
  goal_home_ = node_->get_parameter("home_angles").as_double_array();
  pickup_approach_ = node_->get_parameter("pickup_approach_angles").as_double_array();
  feedback->status = get_action_completion_percentage();

  RCLCPP_INFO(
    node_->get_logger(), "Current state is %s.",
    external_state_names_[static_cast<int>(current_state_)].c_str());

  // Reset inner_state_machine at new goal
  inner_state_machine_->set_internal_state(Internal_State::RESTING);

  // Keep executing the states until the a goal is completed or cancelled
  while (get_action_completion_percentage() < 99.99999) {
    if (inner_state_machine_->get_internal_state() == Internal_State::PAUSED) {
      // Upon triggering pause_, the execution while loop switches to 1HZ.
      // The external and internal states are handled separately by the FSM
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      if (inner_state_machine_->get_internal_state() == Internal_State::CLEANUP) {
        // Abort the goal and return at cleanup
        results->success = false;
        goal_handle->abort(results);
        RCLCPP_ERROR(node_->get_logger(), "Goal aborted !");
        return;
      }

      // Decide if it'a a return ot a place
      if (fidpose_goal->sample_return) {
        // Abort goal is asked to return to an unsaved storage location
        if (pickup_storage_map_.find(fidpose_goal->sample_id) == pickup_storage_map_.end()) {
          RCLCPP_ERROR(node_->get_logger(), "Unrecorded return location at storage");
          results->success = false;
          goal_handle->abort(results);
          return;
        }
        fsm_results = run_return_fsm(fidpose_goal);
      } else {
        fsm_results = run_fsm(fidpose_goal);
      }

      if (!fsm_results && inner_state_machine_->get_internal_state() != Internal_State::PAUSED) {
        // Abort the execution if move_group_ fails except when paused
        results->success = false;
        goal_handle->abort(results);
        RCLCPP_ERROR(node_->get_logger(), "Goal aborted !");
        return;
      }

      if (goal_handle->is_canceling()) {
        // Reset the fsm if goal is cancelled by the action client
        results->success = false;
        reset_fsm();
        goal_handle->canceled(results);
        RCLCPP_WARN(node_->get_logger(), "Goal Cancelled !");
        return;
      }
      feedback->status = get_action_completion_percentage();
      goal_handle->publish_feedback(feedback);

      // Send back results upon task completion
      if (std::abs(get_action_completion_percentage() - 1.00) < 0.000001) {
        // Complete the state transition cycle and go to HOME state
        RCLCPP_INFO(node_->get_logger(), "Set current state to HOME");
        set_current_state(State::HOME);
        results->success = true;
        goal_handle->succeed(results);
        return;
      }
    }
  }
}

moveit::core::MoveItErrorCode PdfBeamtimeFidPoseServer::run_fsm(
  std::shared_ptr<const pdf_beamtime_interfaces::action::FidPoseControlMsg_Goal> goal)
{
  RCLCPP_INFO(
    node_->get_logger(), "Executing state %s",
    external_state_names_[static_cast<int>(current_state_)].c_str());
  moveit::core::MoveItErrorCode motion_results = moveit::core::MoveItErrorCode::FAILURE;

  sample_id = goal->sample_id;
  switch (current_state_) {
    case State::HOME:
      // Moves the robot to pickup approach.
      // If success: change state, increment progress, reset internel state
      motion_results = inner_state_machine_->open_gripper();
      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
      inner_state_machine_->set_internal_state(Internal_State::RESTING);

      motion_results = inner_state_machine_->move_robot(
        move_group_interface_, pickup_approach_);
      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}

      set_current_state(State::PICKUP_APPROACH);
      inner_state_machine_->set_internal_state(Internal_State::RESTING);
      progress_ = progress_ + 1.0;
      break;

    case State::PICKUP_APPROACH: {
        // Moves the robot to pickup in two steps
        // 1. Adust the wrist 3 and wrist 2 positions to face the gripper towards the sample

        std::pair<double, double> new_wrist_angles = tf_utilities_->get_wrist_elbow_alignment(
          move_group_interface_, sample_id);
        adjusted_pickup_ = pickup_approach_;
        adjusted_pickup_[4] = new_wrist_angles.first;

        motion_results = inner_state_machine_->move_robot(move_group_interface_, adjusted_pickup_);
        if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
        inner_state_machine_->set_internal_state(Internal_State::RESTING);

        // 2. Cartesian move the robot to pick up position
        // 2.1 Adjust the z distance
        motion_results = inner_state_machine_->move_robot_cartesian(
          move_group_interface_, tf_utilities_->get_pickup_action_z_adj(
            move_group_interface_, sample_id));
        if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
        inner_state_machine_->set_internal_state(Internal_State::RESTING);

        // 2.2 Cartesian move to pre-pickup location in front of the sample
        motion_results = inner_state_machine_->move_robot_cartesian(
          move_group_interface_, tf_utilities_->get_pickup_action_pre_pickup(
            move_group_interface_, sample_id));
        if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
        inner_state_machine_->set_internal_state(Internal_State::RESTING);

        // Save the joint angles for the pre-pickup. Use this for moving the sample back
        move_group_interface_.getCurrentState()->copyJointGroupPositions(
          move_group_interface_.getCurrentState()->getRobotModel()->getJointModelGroup(
            "ur_arm"),
          pre_pickup_approach_joints_);

        // 2.3 Cartesian move to pickup
        motion_results = inner_state_machine_->move_robot_cartesian(
          move_group_interface_, tf_utilities_->get_pickup_action_pickup(
            move_group_interface_, sample_id));

        if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}

        // Save the joint angles for the pickup.
        move_group_interface_.getCurrentState()->copyJointGroupPositions(
          move_group_interface_.getCurrentState()->getRobotModel()->getJointModelGroup(
            "ur_arm"),
          pickup_joints_);

        // Insert the saved joint positions to maps
        pickup_storage_map_.insert_or_assign(sample_id, pickup_joints_);
        pre_pickup_approach_storage_map_.insert_or_assign(sample_id, pre_pickup_approach_joints_);
        // State propagation
        inner_state_machine_->set_internal_state(Internal_State::RESTING);
        set_current_state(State::PICKUP);
        progress_ = progress_ + 1.0;
      }
      break;

    case State::PICKUP:
      // Pick up object by closing gripper. If success: move to grasp success with progress.
      // if fails, move to grasp_failure
      if (this->gripper_present_) {
        motion_results = inner_state_machine_->close_gripper();
        if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
          progress_ = progress_ + 1.0;
          set_current_state(State::GRASP_SUCCESS);
          inner_state_machine_->set_internal_state(Internal_State::RESTING);
        } else {
          set_current_state(State::GRASP_FAILURE);
          inner_state_machine_->set_internal_state(Internal_State::RESTING);
        }
      }
      break;

    case State::GRASP_SUCCESS:
      // Successfully grasped. Do pickup retreat
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_, pre_pickup_approach_joints_);
      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
      inner_state_machine_->set_internal_state(Internal_State::RESTING);

      motion_results = inner_state_machine_->move_robot(
        move_group_interface_, adjusted_pickup_);
      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
      inner_state_machine_->set_internal_state(Internal_State::RESTING);

      inner_state_machine_->set_internal_state(Internal_State::RESTING);
      set_current_state(State::PICKUP_RETREAT);
      progress_ = progress_ + 1.0;

      break;

    case State::GRASP_FAILURE:
      // Gripper did not close. failed. move to Pickup approach
      // if (!goal->sample_return) {
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_, pre_pickup_approach_joints_);
      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {
        progress_ = progress_ - 1.0;
        break;
      }
      inner_state_machine_->set_internal_state(Internal_State::RESTING);

      motion_results = inner_state_machine_->move_robot(
        move_group_interface_, pickup_approach_);

      if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
        set_current_state(State::PICKUP_APPROACH);
        inner_state_machine_->set_internal_state(Internal_State::RESTING);
      }
      progress_ = progress_ - 1.0;
      break;

    case State::PICKUP_RETREAT:
      // Sample in hand. Move to place approach.
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_,
        goal->inbeam_approach);
      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
      inner_state_machine_->set_internal_state(Internal_State::RESTING);
      set_current_state(State::PLACE_APPROACH);
      progress_ = progress_ + 1.0;

      break;

    case State::PLACE_APPROACH: {
        // Move sample to place
        motion_results = inner_state_machine_->move_robot(
          move_group_interface_,
          goal->inbeam);

        if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
        set_current_state(State::PLACE);
        inner_state_machine_->set_internal_state(Internal_State::RESTING);
        progress_ = progress_ + 1.0;
      }
      break;

    case State::PLACE:
      // Place the sample.
      if (this->gripper_present_) {
        motion_results = inner_state_machine_->open_gripper();
        if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
          progress_ = progress_ + 1.0;
          set_current_state(State::RELEASE_SUCCESS);
          inner_state_machine_->set_internal_state(Internal_State::RESTING);
        } else {
          set_current_state(State::RELEASE_FAILURE);
          inner_state_machine_->set_internal_state(Internal_State::RESTING);
        }
      }
      break;

    case State::RELEASE_SUCCESS:
      // Sample was successfully released.
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_,
        goal->inbeam_approach);

      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
      inner_state_machine_->set_internal_state(Internal_State::RESTING);
      set_current_state(State::PLACE_RETREAT);
      progress_ = progress_ + 1.0;

      break;

    case State::PLACE_RETREAT:
      // Move back to rest/home position
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_, goal_home_);
      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
      set_current_state(State::HOME);
      inner_state_machine_->set_internal_state(Internal_State::RESTING);
      progress_ = progress_ + 1.0;
      pickup_pose_saved = true;
      break;

    default:
      break;
  }
  return motion_results;
}


moveit::core::MoveItErrorCode PdfBeamtimeFidPoseServer::run_return_fsm(
  std::shared_ptr<const pdf_beamtime_interfaces::action::FidPoseControlMsg_Goal> goal)
{
  sample_id = goal->sample_id;
  RCLCPP_INFO(
    node_->get_logger(), "Executing state %s",
    external_state_names_[static_cast<int>(current_state_)].c_str());
  moveit::core::MoveItErrorCode motion_results = moveit::core::MoveItErrorCode::FAILURE;
  switch (current_state_) {
    case State::HOME:
      // Moves the robot to pickup approach.
      // If success: change state, increment progress, reset internel state
      motion_results = inner_state_machine_->open_gripper();
      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
      inner_state_machine_->set_internal_state(Internal_State::RESTING);

      motion_results = inner_state_machine_->move_robot(
        move_group_interface_, goal->inbeam_approach);
      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}

      set_current_state(State::PICKUP_APPROACH);
      inner_state_machine_->set_internal_state(Internal_State::RESTING);
      progress_ = progress_ + 1.0;
      break;

    case State::PICKUP_APPROACH: {
        // Moves the robot to inbeam
        motion_results = inner_state_machine_->move_robot(
          move_group_interface_, goal->inbeam);

        if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
        // State propagation
        inner_state_machine_->set_internal_state(Internal_State::RESTING);
        set_current_state(State::PICKUP);
        progress_ = progress_ + 1.0;
      }
      break;

    case State::PICKUP:
      // Pick up object by closing gripper. If success: move to grasp success with progress.
      // if fails, move to grasp_failure
      if (this->gripper_present_) {
        motion_results = inner_state_machine_->close_gripper();
        if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
          progress_ = progress_ + 1.0;
          set_current_state(State::GRASP_SUCCESS);
          inner_state_machine_->set_internal_state(Internal_State::RESTING);
        } else {
          set_current_state(State::GRASP_FAILURE);
          inner_state_machine_->set_internal_state(Internal_State::RESTING);
        }
      }
      break;

    case State::GRASP_SUCCESS:
      // Successfully grasped. Do pickup retreat to inbeam approach
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_, goal->inbeam_approach);
      inner_state_machine_->set_internal_state(Internal_State::RESTING);
      set_current_state(State::PICKUP_RETREAT);
      progress_ = progress_ + 1.0;
      break;

    case State::GRASP_FAILURE:
      // Gripper did not close. failed. move to Pickup approach
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_, goal->inbeam_approach);
      if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
        set_current_state(State::PICKUP_APPROACH);
        inner_state_machine_->set_internal_state(Internal_State::RESTING);
      }
      progress_ = progress_ - 1.0;
      break;

    case State::PICKUP_RETREAT:
      // Sample in hand. Move to place approach.
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_,
        goal->inbeam_approach);
      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
      inner_state_machine_->set_internal_state(Internal_State::RESTING);
      set_current_state(State::PLACE_APPROACH);
      progress_ = progress_ + 1.0;

      break;

    case State::PLACE_APPROACH: {
        // Move sample to place
        motion_results = inner_state_machine_->move_robot(
          move_group_interface_,
          pickup_approach_);
        if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
        inner_state_machine_->set_internal_state(Internal_State::RESTING);

        motion_results = inner_state_machine_->move_robot(
          move_group_interface_,
          pre_pickup_approach_storage_map_[sample_id]);
        if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
        inner_state_machine_->set_internal_state(Internal_State::RESTING);

        motion_results = inner_state_machine_->move_robot(
          move_group_interface_,
          pickup_storage_map_[sample_id]);

        if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}

        inner_state_machine_->set_internal_state(Internal_State::RESTING);
        set_current_state(State::PLACE);
        progress_ = progress_ + 1.0;
      }
      break;

    case State::PLACE:
      // Place the sample.
      if (this->gripper_present_) {
        motion_results = inner_state_machine_->open_gripper();
        if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
          progress_ = progress_ + 1.0;
          set_current_state(State::RELEASE_SUCCESS);
          inner_state_machine_->set_internal_state(Internal_State::RESTING);
        } else {
          set_current_state(State::RELEASE_FAILURE);
          inner_state_machine_->set_internal_state(Internal_State::RESTING);
        }
      }
      break;

    case State::RELEASE_SUCCESS:
      // Sample was successfully released.
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_,
        pre_pickup_approach_storage_map_[sample_id]);

      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
      inner_state_machine_->set_internal_state(Internal_State::RESTING);

      motion_results = inner_state_machine_->move_robot(
        move_group_interface_,
        pickup_approach_);

      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
      inner_state_machine_->set_internal_state(Internal_State::RESTING);
      set_current_state(State::PLACE_RETREAT);
      progress_ = progress_ + 1.0;

      break;

    case State::PLACE_RETREAT:
      // Move back to rest/home position
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_, goal_home_);
      if (motion_results == moveit::core::MoveItErrorCode::FAILURE) {break;}
      set_current_state(State::HOME);
      inner_state_machine_->set_internal_state(Internal_State::RESTING);
      progress_ = progress_ + 1.0;
      pickup_pose_saved = false;
      break;

    default:
      break;
  }
  return motion_results;
}

moveit::core::MoveItErrorCode PdfBeamtimeFidPoseServer::return_sample()
{
  moveit::core::MoveItErrorCode motion_results = moveit::core::MoveItErrorCode::FAILURE;

  if (pickup_pose_saved) {
    // Move to pickup approach
    motion_results = inner_state_machine_->move_robot(
      move_group_interface_,
      goal->pickup_approach);

    motion_results = inner_state_machine_->move_robot(
      move_group_interface_,
      pickup_joints_);

    motion_results = inner_state_machine_->open_gripper();
  }
  return motion_results;
}

void PdfBeamtimeFidPoseServer::execute_cleanup()
{
  inner_state_machine_->set_internal_state(Internal_State::CLEANUP);
  RCLCPP_INFO(node_->get_logger(), "Cleanup is in progress");

  switch (current_state_) {
    case State::GRASP_SUCCESS: {
        // Move to pick up

        if (pickup_pose_saved) {
          inner_state_machine_->move_robot(
            move_group_interface_,
            pickup_joints_);
          inner_state_machine_->open_gripper();
          inner_state_machine_->move_robot(
            move_group_interface_,
            goal->pickup_approach);
        }
      }
      break;

    case State::PICKUP_APPROACH:
      inner_state_machine_->move_robot(move_group_interface_, goal->pickup_approach);
      break;

    case State::PICKUP_RETREAT:
    case State::PLACE_APPROACH:
    case State::PLACE:
      return_sample();
      break;

    case State::PICKUP:
    case State::GRASP_FAILURE: {
        // Move back to pickup approach
        std::pair<double, double> new_wrist_angles = tf_utilities_->get_wrist_elbow_alignment(
          move_group_interface_, sample_id);
        adjusted_pickup_ = goal->pickup_approach;
        adjusted_pickup_[4] = new_wrist_angles.first;
        adjusted_pickup_[5] = new_wrist_angles.second;
        inner_state_machine_->move_robot(
          move_group_interface_,
          adjusted_pickup_);
      }
      break;

    case State::RELEASE_FAILURE:
    case State::RELEASE_SUCCESS: {
        std::pair<double, double> new_wrist_angles = tf_utilities_->get_wrist_elbow_alignment(
          move_group_interface_, sample_id);
        adjusted_place_ = goal->place_approach;
        adjusted_place_[4] = new_wrist_angles.first;
        adjusted_place_[5] = new_wrist_angles.second;
        inner_state_machine_->move_robot(
          move_group_interface_,
          adjusted_place_);
      }
      break;

    default:
      break;
  }
  reset_fsm();
  inner_state_machine_->set_internal_state(Internal_State::RESTING);
  RCLCPP_INFO(node_->get_logger(), "Cleanup is complete");
}
