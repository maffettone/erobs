/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pdf_beamtime/pdf_beamtime_fidpose_server.hpp>

PdfBeamtimeFidPoseServer::PdfBeamtimeFidPoseServer(
  const std::string & move_group_name = "ur_manipulator",
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
  std::string action_name = "pdf_beamtime_action_server")
: PdfBeamtimeServer(move_group_name, options, action_name)
{
  tf_utilities_ = new TFUtilities(node_);
  RCLCPP_INFO(node_->get_logger(), "Inside the derived constructor");
}

moveit::core::MoveItErrorCode PdfBeamtimeFidPoseServer::run_fsm(
  std::shared_ptr<const pdf_beamtime_interfaces::action::PickPlaceControlMsg_Goal> goal)
{
  RCLCPP_INFO(
    node_->get_logger(), "Executing state %s",
    external_state_names_[static_cast<int>(current_state_)].c_str());
  moveit::core::MoveItErrorCode motion_results = moveit::core::MoveItErrorCode::FAILURE;
  switch (current_state_) {
    case State::HOME:
      // Moves the robot to pickup approach.
      // If success: change state, increment progress, reset internel state
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_,
        goal->pickup_approach);
      if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
        set_current_state(State::PICKUP_APPROACH);
        inner_state_machine_->set_internal_state(Internal_State::RESTING);
        progress_ = progress_ + 1.0;
      }
      break;

    case State::PICKUP_APPROACH: {
        // Moves the robot to pickup in two steps
        // 1. Adust the wrist 3 and wrist 2 positions to face the gripper towards the sample
        std::pair<double, double> new_wrist_angles = tf_utilities_->get_wrist_elbow_alignment(
          move_group_interface_);
        adjusted_pickup_ = goal->pickup_approach;
        adjusted_pickup_[4] = new_wrist_angles.first;
        adjusted_pickup_[5] = new_wrist_angles.second;
        motion_results = inner_state_machine_->move_robot(
          move_group_interface_,
          adjusted_pickup_);

        // 2. Cartesian move the robot to pick up position
        std::vector<geometry_msgs::msg::Pose> pickup_poses =
          tf_utilities_->get_pickup_action_waypoints(
          move_group_interface_);
        motion_results = inner_state_machine_->move_robot_cartesian(
          move_group_interface_,
          pickup_poses);

        if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
          set_current_state(State::PICKUP_APPROACH);
          inner_state_machine_->set_internal_state(Internal_State::RESTING);
          progress_ = progress_ + 1.0;
        }
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
        move_group_interface_,
        adjusted_pickup_);

      if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
        set_current_state(State::PICKUP_RETREAT);
        inner_state_machine_->set_internal_state(Internal_State::RESTING);
        progress_ = progress_ + 1.0;
      }
      break;

    case State::GRASP_FAILURE:
      // Gripper did not close. failed. move to Pickup approach
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_,
        adjusted_pickup_);
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
        goal->place_approach);
      if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
        set_current_state(State::PLACE_APPROACH);
        inner_state_machine_->set_internal_state(Internal_State::RESTING);
        progress_ = progress_ + 1.0;
      }
      break;

    case State::PLACE_APPROACH: {
        // Move sample to place
        // 1. Adust the wrist 3 and wrist 2 positions to face the gripper towards the sample
        std::pair<double, double> new_wrist_angles = tf_utilities_->get_wrist_elbow_alignment(
          move_group_interface_);
        adjusted_place_ = goal->place_approach;
        adjusted_place_[4] = new_wrist_angles.first;
        adjusted_place_[5] = new_wrist_angles.second;
        motion_results = inner_state_machine_->move_robot(
          move_group_interface_,
          adjusted_place_);

        // 2. Cartesian move the robot to pick up position
        std::vector<geometry_msgs::msg::Pose> place_poses =
          tf_utilities_->get_pickup_action_waypoints(
          move_group_interface_);
        motion_results = inner_state_machine_->move_robot_cartesian(
          move_group_interface_,
          place_poses);

        if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
          set_current_state(State::PLACE);
          inner_state_machine_->set_internal_state(Internal_State::RESTING);
          progress_ = progress_ + 1.0;
        }
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
        adjusted_place_);
      if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
        set_current_state(State::PLACE_RETREAT);
        inner_state_machine_->set_internal_state(Internal_State::RESTING);
        progress_ = progress_ + 1.0;
      }
      break;

    case State::PLACE_RETREAT:
      // Move back to rest/home position
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_, goal_home_);
      if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
        set_current_state(State::HOME);
        inner_state_machine_->set_internal_state(Internal_State::RESTING);
        progress_ = progress_ + 1.0;
      }
      break;

    default:
      break;
  }

  return motion_results;
}


moveit::core::MoveItErrorCode PdfBeamtimeFidPoseServer::return_sample()
{
  moveit::core::MoveItErrorCode motion_results = moveit::core::MoveItErrorCode::FAILURE;

  // Move to pickup approach
  motion_results = inner_state_machine_->move_robot(
    move_group_interface_,
    goal->pickup_approach);
  // Move to pick up
  std::pair<double, double> new_wrist_angles = tf_utilities_->get_wrist_elbow_alignment(
    move_group_interface_);
  adjusted_pickup_ = goal->pickup_approach;
  adjusted_pickup_[4] = new_wrist_angles.first;
  adjusted_pickup_[5] = new_wrist_angles.second;
  motion_results = inner_state_machine_->move_robot(
    move_group_interface_,
    adjusted_pickup_);

  // Cartesian move the robot to pick up position
  std::vector<geometry_msgs::msg::Pose> pickup_poses =
    tf_utilities_->get_pickup_action_waypoints(
    move_group_interface_);
  motion_results = inner_state_machine_->move_robot_cartesian(
    move_group_interface_,
    pickup_poses);
  // Open the gripper
  inner_state_machine_->open_gripper();

  // Move back to pickup approach
  motion_results = inner_state_machine_->move_robot(
    move_group_interface_,
    adjusted_pickup_);

  return motion_results;
}

void PdfBeamtimeFidPoseServer::execute_cleanup()
{
  inner_state_machine_->set_internal_state(Internal_State::CLEANUP);
  RCLCPP_INFO(node_->get_logger(), "Cleanup is in progress");

  switch (current_state_) {
    case State::GRASP_SUCCESS: {
        // Move to pick up
        std::pair<double, double> new_wrist_angles = tf_utilities_->get_wrist_elbow_alignment(
          move_group_interface_);
        adjusted_pickup_ = goal->pickup_approach;
        adjusted_pickup_[4] = new_wrist_angles.first;
        adjusted_pickup_[5] = new_wrist_angles.second;
        inner_state_machine_->move_robot(
          move_group_interface_,
          adjusted_pickup_);

        // Cartesian move the robot to pick up position
        std::vector<geometry_msgs::msg::Pose> pickup_poses =
          tf_utilities_->get_pickup_action_waypoints(
          move_group_interface_);
        inner_state_machine_->move_robot_cartesian(
          move_group_interface_,
          pickup_poses);

        inner_state_machine_->open_gripper();

        // Move back to pickup approach
        inner_state_machine_->move_robot(
          move_group_interface_,
          adjusted_pickup_);
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
          move_group_interface_);
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
          move_group_interface_);
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


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create a ROS logger for main scope
  auto const logger = rclcpp::get_logger("hello_moveit");
  using namespace std::chrono_literals;

  // Create a node for synchronously grabbing params
  auto parameter_client_node = rclcpp::Node::make_shared("param_client");
  auto parent_parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(parameter_client_node, "move_group");
  // Boiler plate wait block
  while (!parent_parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        logger, "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "move_group service not available, waiting again...");
  }
  // Get robot config parameters from parameter server
  auto parameters = parent_parameters_client->get_parameters(
    {"robot_description_semantic",
      "robot_description"});

  // Set node parameters using NodeOptions
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides(
  {
    {"robot_description_semantic", parameters[0].value_to_string()},
    {"robot_description", parameters[1].value_to_string()}
  });

  rclcpp::executors::MultiThreadedExecutor executor;

  auto beamtime_server = std::make_shared<PdfBeamtimeFidPoseServer>(
    "ur_arm",
    node_options);

  executor.add_node(beamtime_server->getNodeBaseInterface());
  executor.add_node(beamtime_server->getInterruptNodeBaseInterface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
