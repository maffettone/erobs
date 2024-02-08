/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/

#include <pdf_beamtime/pdf_beamtime_server.hpp>


using moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;

PdfBeamtimeServer::PdfBeamtimeServer(
  const std::string & move_group_name = "ur_arm",
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
  std::string action_name = "pdf_beamtime_action_server")
: node_(std::make_shared<rclcpp::Node>("pdf_beamtime_server", options)),
  move_group_interface_(node_, move_group_name),
  planning_scene_interface_()
{
  // Create the action server
  action_server_ = rclcpp_action::create_server<PickPlaceControlMsg>(
    this->node_,
    action_name,
    std::bind(&PdfBeamtimeServer::handle_goal, this, _1, _2),
    std::bind(&PdfBeamtimeServer::handle_cancel, this, _1),
    std::bind(&PdfBeamtimeServer::handle_accepted, this, _1));

  // // Initialize to home
  current_state_ = State::HOME;
  gripper_present_ = node_->get_parameter("gripper_present").as_bool();

  for (int i = 0; i < num_of_states; ++i) {
    state_holder_[i] = new InnerStateMachine(node_, static_cast<State>(i));
  }

}
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr PdfBeamtimeServer::getNodeBaseInterface()
// Expose the node base interface so that the node can be added to a component manager.
{
  return node_->get_node_base_interface();
}

rclcpp_action::GoalResponse PdfBeamtimeServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PickPlaceControlMsg::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void PdfBeamtimeServer::handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceControlMsg>> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&PdfBeamtimeServer::execute, this, _1), goal_handle}.detach();
}

// Receiving the cancel request.
rclcpp_action::CancelResponse PdfBeamtimeServer::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceControlMsg>> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PdfBeamtimeServer::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceControlMsg>> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  //  Variables for feedback and results
  auto feedback = std::make_shared<PickPlaceControlMsg::Feedback>();
  auto results = std::make_shared<PickPlaceControlMsg::Result>();
  bool fsm_results = true;
  bool state_transition_complete = false;
  progress_ = 0.0;
  auto goal_home = node_->get_parameter("home_angles").as_double_array();
  feedback->status = get_action_completion_percentage();

  RCLCPP_INFO(
    node_->get_logger(), "Current state is state %s.",
    state_names_[static_cast<int>(current_state_)].c_str());
  RCLCPP_INFO(node_->get_logger(), "Robot is moved the HOME state first for a new execution.");
  // if (current_state_ != State::HOME) {
  //   fsm_results = reset_fsm(goal_home);
  // }
  while (!state_transition_complete) {
    fsm_results = run_fsm(goal);
    if (!fsm_results) {
      // Abort the execution if move_group_ fails
      results->success = fsm_results;
      goal_handle->abort(results);
      RCLCPP_ERROR(node_->get_logger(), "Goal aborted !");
      return;
    }

    if (goal_handle->is_canceling()) {
      // Reset the fsm if goal is cancelled by the action client
      results->success = false;
      // reset_fsm(goal_home);
      goal_handle->canceled(results);
      RCLCPP_WARN(node_->get_logger(), "Goal Cancelled !");
      return;
    }
    feedback->status = get_action_completion_percentage();
    goal_handle->publish_feedback(feedback);

    // This marks the completion of a state transition cycle
    if (!fsm_results) {
      state_transition_complete = true;
    }

  }

  if (current_state_ == State::HOME) {
    results->success = fsm_results;
    goal_handle->succeed(results);
  }
}

float PdfBeamtimeServer::get_action_completion_percentage()
{
  return progress_ / total_states_;
}

bool PdfBeamtimeServer::run_fsm(
  std::shared_ptr<const pdf_beamtime_interfaces::action::PickPlaceControlMsg_Goal> goal)
{
  RCLCPP_INFO(
    node_->get_logger(), "Executing state %s",
    state_names_[static_cast<int>(current_state_)].c_str());
  bool state_transition = true;
  switch (current_state_) {
    case State::HOME:
      {
        // state_transition = set_joint_goal(goal->pickup_approach);
        state_holder_[static_cast<int>(State::HOME)]->set_active_true();
        state_holder_[static_cast<int>(State::HOME)]->set_joint_goal(goal->pickup_approach);
        auto next_state =
          state_holder_[static_cast<int>(State::HOME)]->move_robot(move_group_interface_);
        state_holder_[static_cast<int>(State::HOME)]->set_active_false();

        current_state_ = next_state;

      }

      break;

    case State::PICKUP_APPROACH:
      {
        state_holder_[static_cast<int>(State::PICKUP_APPROACH)]->set_active_true();
        state_holder_[static_cast<int>(State::PICKUP_APPROACH)]->set_joint_goal(goal->pickup);
        auto next_state =
          state_holder_[static_cast<int>(State::PICKUP_APPROACH)]->move_robot(move_group_interface_);
        state_holder_[static_cast<int>(State::PICKUP_APPROACH)]->set_active_false();
        current_state_ = next_state;

      }
      break;

    case State::PICKUP:
      {
        state_holder_[static_cast<int>(State::PICKUP)]->set_active_true();
        auto next_state =
          state_holder_[static_cast<int>(State::PICKUP)]->close_gripper();
        state_holder_[static_cast<int>(State::PICKUP)]->set_active_false();
        current_state_ = next_state;

      }
      break;

    case State::GRASP_SUCCESS:
      {
        state_holder_[static_cast<int>(State::GRASP_SUCCESS)]->set_active_true();
        state_holder_[static_cast<int>(State::GRASP_SUCCESS)]->set_joint_goal(goal->pickup_approach);
        auto next_state =
          state_holder_[static_cast<int>(State::GRASP_SUCCESS)]->move_robot(move_group_interface_);
        state_holder_[static_cast<int>(State::GRASP_SUCCESS)]->set_active_false();
        current_state_ = next_state;
      }
      break;

    case State::PICKUP_RETREAT:
      {
        state_holder_[static_cast<int>(State::PICKUP_RETREAT)]->set_active_true();
        state_holder_[static_cast<int>(State::PICKUP_RETREAT)]->set_joint_goal(goal->place_approach);
        auto next_state =
          state_holder_[static_cast<int>(State::PICKUP_RETREAT)]->move_robot(move_group_interface_);
        state_holder_[static_cast<int>(State::PICKUP_RETREAT)]->set_active_false();
        current_state_ = next_state;
      }
      break;

    case State::PLACE_APPROACH:
      {
        state_holder_[static_cast<int>(State::PLACE_APPROACH)]->set_active_true();
        state_holder_[static_cast<int>(State::PLACE_APPROACH)]->set_joint_goal(goal->place);
        auto next_state =
          state_holder_[static_cast<int>(State::PLACE_APPROACH)]->move_robot(move_group_interface_);
        state_holder_[static_cast<int>(State::PLACE_APPROACH)]->set_active_false();
        current_state_ = next_state;
      }
      break;

    case State::PLACE:
      {
        state_holder_[static_cast<int>(State::PLACE)]->set_active_true();
        auto next_state =
          state_holder_[static_cast<int>(State::PLACE)]->open_gripper();
        state_holder_[static_cast<int>(State::PLACE)]->set_active_false();
        current_state_ = next_state;

      }
      break;

    case State::RELEASE_SUCCESS:
      {
        state_holder_[static_cast<int>(State::RELEASE_SUCCESS)]->set_active_true();
        state_holder_[static_cast<int>(State::RELEASE_SUCCESS)]->set_joint_goal(goal->place_approach);
        auto next_state =
          state_holder_[static_cast<int>(State::RELEASE_SUCCESS)]->move_robot(move_group_interface_);
        state_holder_[static_cast<int>(State::RELEASE_SUCCESS)]->set_active_false();
        current_state_ = next_state;
      }
      break;

    case State::PLACE_RETREAT:
      {
        state_holder_[static_cast<int>(State::PLACE_RETREAT)]->set_active_true();
        state_holder_[static_cast<int>(State::PLACE_RETREAT)]->set_joint_goal(
          node_->get_parameter(
            "home_angles").as_double_array());
        auto next_state =
          state_holder_[static_cast<int>(State::PLACE_RETREAT)]->move_robot(move_group_interface_);
        state_holder_[static_cast<int>(State::PLACE_RETREAT)]->set_active_false();
        current_state_ = State::HOME;
        state_transition = false;

      }
      break;

    default:
      break;
  }

  progress_ = progress_ + 1.0;
  return state_transition;
}

bool PdfBeamtimeServer::reset_fsm(std::vector<double> joint_goal)
{
  RCLCPP_INFO(node_->get_logger(), "State machine was RESET");
  current_state_ = State::HOME;
  if (this->gripper_present_) {
    // gripper_open();
  }
  state_holder_[static_cast<int>(State::HOME)]->set_active_true();
  state_holder_[static_cast<int>(State::HOME)]->set_joint_goal(joint_goal);

  state_holder_[static_cast<int>(State::HOME)]->move_robot(move_group_interface_);
  state_holder_[static_cast<int>(State::HOME)]->set_active_false();
  current_state_ = State::HOME;
  return true;
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

  auto parent_node = std::make_shared<PdfBeamtimeServer>(
    "ur_arm",
    node_options);
  rclcpp::spin(parent_node->getNodeBaseInterface());
  rclcpp::shutdown();
  return 0;
}
