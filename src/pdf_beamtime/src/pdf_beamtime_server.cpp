/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pdf_beamtime/pdf_beamtime_server.hpp>

using moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;

PdfBeamtimeServer::PdfBeamtimeServer(
  const std::string & move_group_name = "ur_manipulator",
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
  std::string action_name = "pdf_beamtime_action_server")
: node_(std::make_shared<rclcpp::Node>("pdf_beamtime_server", options)),
  interrupt_node_(std::make_shared<rclcpp::Node>("interrupt_server")),
  gripper_node_(std::make_shared<rclcpp::Node>("gripper_node_client")),
  move_group_interface_(node_, move_group_name),
  planning_scene_interface_()
{
  // Add the obstacles
  planning_scene_interface_.applyCollisionObjects(create_env());

  // // Create the services
  new_box_obstacle_service_ = node_->create_service<BoxObstacleMsg>(
    "pdf_new_box_obstacle",
    std::bind(
      &PdfBeamtimeServer::new_obstacle_service_cb<BoxObstacleMsg::Request,
      BoxObstacleMsg::Response>, this, _1, _2));

  new_cylinder_obstacle_service_ = node_->create_service<CylinderObstacleMsg>(
    "pdf_new_cylinder_obstacle",
    std::bind(
      &PdfBeamtimeServer::new_obstacle_service_cb<CylinderObstacleMsg::Request,
      CylinderObstacleMsg::Response>, this, _1, _2));

  update_obstacles_service_ = node_->create_service<UpdateObstaclesMsg>(
    "pdf_update_obstacles",
    std::bind(
      &PdfBeamtimeServer::update_obstacles_service_cb, this, _1, _2));

  remove_obstacles_service_ = node_->create_service<DeleteObstacleMsg>(
    "pdf_remove_obstacle",
    std::bind(
      &PdfBeamtimeServer::remove_obstacles_service_cb, this, _1, _2));

  // Create the action server
  action_server_ = rclcpp_action::create_server<PickPlaceControlMsg>(
    this->node_,
    action_name,
    std::bind(&PdfBeamtimeServer::handle_goal, this, _1, _2),
    std::bind(&PdfBeamtimeServer::handle_cancel, this, _1),
    std::bind(&PdfBeamtimeServer::handle_accepted, this, _1));

  // Initialize to home
  current_state_ = State::HOME;
  gripper_present_ = node_->get_parameter("gripper_present").as_bool();
  inner_state_machine_ = new InnerStateMachine(node_, gripper_node_);

  bluesky_interrupt_service_ = interrupt_node_->create_service<BlueskyInterruptMsg>(
    "bluesky_interrupt",
    std::bind(
      &PdfBeamtimeServer::bluesky_interrupt_cb, this, _1, _2));

  move_group_interface_.setMaxVelocityScalingFactor(0.1);
  move_group_interface_.setMaxAccelerationScalingFactor(0.1);
}

void PdfBeamtimeServer::bluesky_interrupt_cb(
  const std::shared_ptr<BlueskyInterruptMsg::Request> request,
  std::shared_ptr<BlueskyInterruptMsg::Response> response)
{
  // conver the request string to the mapping enum
  Internal_State request_enum = interrupt_map[request->interrupt_type];

  switch (request_enum) {
    case Internal_State::MOVING:
      handle_resume();
      response->results = true;
      break;

    case Internal_State::PAUSED:
      handle_pause();
      response->results = true;
      break;

    case Internal_State::STOP:
      handle_stop();
      response->results = true;
      break;

    case Internal_State::ABORT:
      handle_abort();
      response->results = true;
      break;

    case Internal_State::HALT:
      handle_halt();
      response->results = true;
      break;

    default:
      RCLCPP_ERROR(node_->get_logger(), "Incorrect interrput type");
      response->results = false;
      break;
  }
}
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr PdfBeamtimeServer::getNodeBaseInterface()
// Expose the node base interface so that the node can be added to a component manager.
{
  return node_->get_node_base_interface();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr PdfBeamtimeServer::
getInterruptNodeBaseInterface()
// Expose the node base interface of the bluesky interrupt node
// so that the node can be added to a component manager.
{
  return interrupt_node_->get_node_base_interface();
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
  goal = goal_handle->get_goal();
  //  Variables for feedback and results
  auto feedback = std::make_shared<PickPlaceControlMsg::Feedback>();
  auto results = std::make_shared<PickPlaceControlMsg::Result>();
  moveit::core::MoveItErrorCode fsm_results;
  progress_ = 0.0;
  goal_home_ = node_->get_parameter("home_angles").as_double_array();
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

      fsm_results = run_fsm(goal);
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

std::vector<moveit_msgs::msg::CollisionObject> PdfBeamtimeServer::create_env()
// Builds a vector of obstacle as defined in the .yaml file and returns the vector
{
  obstacle_type_map_.insert(std::pair<std::string, int>("CYLINDER", 1));
  obstacle_type_map_.insert(std::pair<std::string, int>("BOX", 2));

  // int num_objects = node_->get_parameter("num_objects").as_int();
  std::vector<std::string> object_names = node_->get_parameter("object_names").as_string_array();

  std::vector<moveit_msgs::msg::CollisionObject> all_obstacles;

  // Create objects in a loop
  for (size_t i = 0; i < object_names.size(); i++) {
    std::string name = object_names[i];  // get each name here as it uses as a parameter field

    moveit_msgs::msg::CollisionObject obj;    // collision object
    geometry_msgs::msg::Pose pose;    // object pose
    obj.id = name;
    obj.header.frame_id = "world";

    // Map to the correct int
    switch (obstacle_type_map_[node_->get_parameter("objects." + name + ".type").as_string()]) {
      case 1:
        // TODO(ChandimaFernando): Break these following statements to functions
        // These objects are cylinders
        obj.primitives.resize(1);
        obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        // Populate the fields from the parameters
        obj.primitives[0].dimensions =
        {node_->get_parameter("objects." + name + ".h").as_double(),
          node_->get_parameter("objects." + name + ".r").as_double()};

        pose.position.x = node_->get_parameter("objects." + name + ".x").as_double();
        pose.position.y = node_->get_parameter("objects." + name + ".y").as_double();
        pose.position.z = node_->get_parameter("objects." + name + ".z").as_double();
        obj.pose = pose;
        break;

      case 2:
        // These objects are boxes
        obj.primitives.resize(1);
        obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        obj.primitives[0].dimensions =
        {node_->get_parameter("objects." + name + ".w").as_double(),
          node_->get_parameter("objects." + name + ".d").as_double(),
          node_->get_parameter("objects." + name + ".h").as_double()};

        pose.position.x = node_->get_parameter("objects." + name + ".x").as_double();
        pose.position.y = node_->get_parameter("objects." + name + ".y").as_double();
        pose.position.z = node_->get_parameter("objects." + name + ".z").as_double();
        obj.pose = pose;
        break;

      default:
        break;
    }

    all_obstacles.push_back(obj);
  }
  return all_obstacles;
}

void PdfBeamtimeServer::update_obstacles_service_cb(
  const std::shared_ptr<UpdateObstaclesMsg::Request> request,
  std::shared_ptr<UpdateObstaclesMsg::Response> response)
{
  for (size_t idx = 0; idx < request->property.size(); ++idx) {
    // Update the existing parameter
    auto results = node_->set_parameter(
      rclcpp::Parameter(
        "objects." + request->name + "." + request->property[idx],
        request->value[idx]));

    if (results.successful) {
      response->results = "Success";
      RCLCPP_INFO(node_->get_logger(), "Parameter set is successfully.");
    } else {
      response->results = "Failure";
      RCLCPP_ERROR(node_->get_logger(), "Failed to set parameter");
    }
  }
  planning_scene_interface_.applyCollisionObjects(create_env());
}

void PdfBeamtimeServer::remove_obstacles_service_cb(
  const std::shared_ptr<DeleteObstacleMsg::Request> request,
  std::shared_ptr<DeleteObstacleMsg::Response> response)
{
  std::vector<std::string> removable_object;
  try {
    // Remove the new obstacle name from the existing list
    auto obj_param = node_->get_parameters({"object_names"})[0].as_string_array();

    obj_param.erase(
      std::remove_if(
        obj_param.begin(), obj_param.end(), [&request](const std::string & param) {
          return param == request->name;
        }), obj_param.end());
    // Add the obstacle id (which is the obstacle name to a list)
    removable_object.push_back(request->name);
    response->results = "Success";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    response->results = "Failure";
  }
  planning_scene_interface_.removeCollisionObjects(removable_object);
}

template<typename RequestT, typename ResponseT>
void PdfBeamtimeServer::new_obstacle_service_cb(
  const typename RequestT::SharedPtr request,
  typename ResponseT::SharedPtr response)
{
  try {
    // Adds the new obstacle name to the existing list
    auto obj_param = node_->get_parameters({"object_names"})[0].as_string_array();
    obj_param.push_back(request->name);
    node_->set_parameters(
      {rclcpp::Parameter("object_names", obj_param)});

    // Add the common parameters
    node_->declare_parameter("objects." + request->name + ".type", request->type);
    const std::vector<std::pair<std::string, double>> parameters = {
      {"x", request->x}, {"y", request->y}, {"z", request->z}, {"h", request->h}};

    for (const auto & param : parameters) {
      node_->declare_parameter("objects." + request->name + "." + param.first, param.second);
    }

    // Checks the message type to differentiate Box type from Cylinder
    if constexpr (std::is_same_v<RequestT, BoxObstacleMsg::Request>) {
      node_->declare_parameter("objects." + request->name + ".w", request->w);
      node_->declare_parameter("objects." + request->name + ".d", request->d);
    } else if constexpr (std::is_same_v<RequestT, CylinderObstacleMsg::Request>) {
      node_->declare_parameter("objects." + request->name + ".r", request->r);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Message type not registered");
    }
    // Return response results
    response->results = "Success";
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    response->results = "Failure";
  }
  // Update the whole environment
  planning_scene_interface_.applyCollisionObjects(create_env());
}

float PdfBeamtimeServer::get_action_completion_percentage()
{
  return progress_ / total_states_;
}

moveit::core::MoveItErrorCode PdfBeamtimeServer::run_fsm(
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

    case State::PICKUP_APPROACH:
      // Moves the robot to pickup.
      // If success: change state, increment progress, reset internel state
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_,
        goal->pickup);
      if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
        set_current_state(State::PICKUP);
        inner_state_machine_->set_internal_state(Internal_State::RESTING);
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
        move_group_interface_,
        goal->pickup_approach);

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
        goal->pickup_approach);
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

    case State::PLACE_APPROACH:
      // Move sample to place
      motion_results = inner_state_machine_->move_robot(
        move_group_interface_,
        goal->place);
      if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
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
        goal->place_approach);
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

bool PdfBeamtimeServer::reset_fsm()
{
  bool reset_results = false;
  // This prevents the internal state reset while in clean up
  if (inner_state_machine_->get_internal_state() != Internal_State::CLEANUP) {
    inner_state_machine_->set_internal_state(Internal_State::RESTING);
  }
  auto motion_results = inner_state_machine_->move_robot(move_group_interface_, goal_home_);
  if (this->gripper_present_) {
    inner_state_machine_->open_gripper();
  }
  if (motion_results == moveit::core::MoveItErrorCode::SUCCESS) {
    set_current_state(State::HOME);
    reset_results = true;
    RCLCPP_INFO(node_->get_logger(), "State machine was RESET");
  } else {
    RCLCPP_INFO(node_->get_logger(), "State machine reset FAILED");
  }
  progress_ = 0.0;

  return reset_results;
}

moveit::core::MoveItErrorCode PdfBeamtimeServer::return_sample()
{
  moveit::core::MoveItErrorCode motion_results = moveit::core::MoveItErrorCode::FAILURE;

  // Move to pickup approach
  motion_results = inner_state_machine_->move_robot(
    move_group_interface_,
    goal->pickup_approach);
  // Move to pick up
  motion_results = inner_state_machine_->move_robot(
    move_group_interface_,
    goal->pickup);
  // Open the gripper
  inner_state_machine_->open_gripper();
  // Move back to pickup approach
  motion_results = inner_state_machine_->move_robot(
    move_group_interface_,
    goal->pickup_approach);

  return motion_results;
}

void PdfBeamtimeServer::handle_pause()
{
  RCLCPP_INFO(node_->get_logger(), "PAUSED command received");
  inner_state_machine_->pause(move_group_interface_);
}

void PdfBeamtimeServer::handle_stop()
{
  RCLCPP_INFO(node_->get_logger(), "STOP command received");
  execute_cleanup();
}

void PdfBeamtimeServer::execute_cleanup()
{
  inner_state_machine_->set_internal_state(Internal_State::CLEANUP);
  RCLCPP_INFO(node_->get_logger(), "Cleanup is in progress");

  switch (current_state_) {
    case State::GRASP_SUCCESS:
      inner_state_machine_->move_robot(move_group_interface_, goal->pickup);
      inner_state_machine_->open_gripper();
      inner_state_machine_->move_robot(move_group_interface_, goal->pickup_approach);
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
    case State::GRASP_FAILURE:
      inner_state_machine_->move_robot(move_group_interface_, goal->pickup_approach);
      break;

    case State::RELEASE_FAILURE:
    case State::RELEASE_SUCCESS:
      inner_state_machine_->move_robot(move_group_interface_, goal->place_approach);
      break;

    default:
      break;
  }
  reset_fsm();
  inner_state_machine_->set_internal_state(Internal_State::RESTING);
  RCLCPP_INFO(node_->get_logger(), "Cleanup is complete");
}

void PdfBeamtimeServer::handle_abort()
{
  RCLCPP_INFO(node_->get_logger(), "ABORT command received");
  execute_cleanup();
}

void PdfBeamtimeServer::handle_halt()
{
  RCLCPP_INFO(node_->get_logger(), "HALT command received");
  inner_state_machine_->halt(move_group_interface_);
}

void PdfBeamtimeServer::handle_resume()
{
  RCLCPP_INFO(node_->get_logger(), "RESUME command received");
  inner_state_machine_->set_internal_state(Internal_State::RESTING);
}

void PdfBeamtimeServer::set_current_state(State state)
{
  RCLCPP_INFO(
    node_->get_logger(), "[%s] Current state changed to %s.",
    external_state_names_[static_cast<int>(current_state_)].c_str(),
    external_state_names_[static_cast<int>(state)].c_str());
  current_state_ = state;
}
