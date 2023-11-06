/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pdf_beamtime/pdf_beamtime_server.hpp>

using moveit::planning_interface::MoveGroupInterface;
using namespace std::placeholders;

PdfBeamtimeServer::PdfBeamtimeServer(
  const std::string & move_group_name = "ur_manipulator",
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
  std::string action_name = "pdf_beamtime_action_server")
: node_(std::make_shared<rclcpp::Node>("pdf_beamtime_server", options)),
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

  // // Initialize to home
  current_state_ = State::HOME;
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
  auto goal_home = node_->get_parameter("home_angles").as_double_array();
  feedback->status = get_state_completions();

  RCLCPP_INFO(
    node_->get_logger(), "Current state is state %d . If the robot is not in the HOME state for a new task execution, we move the robot to the HOME state first. ",
    static_cast<int>(current_state_));
  if (current_state_ != State::HOME) {
    fsm_results = reset_fsm(State::HOME, goal_home);
  }

  while (!state_transition_complete) {
    fsm_results = run_fsm(goal);
    if (!fsm_results) {
      // Abort the execution if move_group_ fails
      results->success = fsm_results;
      goal_handle->abort(results);
      break;
    }
    feedback->status = get_state_completions();
    goal_handle->publish_feedback(feedback);

    // This marks the completion of a state transition cycle
    if (current_state_ == State::HOME) {
      state_transition_complete = true;
    }
  }

  if (current_state_ == State::HOME) {
    results->success = fsm_results;
    goal_handle->succeed(results);
  }
}

bool PdfBeamtimeServer::set_joint_goal(std::vector<double> joint_goal)
{
  move_group_interface_.setJointValueTarget(joint_goal);
  // Create a plan to that target pose
  auto const [success, plan] = [this] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface_.plan(msg));
      return std::make_pair(ok, msg);
    }();
  // Execute the plan
  bool exec_results = false;
  if (success) {
    exec_results = static_cast<bool>(move_group_interface_.execute(plan));
    if (exec_results) {
      RCLCPP_INFO(node_->get_logger(), "Execution Succeeded");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Execution failed!");
    }
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
  }
  return exec_results;
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

int PdfBeamtimeServer::get_state_completions()
{
  return progress_ / total_states_;
}

// PdfBeamtimeServer::State
bool PdfBeamtimeServer::run_fsm(
  std::shared_ptr<const pdf_beamtime_interfaces::action::PickPlaceControlMsg_Goal> goal)
{
  RCLCPP_INFO(node_->get_logger(), "Executing state %d", static_cast<int>(current_state_));
  bool state_transition = false;
  switch (current_state_) {
    case State::HOME:
      state_transition = set_joint_goal(goal->pickup_approach);
      break;

    case State::PICKUP_APPROACH:
      state_transition = set_joint_goal(goal->pickup);
      break;

    case State::PICKUP:
      // gripper_close();
      state_transition = true;
      break;

    case State::GRASP_SUCCESS:
      state_transition = set_joint_goal(goal->pickup_approach);
      break;

    case State::PICKUP_RETREAT:
      state_transition = set_joint_goal(goal->place_approach);
      break;

    case State::PLACE_APPROACH:
      state_transition = set_joint_goal(goal->place);
      break;

    case State::PLACE:
      // gripper_open();
      state_transition = true;
      break;

    case State::RELEASE_SUCCESS:
      state_transition = set_joint_goal(goal->place_approach);
      break;

    case State::PLACE_RETREAT:
      state_transition = set_joint_goal(node_->get_parameter("home_angles").as_double_array());
      break;

    default:
      break;
  }
  std::this_thread::sleep_for(std::chrono::seconds(3)); // 3 second wait for robot movement to complete
  progress_++;
  // Propegate the current state here
  if (current_state_ == State::PLACE_RETREAT && state_transition) {
    // Complete the state transition cycle and go to HOME state
    RCLCPP_INFO(node_->get_logger(), "Set current state to HOME");
    current_state_ = State::HOME;
  } else {
    current_state_ = static_cast<State>(static_cast<int>(current_state_) + 1);
  }
  return state_transition;
}

bool PdfBeamtimeServer::reset_fsm(State STATE, std::vector<double> joint_goal)
{
  RCLCPP_INFO(node_->get_logger(), "State machine RESET");
  current_state_ = STATE;
  // gripper_open();
  return set_joint_goal(joint_goal);
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
    "ur_manipulator",
    node_options);
  rclcpp::spin(parent_node->getNodeBaseInterface());
  rclcpp::shutdown();
  return 0;
}
