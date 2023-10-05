#include <pdf_beamtime/simple_server.hpp>

using moveit::planning_interface::MoveGroupInterface;

SimpleServer::SimpleServer(
  const std::string & move_group_name = "ur_manipulator",
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: node_(std::make_shared<rclcpp::Node>("simple_server", options))
{
  // Add the obstacles
  this->planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
  this->planning_scene_interface->applyCollisionObjects(create_env());

  this->move_group_interface_ = new MoveGroupInterface(this->node_, move_group_name);

  // Create the action server
  this->action_server_ = rclcpp_action::create_server<SimpleActionMsg>(
    this->node_,
    this->ACTION_NAME,
    std::bind(&SimpleServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SimpleServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&SimpleServer::handle_accepted, this, std::placeholders::_1));

}
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr SimpleServer::getNodeBaseInterface()
// Expose the node base interface so that the node can be added to a component manager.
{
  return node_->get_node_base_interface();
}

rclcpp_action::GoalResponse SimpleServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const SimpleActionMsg::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void SimpleServer::handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<SimpleActionMsg>> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&SimpleServer::execute, this, _1), goal_handle}.detach();
}

// Receiving the cancel request.
rclcpp_action::CancelResponse SimpleServer::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<SimpleActionMsg>> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SimpleServer::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<SimpleActionMsg>> goal_handle)
{
  // A cool Moveit function
  std::vector<double> home_angles = node_->get_parameter("home_angles").as_double_array();

  this->move_group_interface_->setJointValueTarget(home_angles);
  // Create a plan to that target pose
  auto const [success, plan] = [this] {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(this->move_group_interface_->plan(msg));
      return std::make_pair(ok, msg);
    }();
  // Execute the plan
  if (success) {
    this->move_group_interface_->execute(plan);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
  }

}

std::vector<moveit_msgs::msg::CollisionObject> SimpleServer::create_env()
// Builds a vector of obstacle as defined in the .yaml file and returns the vector
{
  obstacle_type_map.insert(std::pair<std::string, int>("CYLINDER", 1));
  obstacle_type_map.insert(std::pair<std::string, int>("BOX", 2));

  // int num_objects = node_->get_parameter("num_objects").as_int();
  std::vector<std::string> object_names = node_->get_parameter("object_names").as_string_array();

  std::vector<moveit_msgs::msg::CollisionObject> all_obstacles;

  // Create objects in a recursion
  for (size_t i = 0; i < object_names.size(); i++) {
    std::string name = object_names[i];    //get each name here as it uses as a parameter field

    moveit_msgs::msg::CollisionObject obj;    // collision object
    geometry_msgs::msg::Pose pose;    // object pose
    obj.id = name;
    obj.header.frame_id = "world";

    // Map to the correct int
    switch (obstacle_type_map[node_->get_parameter("objects." + name + ".type").as_string()]) {
      case 1:
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

  auto parent_node = std::make_shared<SimpleServer>(
    "ur_manipulator",
    node_options);
  rclcpp::spin(parent_node->getNodeBaseInterface());
  rclcpp::shutdown();
  return 0;

}
