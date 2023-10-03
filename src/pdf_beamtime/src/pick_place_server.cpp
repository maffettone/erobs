/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <moveit/move_group_interface/move_group_interface.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <hello_moveit_interfaces/action/pick_place_repeat.hpp>

using namespace std::chrono_literals;

/*
This Action server cannot use the plugin approaches, because it is not a node with standard initialization.
Instead of following the tutorials here directl we need to spin a node in a main function.
https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html

I.e. using a namespace and RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)
and rclcpp_components_register_nodes will not work. It will create an compilation error:
/usr/include/c++/11/ext/new_allocator.h:162:11: \
error: no matching function for call to ‘pick_place_repeat_server::PickPlaceServer::PickPlaceServer(const rclcpp::NodeOptions&)’
{ ::new((void *)__p) _Up(std::forward<_Args>(__args)...); }
*/
class PickPlaceServer
{
public:
  using PickPlaceRepeat = hello_moveit_interfaces::action::PickPlaceRepeat;
  explicit PickPlaceServer(
    const std::string & move_group_name = "ur_manipulator",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : node_(std::make_shared<rclcpp::Node>("pick_place_server", options)),
    move_group_interface_(node_, move_group_name)
  {
    using namespace std::placeholders;
    using namespace std::chrono_literals;

    // Create parameters for list of waypoints
    RCLCPP_INFO(node_->get_logger(), "Not declaring parameters for waypoints");

    RCLCPP_INFO(node_->get_logger(), "Reading parameters for waypoints");
    home_pose_ = node_->get_parameter("home_pose").as_double_array();

    // Read in travel waypoints as Poses from config/waypoints.yaml file
    auto const waypoints_file = node_->get_parameter("waypoints_file").as_string();
    travel_waypoints = read_waypoints(waypoints_file);

    // Add the obstacles
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    planning_scene_interface->applyCollisionObjects(create_env());


    action_server_ = rclcpp_action::create_server<PickPlaceRepeat>(
      node_,
      "pick_place_repeat",
      std::bind(&PickPlaceServer::handle_goal, this, _1, _2),
      std::bind(&PickPlaceServer::handle_cancel, this, _1),
      std::bind(&PickPlaceServer::handle_accepted, this, _1));
    RCLCPP_INFO(node_->get_logger(), "PickPlaceServer has been initialized.");
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
  // Expose the node base interface so that the node can be added to a component manager.
  {
    return node_->get_node_base_interface();
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  rclcpp_action::Server<PickPlaceRepeat>::SharedPtr action_server_;
  int num_repeats_;
  int total_steps_ = 2 * 8;
  std::vector<geometry_msgs::msg::Pose> travel_waypoints;
  std::vector<double> home_pose_;

  moveit::planning_interface::PlanningSceneInterface * planning_scene_interface;
  std::map<std::string, int> obstacle_type_map;

  std::vector<geometry_msgs::msg::Pose> read_waypoints(const std::string & filename)
  {
    /* Function to read in a series of poses from a yaml file*/
    std::vector<geometry_msgs::msg::Pose> waypoints;
    RCLCPP_INFO(node_->get_logger(), "Reading waypoints from %s", filename.c_str());

    std::ifstream file(filename);
    YAML::Node yaml_node = YAML::Load(file);
    for (auto const & pose : yaml_node) {
      geometry_msgs::msg::Pose msg;
      const YAML::Node & poseNode = pose["pose"];
      msg.position.x = poseNode["position"]["x"].as<double>();
      msg.position.y = poseNode["position"]["y"].as<double>();
      msg.position.z = poseNode["position"]["z"].as<double>();
      msg.orientation.x = poseNode["orientation"]["x"].as<double>();
      msg.orientation.y = poseNode["orientation"]["y"].as<double>();
      msg.orientation.z = poseNode["orientation"]["z"].as<double>();
      msg.orientation.w = poseNode["orientation"]["w"].as<double>();
      waypoints.push_back(msg);
    }
    return waypoints;
  }

  // Receiving the goal request. Simply asking for, do the thing several times.
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickPlaceRepeat::Goal> goal)
  {
    RCLCPP_INFO(node_->get_logger(), "Received goal request with order %d", goal->repeats);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Receiving the cancel request.
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceRepeat>> goal_handle)
  {
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Spawn a thread to actually do the work.
  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceRepeat>> goal_handle)
  {
    using namespace std::placeholders;

    // Start the thread to execute the action
    std::thread{std::bind(&PickPlaceServer::execute, this, _1), goal_handle}.detach();
  }

  // Helper function to plan and execute in joint space
  bool plan_and_execute_joint_target(const std::vector<double> & angles)
  // TODO(maffettone): take the goal handle and add cancellation infrastructure
  {
    move_group_interface_.setJointValueTarget(angles);
    // Create a plan to that target pose
    auto const [success, plan] = [this] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(this->move_group_interface_.plan(msg));
        return std::make_pair(ok, msg);
      }();
    // Execute the plan
    if (success) {
      move_group_interface_.execute(plan);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
    }
    return success;
  }


  // Helper function to plan and execute based on pose
  bool plan_and_execute_pose(const geometry_msgs::msg::Pose & pose)
  // TODO(maffettone): take the goal handle and add cancellation infrastructure
  {
    move_group_interface_.setPoseTarget(pose);
    // Create a plan to that target pose
    auto const [success, plan] = [this] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(this->move_group_interface_.plan(msg));
        return std::make_pair(ok, msg);
      }();
    // Execute the plan
    if (success) {
      move_group_interface_.execute(plan);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed!");
    }
    return success;
  }

  // The actual work. Perform the movement between waypoints several times, tracking progress.
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceRepeat>> goal_handle)
  {
    RCLCPP_INFO(node_->get_logger(), "Executing goal");

    // Helper variables
    const auto goal = goal_handle->get_goal();
    this->num_repeats_ = goal->repeats;
    auto feedback = std::make_shared<PickPlaceRepeat::Feedback>();
    auto & num_completed = feedback->num_completed;
    auto & percent_completed = feedback->percent_completed;
    auto & percentage_current = feedback->percentage_current;
    auto result = std::make_shared<PickPlaceRepeat::Result>();
    bool success = true;
    auto loop_rate = rclcpp::Rate(3s);


    // Perform the movement between waypoints several times, tracking progress.
    for (num_completed = 0; num_completed < num_repeats_; ++num_completed) {
      percent_completed = static_cast<float>(num_completed) / num_repeats_;
      percentage_current = 0.0;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(node_->get_logger(), "Completed %d of %d repeats", num_completed, num_repeats_);
      // Move to home pose
      RCLCPP_INFO(node_->get_logger(), "Moving to home pose in joint space");

      success = plan_and_execute_joint_target(home_pose_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);

      // Sleep for a while
      loop_rate.sleep();
    }

    // Send the result
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
    }
  }

  std::vector<moveit_msgs::msg::CollisionObject> create_env()
  {
    obstacle_type_map.insert(std::pair<std::string, int>("CYLINDER", 1));
    obstacle_type_map.insert(std::pair<std::string, int>("BOX", 2));

    // int num_objects = node_->get_parameter("num_objects").as_int();
    std::vector<std::string> object_names = node_->get_parameter("object_names").as_string_array();

    std::vector<moveit_msgs::msg::CollisionObject> all_objects;

    // Create objects in a recursion
    for (size_t i = 0; i < object_names.size(); i++) {

      std::string name = object_names[i]; //get each name here as it uses as a parameter field

      moveit_msgs::msg::CollisionObject obj; // collision object
      geometry_msgs::msg::Pose pose; // object pose
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

      all_objects.push_back(obj);
    }
    return all_objects;
  }

};   // class PickPlaceServer

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create a ROS logger for main scope
  auto const logger = rclcpp::get_logger("pick_place_server");

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
      "robot_description",
      "robot_description_kinematics"});

  // Set node parameters using NodeOptions
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides(
  {
    {"robot_description_semantic", parameters[0].value_to_string()},
    {"robot_description", parameters[1].value_to_string()},
    {"robot_description_kinematics", parameters[2].value_to_string()}
  });

  auto parent_node = std::make_shared<PickPlaceServer>(
    "ur_manipulator",
    node_options);
  rclcpp::spin(parent_node->getNodeBaseInterface());
  rclcpp::shutdown();
  return 0;
}
