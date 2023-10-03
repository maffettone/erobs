/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
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
#include <moveit/move_group_interface/move_group_interface.h>
#include <yaml-cpp/yaml.h>
#include <hello_moveit_interfaces/action/pick_place_repeat.hpp>

using namespace std::chrono_literals;

// This only adds the obstacle environment. This should be changed to have a header in the future iterations

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

    // Add the obstacles
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    planning_scene_interface->applyCollisionObjects(create_env());
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
  // Expose the node base interface so that the node can be added to a component manager.
  {
    return node_->get_node_base_interface();
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;

  moveit::planning_interface::PlanningSceneInterface * planning_scene_interface;
  std::map<std::string, int> obstacle_type_map;


  std::vector<moveit_msgs::msg::CollisionObject> create_env()
  // Builds a vector of obstacle as defined in the .yaml file and returns the vector
  {
    obstacle_type_map.insert(std::pair<std::string, int>("CYLINDER", 1));
    obstacle_type_map.insert(std::pair<std::string, int>("BOX", 2));

    // int num_objects = node_->get_parameter("num_objects").as_int();
    std::vector<std::string> object_names = node_->get_parameter("object_names").as_string_array();

    std::vector<moveit_msgs::msg::CollisionObject> all_obstacles;

    // Create objects in a recursion
    for (size_t i = 0; i < object_names.size(); i++) {
      std::string name = object_names[i];  //get each name here as it uses as a parameter field

      moveit_msgs::msg::CollisionObject obj;  // collision object
      geometry_msgs::msg::Pose pose;  // object pose
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
