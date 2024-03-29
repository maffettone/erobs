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

#include <hello_moveit_interfaces/action/pick_place_repeat.hpp>

using namespace std::chrono_literals;

/*
This Action server cannot use the plugin approaches, because it is not a node with standard initialization.
Instead of following the tutorials here directl we need to spin a node in a main function.
https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html

I.e. using a namespace and RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)
and rclcpp_components_register_nodes will not work. It will create an compilation error:
/usr/include/c++/11/ext/new_allocator.h:162:11: \
error: no matching function for call to ‘pick_place_repeat_server::PickPlaceRepeatServer::PickPlaceRepeatServer(const rclcpp::NodeOptions&)’
{ ::new((void *)__p) _Up(std::forward<_Args>(__args)...); }
*/
class PickPlaceRepeatServer
{
public:
  using PickPlaceRepeat = hello_moveit_interfaces::action::PickPlaceRepeat;
  explicit PickPlaceRepeatServer(
    const std::string & move_group_name = "ur_manipulator",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : node_(std::make_shared<rclcpp::Node>("pick_place_repeat_server", options)),
    move_group_interface_(node_, move_group_name)
  {
    using namespace std::placeholders;
    using namespace std::chrono_literals;

    // Create parameters for list of waypoints
    RCLCPP_INFO(node_->get_logger(), "Not declaring parameters for waypoints");
    // Parameters need not be declared because of Moveit Node Options
    // node_->declare_parameter<std::vector<double>>("pickup_approach");
    // node_->declare_parameter<std::vector<double>>("pickup_grasp");
    // node_->declare_parameter<std::vector<double>>("pickup_retreat");
    // node_->declare_parameter<std::vector<double>>("dropoff_approach");
    // node_->declare_parameter<std::vector<double>>("dropoff_grasp");
    // node_->declare_parameter<std::vector<double>>("dropoff_retreat");
    // node_->declare_parameter<std::vector<double>>("home_pose");
    // node_->declare_parameter<std::string>("waypoints_file");
    RCLCPP_INFO(node_->get_logger(), "Reading parameters for waypoints");
    home_pose_ = node_->get_parameter("home_pose").as_double_array();
    pickup_approach_ = node_->get_parameter("pickup_approach").as_double_array();
    pickup_grasp_ = node_->get_parameter("pickup_grasp").as_double_array();
    pickup_retreat_ = node_->get_parameter("pickup_retreat").as_double_array();
    dropoff_approach_ = node_->get_parameter("dropoff_approach").as_double_array();
    dropoff_grasp_ = node_->get_parameter("dropoff_grasp").as_double_array();
    dropoff_retreat_ = node_->get_parameter("dropoff_retreat").as_double_array();

    // Read in travel waypoints as Poses from config/waypoints.yaml file
    auto const waypoints_file = node_->get_parameter("waypoints_file").as_string();
    travel_waypoints = read_waypoints(waypoints_file);


    action_server_ = rclcpp_action::create_server<PickPlaceRepeat>(
      node_,
      "pick_place_repeat",
      std::bind(&PickPlaceRepeatServer::handle_goal, this, _1, _2),
      std::bind(&PickPlaceRepeatServer::handle_cancel, this, _1),
      std::bind(&PickPlaceRepeatServer::handle_accepted, this, _1));
    RCLCPP_INFO(node_->get_logger(), "PickPlaceRepeatServer has been initialized.");
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
  std::vector<double> pickup_approach_;
  std::vector<double> pickup_grasp_;
  std::vector<double> pickup_retreat_;
  std::vector<double> dropoff_approach_;
  std::vector<double> dropoff_grasp_;
  std::vector<double> dropoff_retreat_;


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
    std::thread{std::bind(&PickPlaceRepeatServer::execute, this, _1), goal_handle}.detach();
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

      // Move to pickup approach pose
      RCLCPP_INFO(node_->get_logger(), "Moving to pickup approach pose");
      success = plan_and_execute_joint_target(pickup_approach_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);


      // Move to pickup grasp pose
      RCLCPP_INFO(node_->get_logger(), "Moving to pickup grasp pose");
      success = plan_and_execute_joint_target(pickup_grasp_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);

      RCLCPP_WARN(node_->get_logger(), "TODO: Add gripper close action here");

      // Move to pickup retreat pose
      RCLCPP_INFO(node_->get_logger(), "Moving to pickup retreat pose");
      success = plan_and_execute_joint_target(pickup_retreat_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);

      // Move to each waypoint in turn

      for (auto const & waypoint : this->travel_waypoints) {
        success = plan_and_execute_pose(waypoint);
      }
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);

      // Move to dropoff approach pose
      RCLCPP_INFO(node_->get_logger(), "Moving to dropoff approach pose");
      success = plan_and_execute_joint_target(dropoff_approach_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);

      // Move to dropoff grasp pose
      RCLCPP_INFO(node_->get_logger(), "Moving to dropoff grasp pose");
      success = plan_and_execute_joint_target(dropoff_grasp_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);

      RCLCPP_WARN(node_->get_logger(), "TODO: Add gripper open action here");

      // Move to dropoff retreat pose
      RCLCPP_INFO(node_->get_logger(), "Moving to dropoff grasp pose");
      success = plan_and_execute_joint_target(dropoff_retreat_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);

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


      // Repeat this whole thing in reverse
      // Move to dropoff retreat pose
      RCLCPP_INFO(node_->get_logger(), "Moving to dropoff retreat pose");
      success = plan_and_execute_joint_target(dropoff_retreat_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);
      // Move to dropoff grasp pose
      RCLCPP_INFO(node_->get_logger(), "Moving to dropoff grasp pose");
      success = plan_and_execute_joint_target(dropoff_grasp_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);

      RCLCPP_WARN(node_->get_logger(), "TODO: Add gripper close action here");

      // Move to dropoff approach pose
      RCLCPP_INFO(node_->get_logger(), "Moving to dropoff approach pose");
      success = plan_and_execute_joint_target(dropoff_approach_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);
      // Move to each waypoint in reverse order
      for (auto it = this->travel_waypoints.rbegin(); it != this->travel_waypoints.rend(); ++it) {
        success = plan_and_execute_pose(*it);
      }
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);
      // Move to pickup retreat pose
      RCLCPP_INFO(node_->get_logger(), "Moving to pickup retreat pose");
      success = plan_and_execute_joint_target(pickup_retreat_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);
      // Move to pickup grasp pose
      RCLCPP_INFO(node_->get_logger(), "Moving to pickup grasp pose");
      success = plan_and_execute_joint_target(pickup_grasp_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);

      RCLCPP_WARN(node_->get_logger(), "TODO: Add gripper open action here");

      // Move to pickup approach pose
      RCLCPP_INFO(node_->get_logger(), "Moving to pickup approach pose");
      success = plan_and_execute_joint_target(pickup_approach_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current += 1.0 / total_steps_;
      goal_handle->publish_feedback(feedback);
      // Move to home pose
      RCLCPP_INFO(node_->get_logger(), "Moving to home pose");
      success = plan_and_execute_joint_target(home_pose_);
      if (!success) {
        result->success = false;
        goal_handle->succeed(result);
        return;
      }
      percentage_current = 1.0;
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
};  // class PickPlaceRepeatServer

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create a ROS logger for main scope
  auto const logger = rclcpp::get_logger("hello_moveit");

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


  auto parent_node = std::make_shared<PickPlaceRepeatServer>(
    "ur_manipulator",
    node_options);
  rclcpp::spin(parent_node->getNodeBaseInterface());
  rclcpp::shutdown();
  return 0;
}
