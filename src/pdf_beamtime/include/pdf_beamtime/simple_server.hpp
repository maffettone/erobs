#ifndef SIMPLE_SERVER_HPP
#define SIMPLE_SERVER_HPP

#include <chrono>
#include <fstream>
#include <iostream>
#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <hello_moveit_interfaces/action/simple_action_msg.hpp>

/// @brief Create the obstacle environment and an simple action server for the robot to move
class SimpleServer
{
public:
  using SimpleActionMsg = hello_moveit_interfaces::action::SimpleActionMsg;

  SimpleServer(const std::string & move_group_name, const rclcpp::NodeOptions & options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

private:
  rclcpp::Node::SharedPtr node_;

  /// @brief Pointer to the action server
  rclcpp_action::Server<SimpleActionMsg>::SharedPtr action_server_;

  const std::string ACTION_NAME = "pdf_simple_action";

  moveit::planning_interface::PlanningSceneInterface * planning_scene_interface;
  /// @brief records the two types of obstacles CYLINDER and BOX.
  std::map<std::string, int> obstacle_type_map;

  moveit::planning_interface::MoveGroupInterface * move_group_interface_;

  // Action server related callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SimpleActionMsg::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SimpleActionMsg>> goal_handle);

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<SimpleActionMsg>> goal_handle);
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<SimpleActionMsg>> goal_handle);

  /// @brief generates a vector of obstacles from a yaml file.
  /// @return a vector of CollisionObjects
  std::vector<moveit_msgs::msg::CollisionObject> create_env();

};

#endif
