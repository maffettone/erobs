/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#ifndef PDF_BEAMTIME__PDF_BEAMTIME_SERVER_HPP_
#define PDF_BEAMTIME__PDF_BEAMTIME_SERVER_HPP_

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <pdf_beamtime_interfaces/action/pick_place_control_msg.hpp>
#include <pdf_beamtime_interfaces/srv/new_obstacle_msg.hpp>
#include <pdf_beamtime_interfaces/srv/update_obstacle_msg.hpp>

/// @brief Create the obstacle environment and an simple action server for the robot to move
class PdfBeamtimeServer
{
public:
  using PickPlaceControlMsg = pdf_beamtime_interfaces::action::PickPlaceControlMsg;
  using NewObstacleMsg = pdf_beamtime_interfaces::srv::NewObstacleMsg;
  using UpdateObstaclesMsg = pdf_beamtime_interfaces::srv::UpdateObstacleMsg;

  explicit PdfBeamtimeServer(
    const std::string & move_group_name, const rclcpp::NodeOptions & options,
    std::string action_name);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  /// @brief records the two types of obstacles CYLINDER and BOX.
  std::map<std::string, int> obstacle_type_map_;

  rclcpp::Service<NewObstacleMsg>::SharedPtr env_refresh_service_;
  rclcpp::Service<UpdateObstaclesMsg>::SharedPtr update_obstacles_service_;

  /// @brief Pointer to the action server
  rclcpp_action::Server<PickPlaceControlMsg>::SharedPtr action_server_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

  // Action server related callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickPlaceControlMsg::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceControlMsg>> goal_handle);

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceControlMsg>> goal_handle);

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceControlMsg>> goal_handle);

  /// @brief generates a vector of obstacles from a yaml file.
  /// @return a vector of CollisionObjects
  std::vector<moveit_msgs::msg::CollisionObject> create_env();

  /// @brief Callback for adding a new obstacle
  /// @param request a NewObstacleMsg
  /// @param response Success / Failure
  void new_obstacle_service_cb(
    const std::shared_ptr<NewObstacleMsg::Request> request,
    std::shared_ptr<NewObstacleMsg::Response> response);

  /// @brief Callback for changing the value of an existing obstacle
  /// @param request UpdateObstaclesMsg
  /// @param response Success / Failure
  void update_obstacles_service_cb(
    const std::shared_ptr<UpdateObstaclesMsg::Request> request,
    std::shared_ptr<UpdateObstaclesMsg::Response> response);
};

#endif  // PDF_BEAMTIME__PDF_BEAMTIME_SERVER_HPP_
