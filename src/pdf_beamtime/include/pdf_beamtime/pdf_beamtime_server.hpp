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
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <pdf_beamtime_interfaces/action/pick_place_control_msg.hpp>
#include <pdf_beamtime_interfaces/srv/update_obstacle_msg.hpp>
#include <pdf_beamtime_interfaces/srv/delete_obstacle_msg.hpp>
#include <pdf_beamtime_interfaces/srv/box_obstacle_msg.hpp>
#include <pdf_beamtime_interfaces/srv/cylinder_obstacle_msg.hpp>
#include <pdf_beamtime_interfaces/srv/bluesky_interrupt_msg.hpp>
#include <pdf_beamtime/inner_state_machine.hpp>
#include <pdf_beamtime/state_enum.hpp>

/// @brief Create the obstacle environment and an simple action server for the robot to move
class PdfBeamtimeServer
{
public:
  using PickPlaceControlMsg = pdf_beamtime_interfaces::action::PickPlaceControlMsg;
  using BoxObstacleMsg = pdf_beamtime_interfaces::srv::BoxObstacleMsg;
  using CylinderObstacleMsg = pdf_beamtime_interfaces::srv::CylinderObstacleMsg;
  using UpdateObstaclesMsg = pdf_beamtime_interfaces::srv::UpdateObstacleMsg;
  using DeleteObstacleMsg = pdf_beamtime_interfaces::srv::DeleteObstacleMsg;
  using BlueskyInterruptMsg = pdf_beamtime_interfaces::srv::BlueskyInterruptMsg;

  explicit PdfBeamtimeServer(
    const std::string & move_group_name, const rclcpp::NodeOptions & options,
    std::string action_name);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getInterruptNodeBaseInterface();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr interrupt_node_;

  moveit::planning_interface::MoveGroupInterface move_group_interface_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
  /// @brief records the two types of obstacles CYLINDER and BOX.
  std::map<std::string, int> obstacle_type_map_;

  rclcpp::Service<BoxObstacleMsg>::SharedPtr new_box_obstacle_service_;
  rclcpp::Service<CylinderObstacleMsg>::SharedPtr new_cylinder_obstacle_service_;
  rclcpp::Service<UpdateObstaclesMsg>::SharedPtr update_obstacles_service_;
  rclcpp::Service<DeleteObstacleMsg>::SharedPtr remove_obstacles_service_;
  rclcpp::Service<BlueskyInterruptMsg>::SharedPtr bluesky_interrupt_service_;

  /// @brief Pointer to the action server
  rclcpp_action::Server<PickPlaceControlMsg>::SharedPtr action_server_;

  /// @brief Pointer to the inner state machine object
  InnerStateMachine * inner_state_machine_;

  /// @brief records home state
  std::vector<double, std::allocator<double>> goal_home_;

  /// @brief map the Bluesky syntax interrupt command strings to enums
  /// Note the re-use of Internal_state enums for dual purposes.
  std::map<std::string, Internal_State> interrupt_map = {
    {"PAUSE", Internal_State::PAUSED},
    {"STOP", Internal_State::STOP},
    {"HALT", Internal_State::HALT},
    {"ABORT", Internal_State::ABORT},
    {"RESUME", Internal_State::MOVING}
  };

  std::vector<std::string> external_state_names_ =
  {"HOME", "PICKUP_APPROACH", "PICKUP", "GRASP_SUCCESS", "GRASP_FAILURE", "PICKUP_RETREAT",
    "PLACE_APPROACH", "PLACE", "RELEASE_SUCCESS", "RELEASE_FAILURE", "PLACE_RETREAT",
    "RETRY_PICKUP"};

  std::vector<std::string> internal_state_names =
  {"RESTING", "MOVING", "PAUSED", "ABORT", "HALT", "STOP"};

  /// @brief current state of the robot
  State current_state_;
  /// @brief used to calculate the completion precentage
  const float total_states_ = 9.0;
  float progress_ = 0.0;

/// @todo @ChandimaFernando Implement to see if gripper is attached
  bool gripper_present_ = false;

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

  /// @brief Callback for changing the value of an existing obstacle
  /// @param request UpdateObstaclesMsg
  /// @param response Success / Failure
  void update_obstacles_service_cb(
    const std::shared_ptr<UpdateObstaclesMsg::Request> request,
    std::shared_ptr<UpdateObstaclesMsg::Response> response);

  /// @brief Callback to remove an existing obstacle
  /// @param request DeleteObstacleMsg
  /// @param response Success / Failure
  void remove_obstacles_service_cb(
    const std::shared_ptr<DeleteObstacleMsg::Request> request,
    std::shared_ptr<DeleteObstacleMsg::Response> response);

  /// @brief Callback to handle interrupts frm bluesky
  /// @param request type of interrupt: int
  /// @param response Success / Failure : bool
  void bluesky_interrupt_cb(
    const std::shared_ptr<BlueskyInterruptMsg::Request> request,
    std::shared_ptr<BlueskyInterruptMsg::Response> response);

  /// @brief Callback for adding a new obstacle
  /// @param request a CylinderObstacleMsg or BoxObstacleMsg
  /// @param response Success / Failure
  template<typename RequestT, typename ResponseT>
  void new_obstacle_service_cb(
    const typename RequestT::SharedPtr request,
    typename ResponseT::SharedPtr response);

  /// @brief Set the current state to the next state
  float get_action_completion_percentage();

  /// @brief Performs the transitions for each State
  moveit::core::MoveItErrorCode run_fsm(
    std::shared_ptr<const pdf_beamtime_interfaces::action::PickPlaceControlMsg_Goal> goal);

  /// @brief Set the current state to HOME and move robot to home position
  bool reset_fsm();

  /// @brief Handles bluesky interrupt to PAUSE
  void handle_pause();
  void handle_stop();
  void execute_stop();
  void handle_abort();

  /// @brief Handles bluesky interrupt to RESUME
  void handle_resume();

  /// @brief change the current state here
  void set_current_state(State state);
};

#endif  // PDF_BEAMTIME__PDF_BEAMTIME_SERVER_HPP_
