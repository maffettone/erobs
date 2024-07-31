/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#pragma once

#include <pdf_beamtime/pdf_beamtime_server.hpp>

/// @brief Create the obstacle environment and an simple action server for the robot to move
class PdfBeamtimeFidPoseServer : public PdfBeamtimeServer
{
public:
  explicit PdfBeamtimeFidPoseServer(
    const std::string & move_group_name, const rclcpp::NodeOptions & options,
    std::string action_name);

protected:
  // void handle_accepted(
  //   const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceControlMsg>> goal_handle);
  // rclcpp_action::GoalResponse handle_goal(
  //   const rclcpp_action::GoalUUID & uuid,
  //   std::shared_ptr<const PickPlaceControlMsg::Goal> goal);
  // rclcpp_action::CancelResponse handle_cancel(
  //   const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceControlMsg>> goal_handle);

  moveit::core::MoveItErrorCode run_fsm(
    std::shared_ptr<const pdf_beamtime_interfaces::action::PickPlaceControlMsg_Goal> goal);

//   std::vector<std::string> external_state_names_ =
//   {"HOME", "PICKUP_APPROACH", "PICKUP", "GRASP_SUCCESS", "GRASP_FAILURE", "PICKUP_RETREAT",
//     "PLACE_APPROACH", "PLACE", "RELEASE_SUCCESS", "RELEASE_FAILURE", "PLACE_RETREAT"};

//   std::vector<std::string> internal_state_names =
//   {"RESTING", "MOVING", "PAUSED", "ABORT", "HALT", "STOP", "CLEANUP"};

//   /// @brief current state of the robot
//   State current_state_;

//   /// @brief holds the current goal to be used by return_sample() function
//   std::shared_ptr<const pdf_beamtime_interfaces::action::PickPlaceControlMsg_Goal> goal;

//   /// @brief used to calculate the completion precentage
//   const float total_states_ = 9.0;
//   float progress_ = 0.0;

// /// @todo @ChandimaFernando Implement to see if gripper is attached
//   bool gripper_present_ = false;

//   // Action server related callbacks
//   rclcpp_action::GoalResponse handle_goal(
//     const rclcpp_action::GoalUUID & uuid,
//     std::shared_ptr<const PickPlaceControlMsg::Goal> goal);

//   rclcpp_action::CancelResponse handle_cancel(
//     const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceControlMsg>> goal_handle);

//   void handle_accepted(
//     const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceControlMsg>> goal_handle);

//   virtual void execute(
//     const std::shared_ptr<rclcpp_action::ServerGoalHandle<PickPlaceControlMsg>> goal_handle);

//   /// @brief generates a vector of obstacles from a yaml file.
//   /// @return a vector of CollisionObjects
//   std::vector<moveit_msgs::msg::CollisionObject> create_env();

//   /// @brief Callback for changing the value of an existing obstacle
//   /// @param request UpdateObstaclesMsg
//   /// @param response Success / Failure
//   void update_obstacles_service_cb(
//     const std::shared_ptr<UpdateObstaclesMsg::Request> request,
//     std::shared_ptr<UpdateObstaclesMsg::Response> response);

//   /// @brief Callback to remove an existing obstacle
//   /// @param request DeleteObstacleMsg
//   /// @param response Success / Failure
//   void remove_obstacles_service_cb(
//     const std::shared_ptr<DeleteObstacleMsg::Request> request,
//     std::shared_ptr<DeleteObstacleMsg::Response> response);

//   /// @brief Callback to handle interrupts frm bluesky
//   /// @param request type of interrupt: int
//   /// @param response Success / Failure : bool
//   void bluesky_interrupt_cb(
//     const std::shared_ptr<BlueskyInterruptMsg::Request> request,
//     std::shared_ptr<BlueskyInterruptMsg::Response> response);

//   /// @brief Callback for adding a new obstacle
//   /// @param request a CylinderObstacleMsg or BoxObstacleMsg
//   /// @param response Success / Failure
//   template<typename RequestT, typename ResponseT>
//   void new_obstacle_service_cb(
//     const typename RequestT::SharedPtr request,
//     typename ResponseT::SharedPtr response);

//   /// @brief Set the current state to the next state
//   float get_action_completion_percentage();

//   /// @brief Performs the transitions for each State
//   moveit::core::MoveItErrorCode run_fsm(
//     std::shared_ptr<const pdf_beamtime_interfaces::action::PickPlaceControlMsg_Goal> goal);

//   /// @brief Set the current state to HOME and move robot to home position
//   bool reset_fsm();

//   /// @brief Put back the sample
//   moveit::core::MoveItErrorCode return_sample();

//   /// @brief Handles bluesky interrupt to PAUSE, STOP, ABORT, and HALT
//   void handle_pause();
//   void handle_stop();
//   /// @brief returns the sample to where it was picked and ready robot to receive a new goal
//   void execute_cleanup();
//   void handle_abort();
//   void handle_halt();

//   /// @brief Handles bluesky interrupt to RESUME
//   void handle_resume();

//   /// @brief change the current state here
//   void set_current_state(State state);
};
