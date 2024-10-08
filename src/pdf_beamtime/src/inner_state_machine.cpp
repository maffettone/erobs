/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pdf_beamtime/inner_state_machine.hpp>

using namespace std::chrono_literals;

InnerStateMachine::InnerStateMachine(
  const rclcpp::Node::SharedPtr node, const rclcpp::Node::SharedPtr gripper_node)
: node_(node), gripper_node_(gripper_node)
{
  internal_state_enum_ = Internal_State::RESTING;

  // Create gripper client
  gripper_client_ =
    gripper_node_->create_client<pdf_beamtime_interfaces::srv::GripperControlMsg>(
    "gripper_service");
}

moveit::core::MoveItErrorCode InnerStateMachine::move_robot(
  moveit::planning_interface::MoveGroupInterface & mgi, std::vector<double> joint_goal)
{
  moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;
  joint_goal_ = joint_goal;

  switch (internal_state_enum_) {
    case Internal_State::RESTING:
    case Internal_State::CLEANUP: {
        mgi.setJointValueTarget(joint_goal_);
        // Create a plan to that target pose
        auto const [planing_success, plan] = [&mgi] {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto const ok = static_cast<bool>(mgi.plan(plan));
            return std::make_pair(ok, plan);
          }();
        if (planing_success) {
          // Change inner state to Moving if the robot is ready to move and not on clean up
          if (internal_state_enum_ == Internal_State::RESTING) {
            set_internal_state(Internal_State::MOVING);
          }
          auto exec_results = mgi.execute(plan);
          return_error_code = exec_results;
        } else {
          return_error_code = moveit::core::MoveItErrorCode::FAILURE;
        }
      }
      break;

    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Robot's current internal state is %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return_error_code = moveit::core::MoveItErrorCode::FAILURE;
      break;
  }

  return return_error_code;
}

moveit::core::MoveItErrorCode InnerStateMachine::move_robot_cartesian(
  moveit::planning_interface::MoveGroupInterface & mgi,
  std::vector<geometry_msgs::msg::Pose> target_pose)
{
  moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;

  switch (internal_state_enum_) {
    case Internal_State::RESTING:
    case Internal_State::CLEANUP: {
        // Create a plan to that target pose
        auto const [planing_success, plan] = [&mgi, target_pose] {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            // path_achieved_fraction, between 0.0 and 1.0, indicating the fraction of the path
            // achieved as described by the waypoints. Return -1.0 in case of error.
            double path_achieved_fraction = mgi.computeCartesianPath(
              target_pose, 0.005, 0.0,
              plan.trajectory_);
            return std::make_pair(path_achieved_fraction, plan);
          }();
        if (1.0 - planing_success < 0.000001) {
          // Change inner state to Moving if the robot is ready to move and not on clean up
          if (internal_state_enum_ == Internal_State::RESTING) {
            set_internal_state(Internal_State::MOVING);
          }
          auto exec_results = mgi.execute(plan);
          return_error_code = exec_results;
        } else {
          return_error_code = moveit::core::MoveItErrorCode::FAILURE;
        }
      }
      break;

    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Robot's current internal state is %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return_error_code = moveit::core::MoveItErrorCode::FAILURE;
      break;
  }

  return return_error_code;
}

moveit::core::MoveItErrorCode InnerStateMachine::close_gripper()
{
  moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;
  switch (internal_state_enum_) {
    case Internal_State::RESTING:
    case Internal_State::CLEANUP: {
        auto request = std::make_shared<pdf_beamtime_interfaces::srv::GripperControlMsg::Request>();
        request->command = "CLOSE";
        request->grip = 100;

        if (!gripper_client_->wait_for_service(10s)) {
          return_error_code = moveit::core::MoveItErrorCode::FAILURE;
          break;
        } else {
          set_internal_state(Internal_State::MOVING);
          auto result = gripper_client_->async_send_request(request);
          if (rclcpp::spin_until_future_complete(gripper_node_, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper open: %d", result.get()->results);
            std::this_thread::sleep_for(3s);
            return_error_code = moveit::core::MoveItErrorCode::SUCCESS;
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            return_error_code = moveit::core::MoveItErrorCode::FAILURE;
          }
        }
      }
      break;

    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Robot's current internal state is %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return_error_code = moveit::core::MoveItErrorCode::FAILURE;
      break;
  }
  return return_error_code;
}

moveit::core::MoveItErrorCode InnerStateMachine::open_gripper()
{
  moveit::core::MoveItErrorCode return_error_code = moveit::core::MoveItErrorCode::FAILURE;

  switch (internal_state_enum_) {
    case Internal_State::RESTING:
    case Internal_State::CLEANUP: {
        auto request = std::make_shared<pdf_beamtime_interfaces::srv::GripperControlMsg::Request>();
        request->command = "OPEN";
        request->grip = 100;

        if (!gripper_client_->wait_for_service(10s)) {
          return_error_code = moveit::core::MoveItErrorCode::FAILURE;
          break;
        } else {
          set_internal_state(Internal_State::MOVING);
          auto result = gripper_client_->async_send_request(request);
          if (rclcpp::spin_until_future_complete(gripper_node_, result) ==
            rclcpp::FutureReturnCode::SUCCESS)
          {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper open: %d", result.get()->results);
            std::this_thread::sleep_for(3s);
            return_error_code = moveit::core::MoveItErrorCode::SUCCESS;
          } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
            return_error_code = moveit::core::MoveItErrorCode::FAILURE;
          }
        }
      }
      break;

    default:
      RCLCPP_ERROR(
        node_->get_logger(), "Robot's current internal state is %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      return_error_code = moveit::core::MoveItErrorCode::FAILURE;
      break;
  }
  return return_error_code;
}

void InnerStateMachine::pause(moveit::planning_interface::MoveGroupInterface & mgi)
{
  switch (internal_state_enum_) {
    case Internal_State::RESTING:
      RCLCPP_INFO(
        node_->get_logger(), "Paused while at internal state %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      set_internal_state(Internal_State::PAUSED);
      break;

    case Internal_State::MOVING:
      mgi.stop();
      RCLCPP_INFO(
        node_->get_logger(), "Paused while at internal state %s ",
        internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
      set_internal_state(Internal_State::PAUSED);
      break;
    default:
      break;
  }
}

void InnerStateMachine::abort(moveit::planning_interface::MoveGroupInterface & mgi)
{
  mgi.stop();
  RCLCPP_INFO(
    node_->get_logger(), "Stopped while at internal state %s ",
    internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
  set_internal_state(Internal_State::ABORT);
}

void InnerStateMachine::halt(moveit::planning_interface::MoveGroupInterface & mgi)
{
  mgi.stop();
  RCLCPP_INFO(
    node_->get_logger(), "Halted while at internal state %s ",
    internal_state_names[static_cast<int>(internal_state_enum_)].c_str());
  set_internal_state(Internal_State::HALT);
}

void InnerStateMachine::rewind()
{
  if (internal_state_enum_ == Internal_State::PAUSED) {
    set_internal_state(Internal_State::RESTING);
  }
}

void InnerStateMachine::set_internal_state(Internal_State state)
{
  RCLCPP_INFO(
    node_->get_logger(), "Internal state changed from %s to %s ",
    internal_state_names[static_cast<int>(internal_state_enum_)].c_str(),
    internal_state_names[static_cast<int>(state)].c_str());
  internal_state_enum_ = state;
}

Internal_State InnerStateMachine::get_internal_state()
{
  return internal_state_enum_;
}
