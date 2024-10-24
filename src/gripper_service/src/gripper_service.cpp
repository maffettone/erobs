/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <gripper_service/gripper_service.hpp>

using namespace std::placeholders;

GripperService::GripperService()
: Node("gripper_server_node"),
  gripper_(kComPort, kSlaveID)
{
  RCLCPP_INFO(this->get_logger(), "Activate the gripper ...");
  // Clear the registers
  gripper_.deactivateGripper();

  // Activate the gripper
  gripper_.activateGripper();
  gripper_.setSpeed(0x0F);

  RCLCPP_INFO(this->get_logger(), "Activation is successful");

  service =
    this->create_service<pdf_beamtime_interfaces::srv::GripperControlMsg>(
    "gripper_service",
    std::bind(
      &GripperService::gripper_controller, this, _1, _2));

  // Check if the service was created successfully
  if (service == nullptr) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create service");
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(this->get_logger(), "Service %s created successfully", service->get_service_name());
    RCLCPP_INFO(this->get_logger(), "Ready to receive gripper commands.");
  }
}

void GripperService::gripper_controller(
  const std::shared_ptr<pdf_beamtime_interfaces::srv::GripperControlMsg::Request> request,
  std::shared_ptr<pdf_beamtime_interfaces::srv::GripperControlMsg::Response> response)
{
  int status = 0;
  try {
    //  conver the request command string to the mapping enum
    Gripper_Command gripper_command_enum = gripper_command_map_[request->command];

    switch (gripper_command_enum) {
      case Gripper_Command::ACTIVE:
        // Activate the gripper
        gripper_.deactivateGripper();
        gripper_.activateGripper();
        RCLCPP_INFO(this->get_logger(), "Activation is successful");

        break;

      case Gripper_Command::DEACTIVE:
        // Deactivate the gripper
        gripper_.deactivateGripper();
        RCLCPP_INFO(this->get_logger(), "Gripper is Deactivated");

        break;

      case Gripper_Command::PARTIAL:
        {
          //  Closes the gripper to the percentage set by request->grip
          uint8_t val = request->grip * 2.55;  // convert the scales from 01-100 to 0-255
          gripper_.setGripperPosition(val);
          RCLCPP_INFO(this->get_logger(), "Gripper is Open");
        }
        break;

      case Gripper_Command::OPEN:
        //  Open the gripper fully
        gripper_.setGripperPosition(0x00);
        RCLCPP_INFO(this->get_logger(), "Gripper is Open");
        break;

      case Gripper_Command::CLOSE:
        //  Close the gripper fully
        gripper_.setGripperPosition(0xFF);
        RCLCPP_INFO(this->get_logger(), "Gripper is Close");
        break;

      default:
        break;
    }
    status = 1;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    status = 0;
  }

  //  Send the response back
  response->results = status;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto gripper_server_node = std::make_shared<GripperService>();

  rclcpp::spin(gripper_server_node);
  rclcpp::shutdown();

  return 0;
}
