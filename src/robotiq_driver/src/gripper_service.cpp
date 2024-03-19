#include <robotiq_driver/gripper_service.hpp>

using namespace std::placeholders;

GripperService::GripperService()
: node_(std::make_shared<rclcpp::Node>("gripper_server_node")),
  gripper_(kComPort, kSlaveID)
{
  RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Activate the gripper ...");
  // Clear the registers
  gripper_.deactivateGripper();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // Activate the gripper
  gripper_.activateGripper();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  gripper_.setSpeed(0x0F);

  RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Activation successful");

  rclcpp::Service<pdf_beamtime_interfaces::srv::GripperControlMsg>::SharedPtr service =
    node_->create_service<pdf_beamtime_interfaces::srv::GripperControlMsg>(
    "gripper_service",
    std::bind(
      &GripperService::gripper_controller, this, _1, _2));                                                                                                                                                             // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Ready to receive gripper commands.");                     // CHANGE

}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr GripperService::getNodeBaseInterface()
// Expose the node base interface so that the node can be added to a component manager.
{
  return node_->get_node_base_interface();
}

void GripperService::gripper_controller(
  const std::shared_ptr<pdf_beamtime_interfaces::srv::GripperControlMsg::Request> request,
  std::shared_ptr<pdf_beamtime_interfaces::srv::GripperControlMsg::Response> response)
{
  // This function sends the gripper control command
  // moveit::core::MoveItErrorCode gripper_results = moveit::core::MoveItErrorCode::FAILURE;

  int status = 0;
  try {
    // conver the request command string to the mapping enum
    Gripper_State gripper_command_enum = gripper_command_map_[request->command];

    switch (gripper_command_enum) {
      case Gripper_State::ACTIVE:
        // Activate the gripper
        gripper_.deactivateGripper();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        gripper_.activateGripper();
        RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Activation successful");

        break;

      case Gripper_State::DEACTIVE:
        // Deactivate the gripper
        gripper_.deactivateGripper();
        RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Deactivated");

        break;

      case Gripper_State::PARTIAL:
        {
          // Closes the gripper to the percentage set by request->grip
          uint8_t val = request->grip * 2.55; // convert the scales from 01-100 to 0-255
          // std::cout << "######### request and val : " << std::endl ;
          // std::cout << static_cast<int16_t>(request->grip) << std::endl ;
          // std::cout << static_cast<int16_t>(val) << std::endl ;
          gripper_.setGripperPosition(val);
          RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Gripper Open");
        }
        break;

      case Gripper_State::OPEN:
        /* Open the grippper fully */
        gripper_.setGripperPosition(0x00);
        RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Gripper Open");
        break;

      case Gripper_State::CLOSE:
        /* Close the grippper fully */
        gripper_.setGripperPosition(0xFF);
        RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Gripper Open");
        break;

      default:
        break;
    }

    status = 1;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    status = 0;
  }

  // Send the response back
  response->results = status;

}

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  auto gripper_server = std::make_shared<GripperService>();

  // std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gripper_service");   // CHANGE

  rclcpp::spin(gripper_server->getNodeBaseInterface());
  rclcpp::shutdown();

  return 0;

}
