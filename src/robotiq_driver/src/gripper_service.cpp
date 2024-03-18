// constexpr auto kComPort = "/tmp/ttyUR";
// constexpr auto kSlaveID = 0x09;

#include <robotiq_driver/gripper_service.hpp>


GripperService::GripperService()
{
  RobotiqGripperInterface gripper(kComPort, kSlaveID);
}

void gripper_controller(
  const std::shared_ptr<custom_msgs::srv::GripperCmd::Request> request,
  std::shared_ptr<custom_msgs::srv::GripperCmd::Response> response)
{

  // This function sends the gripper control command
  bool status = false;
  try {
    char cmd = request->cmd;

    switch (cmd) {
      case 'A':
        // Activate the gripper
        gripper.deactivateGripper();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        gripper.activateGripper();
        RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Activation successful");

        break;

      case 'D':
        // Deactivate the gripper
        gripper.deactivateGripper();
        RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Deactivated");

        break;

      case 'M':
        {
          // Closes the gripper to the percentage set by request->grip
          uint8_t val = request->grip * 2.55; // convert the scales from 01-100 to 0-255
          // std::cout << "######### request and val : " << std::endl ;
          // std::cout << static_cast<int16_t>(request->grip) << std::endl ;
          // std::cout << static_cast<int16_t>(val) << std::endl ;
          gripper.setGripperPosition(val);
          RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Gripper Open");
        }
        break;

      case 'O':
        /* Open the grippper fully */
        gripper.setGripperPosition(0x00);
        RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Gripper Open");
        break;

      case 'C':
        /* Close the grippper fully */
        gripper.setGripperPosition(0xFF);
        RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Gripper Open");
        break;

      // case 'S':
      //   /* Close the grippper fully */
      //   gripper.setGripperPosition(0xFF);
      //   break;

      default:
        break;
    }

    status = true;
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    status = false;
  }

  // Send the response back
  response->status = status;

}

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gripper_service");   // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Activate the gripper ...");
  // Clear the registers
  gripper.deactivateGripper();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  // Activate the gripper
  gripper.activateGripper();

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  gripper.setSpeed(0x0F);

  RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Activation successful");

  rclcpp::Service<custom_msgs::srv::GripperCmd>::SharedPtr service =
    node->create_service<custom_msgs::srv::GripperCmd>("gripper_service", &gripper_controller);                                                                     // CHANGE

  RCLCPP_INFO(rclcpp::get_logger("gripper_controller"), "Ready to recieve gripper commands.");                     // CHANGE

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;

}
