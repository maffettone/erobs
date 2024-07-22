/*Copyright 2024 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "pdf_beamtime_interfaces/srv/gripper_control_msg.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);


  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("gripper_close");
  rclcpp::Client<pdf_beamtime_interfaces::srv::GripperControlMsg>::SharedPtr client =
    node->create_client<pdf_beamtime_interfaces::srv::GripperControlMsg>("gripper_service");

  auto request = std::make_shared<pdf_beamtime_interfaces::srv::GripperControlMsg::Request>();
  request->command = "CLOSE";
  request->grip = 50;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Results: %d", result.get()->results);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}
