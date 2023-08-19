/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto client_node = rclcpp::Node::make_shared("param_client");


  auto parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(client_node, "move_group");


  // Boiler plate wait block
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto parameters = parameters_client->get_parameters(
    {"robot_description_semantic",
      "robot_description"});


  std::stringstream ss;
  // Get a few of the parameters just set.
  for (auto & parameter : parameters) {
    ss << "\nParameter name: " << parameter.get_name();
    ss << "\nParameter value (" << parameter.get_type_name() << "): " <<
      parameter.value_to_string();
  }
  std::cout << ss.str();
}

/* An alternative approach may have used some of these components.
// NOT FUNCTIONAL JUST NOTES

auto get_request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
auto get_client = client_node->create_client<rcl_interfaces::srv::GetParameters>(
  "/move_group/get_parameters");
get_request->names.push_back("move_group/robot_description_semantic");
get_request->names.push_back("move_group/robot_description");


  auto parameter = rcl_interfaces::msg::Parameter();
  auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();
  client = this->create_client<rcl_interfaces::srv::SetParametersAtomically>("/move_group");   // E.g.: serviceName = "/turtlesim/set_parameters_atomically"

  parameter.name = parameter_name;   // E.g.: parameter_name = "background_b"
  parameter.value.type = 1        //  bool = 1,    int = 2,        float = 3,     string = 4
    parameter.value.bool_value = true   // .bool_value, .integer_value, .double_value, .string_value

    request->parameters.push_back(parameter);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(get_logger(), "service " << serviceName << " not available, waiting again...");
  }
  auto result = client->async_send_request(request);

*/
