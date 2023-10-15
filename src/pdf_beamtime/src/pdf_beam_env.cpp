/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <pdf_beamtime/pdf_beam_env.hpp>

using namespace std::chrono_literals;

PdfBeamEnvironment::PdfBeamEnvironment(
  const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
: node_(std::make_shared<rclcpp::Node>("pdf_beam_env", options))
{
  new_obstacle_client_ = node_->create_client<NewObstacleMsg>("pdf_new_obstacle");
  update_obstacle_client_ = node_->create_client<UpdateObstacleMsg>("pdf_update_obstacles");

  this->change_obstacle("inbeam_platform", "z", 1.5);

  new_obstacle_ = {"detector3", "BOX", 01.3, 0.1, 0.03, 0.0, 0.0, 0.39, 0.5};
  this->add_new_obstacle(new_obstacle_);
}
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr PdfBeamEnvironment::getNodeBaseInterface()
// Expose the node base interface so that the node can be added to a component manager.
{
  return node_->get_node_base_interface();
}

void PdfBeamEnvironment::change_obstacle(
  std::string obstacle_name, std::string property,
  double value)
{
  auto request = std::make_shared<UpdateObstacleMsg::Request>();
  request->name = obstacle_name;
  request->property = property;
  request->value = value;

  while (!this->update_obstacle_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      break;
    }
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }

  // Send the request
  auto result = this->update_obstacle_client_->async_send_request(request);
}

void PdfBeamEnvironment::add_new_obstacle(const Obstacle & new_obstacle_)
{
  auto request = std::make_shared<NewObstacleMsg::Request>();

  // Add the desired obstacle properties to the request
  request->request = "new_obstacle";
  request->name = new_obstacle_.name;
  request->type = new_obstacle_.type;
  request->x = new_obstacle_.x;
  request->y = new_obstacle_.y;
  request->z = new_obstacle_.z;
  request->w = new_obstacle_.w;
  request->h = new_obstacle_.h;
  request->d = new_obstacle_.d;
  request->r = new_obstacle_.r;

  while (!this->new_obstacle_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      break;
    }
    RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
  }

  auto result = this->new_obstacle_client_->async_send_request(request);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Set node parameters using NodeOptions
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto parent_node = std::make_shared<PdfBeamEnvironment>(
    node_options);
  rclcpp::spin(parent_node->getNodeBaseInterface());
  rclcpp::shutdown();
  return 0;
}
