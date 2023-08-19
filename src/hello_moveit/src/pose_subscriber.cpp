/*Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details.*/
#include <memory>
#include <chrono>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

/*
Initial efforts to inherit from rclcpp::Node were unsuccessful.
This often came down to the fact that the move_group_interface_ member has no default constructor, so needed to be
initialized in the constructor initializer list. However, there is no SharedPtr until after the constructor body (only
a weak ptr).
So following from this MTC minimal example, I created a "Node" that doesn't inherit from rclcpp::Node, but instead
has a class member that is a node.
(https://github.com/ros-planning/moveit2_tutorials/blob/10113eb3d1a38cff0f3a077aa7bd053069f64f3f
/doc/tutorials/pick_and_place_with_moveit_task_constructor/src/minimal.cpp)
*/

class PoseSubscriberNode
{
public:
  PoseSubscriberNode(
    const std::string & move_group_name = "ur_manipulator",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : node_(std::make_shared<rclcpp::Node>("pose_subscriber_node", options)),
    move_group_interface_(node_, move_group_name)
  {
    subscriber_ = node_->create_subscription<geometry_msgs::msg::Pose>(
      "pose", 10, std::bind(&PoseSubscriberNode::callback, this, _1));
  }

  void callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  // Callback for when a new message is received.
  {
    RCLCPP_INFO(logger, "I heard a new message with x: '%f'", msg->position.x);
    move_group_interface_.setPoseTarget(*msg);
    // Create a plan to that target pose
    auto const [success, plan] = [this] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(this->move_group_interface_.plan(msg));
        return std::make_pair(ok, msg);
      }();
    // Execute the plan
    if (success) {
      this->move_group_interface_.execute(plan);
    } else {
      RCLCPP_ERROR(logger, "Planning failed!");
    }
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface()
  // Expose the node base interface so that the node can be added to a component manager.
  {
    return node_->get_node_base_interface();
  }

  rclcpp::Logger logger = rclcpp::get_logger("pose_subscriber_node");

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Create a ROS logger for main scope
  auto const logger = rclcpp::get_logger("hello_moveit");
  // Create a node for synchronously grabbing params
  auto parameter_client_node = rclcpp::Node::make_shared("param_client");
  auto parent_parameters_client =
    std::make_shared<rclcpp::SyncParametersClient>(parameter_client_node, "move_group");
  // Boiler plate wait block
  while (!parent_parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        logger, "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(logger, "move_group service not available, waiting again...");
  }
  // Get robot config parameters from parameter server
  auto parameters = parent_parameters_client->get_parameters(
    {"robot_description_semantic",
      "robot_description"});

  // Set node parameters using NodeOptions
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides(
  {
    {"robot_description_semantic", parameters[0].value_to_string()},
    {"robot_description", parameters[1].value_to_string()}
  });

  auto parent_node = std::make_shared<PoseSubscriberNode>("ur_manipulator", node_options);
  rclcpp::spin(parent_node->getNodeBaseInterface());
  rclcpp::shutdown();
  return 0;
}
