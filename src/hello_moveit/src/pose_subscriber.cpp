#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

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

class PoseSubscriberNode : public rclcpp::Node
{
public:
  PoseSubscriberNode(const std::string & move_group_name = "ur_manipulator")
  : Node("pose_subscriber",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    move_group_interface_(move_group_name)
  {
    subscriber_ = create_subscription<geometry_msgs::msg::Pose>(
      "pose", 10, std::bind(&PoseSubscriberNode::callback, this, _1));


    auto parent_parameters_client =
      std::make_shared<rclcpp::SyncParametersClient>(this, "move_group");
    // Boiler plate wait block
    while (!parent_parameters_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
          get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(get_logger(), "move_group service not available, waiting again...");
    }
    // Get robot config parameters from parameter server
    auto parameters = parent_parameters_client->get_parameters(
      {"robot_description_semantic",
        "robot_description"});
    declare_parameter<std::string>("robot_description_semantic", parameters[0].value_to_string());
    declare_parameter<std::string>("robot_description", parameters[1].value_to_string());


    RCLCPP_INFO(get_logger(), "re-assembling move_group_interface");
    declare_parameter<std::string>("move_group_name", move_group_name);
    move_group_interface_ =
      moveit::planning_interface::MoveGroupInterface(
      shared_from_this(),
      this->get_parameter(
        "move_group_name").as_string());


  }

  void callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "I heard: '%f'", msg->position.x);
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
      RCLCPP_ERROR(get_logger(), "Planning failed!");
    }

  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscriber_;
  moveit::planning_interface::MoveGroupInterface move_group_interface_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseSubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
