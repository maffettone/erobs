"""Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details."""

import rclpy
from rclpy.action import ActionClient, client
from rclpy.node import Node
from rclpy.task import Future

from hello_moveit_interfaces.action import PickPlaceRepeat


class PickPlaceRepeatClient(Node):
    def __init__(self):
        super().__init__("pick_place_repeat_client")
        self._action_client = ActionClient(self, PickPlaceRepeat, "pick_place_repeat")
        self._goal_handle: client.ClientGoalHandle = None
        self._send_goal_future: Future = None
        self._get_result_future: Future = None

    def send_goal(self, repeats: int):
        goal_msg = PickPlaceRepeat.Goal()
        goal_msg.repeats = repeats

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future):
        goal_handle: client.ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        else:
            self.get_logger().info("Goal accepted")

        self._goal_handle = goal_handle
        self._get_result_future: Future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future: Future):
        result: PickPlaceRepeat.Result = future.result().result
        self.get_logger().info(f"Result: {result.success}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback: PickPlaceRepeat.Feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Number completed: {feedback.num_completed}. "
            f"Percentage completed: {feedback.percent_completed}. "
            f"Percentage current: {feedback.percentage_current}"
        )

    def cancel_goal(self):
        if self._goal_handle is not None:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done)

    def cancel_done(self, future: Future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successfully canceled")
        else:
            self.get_logger().info("Goal failed to cancel")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    client = PickPlaceRepeatClient()
    client.send_goal(1)

    rclpy.spin(client)

    client.destroy_node()


if __name__ == "__main__":
    main()
