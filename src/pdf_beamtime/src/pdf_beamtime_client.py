"""Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details."""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import math

from pdf_beamtime_interfaces.action import PickPlaceControlMsg


class SimpleClient(Node):
    # Send a simple goal request to the action server
    def __init__(self):
        super().__init__("pdf_beamtime_client")
        self._action_client = ActionClient(self, PickPlaceControlMsg, "pdf_beamtime_action_server")

    def send_goal(self):
        goal_msg = PickPlaceControlMsg.Goal()
        goal_msg.pickup_approach = [1.466, -2.042, -2.1293, -2.164, -0.105, 0.0]
        goal_msg.pickup = [1.099, -2.234, -1.728, -2.339, -0.489, -0.035]
        goal_msg.place_approach = [2.618, -2.356, -1.763, -2.216, -2.215, 0.0]
        goal_msg.place = [2.618, -2.356, -1.663, -2.416, -2.215, 0.0]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def send_incompatible_goal(self):
        goal_msg = PickPlaceControlMsg.Goal()
        goal_msg.pickup_approach = [1.466, -2.042, -2.1293, -2.164, -0.105, 0.0]
        goal_msg.pickup = [1.099, -2.234, -1.728, -2.339, -0.489, -0.035]
        goal_msg.place_approach = [2.618, -2.356, -1.763, -2.216, -2.215, 0.0]
        goal_msg.place = [2.618, -2.566, -1.7453, -2.025, -2.217, 0.0]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("Completion percentage: {0} %".format(math.ceil(feedback.status * 100)))


def main(args=None):
    rclpy.init(args=args)

    client = SimpleClient()
    client.send_goal()
    # client.send_incompatible_goal()

    rclpy.spin(client)

    client.destroy_node()


if __name__ == "__main__":
    main()
