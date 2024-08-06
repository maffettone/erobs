"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from pdf_beamtime_interfaces.action import PickPlaceControlMsg


class SimpleClient(Node):
    """Send a simple goal request to the action server."""

    def __init__(self):
        """Python init."""
        super().__init__("pdf_beamtime_client")
        self._action_client = ActionClient(self, PickPlaceControlMsg, "pdf_beamtime_action_server")
        self._goal_handle = None

    def send_goal(self):
        """Send a working goal."""
        goal_msg = PickPlaceControlMsg.Goal()

        goal_msg.pickup_approach = [240.86, -63.96, 127.57, -61.50, 98.26, 180.0]
        goal_msg.pickup_approach = [x / 180 * math.pi for x in goal_msg.pickup_approach]

        goal_msg.pickup = [236.97, -54.79, 108.08, -51.21, 94.39, 180.0]
        goal_msg.pickup = [x / 180 * math.pi for x in goal_msg.pickup]

        goal_msg.place_approach = [54.55, -84.79, 120.92, -36.14, 51.53, 180.0]
        goal_msg.place_approach = [x / 180 * math.pi for x in goal_msg.place_approach]

        goal_msg.place = [62.16, -70.19, 100.56, -30.42, 59.15, 180.0]
        goal_msg.place = [x / 180 * math.pi for x in goal_msg.place]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        """Display the feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info("Completion percentage: {0} %".format(math.ceil(feedback.status * 100)))

    def goal_response_callback(self, future):
        """Send a cancellation after 15 seconds."""
        self._goal_handle = future.result()
        time.sleep(15.0)
        self.get_logger().warn("********** Goal Canceling Now *********")
        self._goal_handle.cancel_goal_async()


def main(args=None):
    """Python main."""
    rclpy.init(args=args)

    client = SimpleClient()
    client.send_goal()

    rclpy.spin(client)

    client.destroy_node()


if __name__ == "__main__":
    main()
