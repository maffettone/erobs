"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from pdf_beamtime_interfaces.action import FidPoseControlMsg


class SimpleClient(Node):
    """Send a simple goal request to the action server."""

    def __init__(self):
        """Python init."""
        super().__init__("pdf_beamtime_fidpose_client")
        self._action_client = ActionClient(self, FidPoseControlMsg, "pdf_beamtime_fidpose_action_server")
        self._goal_handle = None

    def send_pickup_goal(self):
        """Send a working goal."""
        goal_msg = FidPoseControlMsg.Goal()

        goal_msg.inbeam_approach = [x / 180 * math.pi for x in [55.10, -51.78, 124.84, -73.16, 52.24, 180.0]]

        goal_msg.inbeam = [x / 180 * math.pi for x in [63.84, -43.13, 98.29, -55.25, 61.00, 180.0]]

        goal_msg.sample_return = False
        goal_msg.sample_id = 150

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def send_return_sample_goal(self):
        """Send a working goal."""
        goal_msg = FidPoseControlMsg.Goal()

        goal_msg.inbeam_approach = [x / 180 * math.pi for x in [55.10, -51.78, 124.84, -73.16, 52.24, 180.0]]

        goal_msg.inbeam = [x / 180 * math.pi for x in [63.84, -43.13, 98.29, -55.25, 61.00, 180.0]]

        goal_msg.sample_return = True
        goal_msg.sample_id = 150

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
    # client.send_pickup_goal()
    client.send_return_sample_goal()

    rclpy.spin(client)

    client.destroy_node()


if __name__ == "__main__":
    main()
