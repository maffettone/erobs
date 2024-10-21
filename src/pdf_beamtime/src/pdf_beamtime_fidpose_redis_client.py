"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""

import math
import time

import rclpy
import redis

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

    def send_pickup_goal(self, sample_id):
        """Send a working goal."""
        goal_msg = FidPoseControlMsg.Goal()

        goal_msg.inbeam_approach = [x / 180 * math.pi for x in [55.10, -51.78, 124.84, -73.16, 52.24, 180.0]]
        goal_msg.inbeam = [x / 180 * math.pi for x in [63.84, -47.71, 98.22, -50.59, 61.00, 180.0]]

        goal_msg.sample_return = False

        goal_msg.sample_id = sample_id

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def send_return_sample_goal(self, sample_id):
        """Send a working goal."""
        goal_msg = FidPoseControlMsg.Goal()

        goal_msg.inbeam_approach = [x / 180 * math.pi for x in [55.10, -51.78, 124.84, -73.16, 52.24, 180.0]]
        goal_msg.inbeam = [x / 180 * math.pi for x in [63.84, -47.71, 98.22, -50.59, 61.00, 180.0]]

        goal_msg.sample_return = True
        goal_msg.sample_id = sample_id

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

    # Change the sample name to represent the correct sample to be picked.
    sample_name = "sample_1"

    # Read sample ID from the redis server
    redis_client = redis.Redis(host="192.168.56.1", port=6379, db=0)
    tag_key = redis_client.hget("sample_name_index", sample_name).decode("utf-8")
    tag_id = int(redis_client.hget(tag_key, "id"))

    client = SimpleClient()
    client.send_pickup_goal(tag_id)
    # client.send_return_sample_goal(tag_id)

    rclpy.spin(client)
    client.destroy_node()


if __name__ == "__main__":
    main()
