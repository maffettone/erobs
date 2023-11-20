"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""

import math

import bluesky.plan_stubs as bps
from bluesky import RunEngine

from bluesky_ros.ophyd_ros import ActionMovable

from pdf_beamtime_interfaces.action import PickPlaceControlMsg

import rclpy
from rclpy.task import Future


class PickPlaceRepeatDevice(ActionMovable):
    """Construct a class to build a new device object."""

    action_type = PickPlaceControlMsg
    parent = None

    def construct_goal_mesage(self, *args, **kwargs):
        """Populate the goal msg."""
        goal_msg = PickPlaceControlMsg.Goal()
        goal_msg.pickup_approach = kwargs.get("pickup_approach")
        goal_msg.pickup = kwargs.get("pickup")
        goal_msg.place_approach = kwargs.get("place_approach")
        goal_msg.place = kwargs.get("place")
        return goal_msg

    def feedback_callback(self, feedback_msg):
        """Handle regular feedback and print the completion percentage."""
        feedback = feedback_msg.feedback
        self.get_logger().info("Completion percentage: {0} %".format(math.ceil(feedback.status * 100)))

    def get_result_callback(self, future: Future):
        """Handle results at the end of a state transition."""
        return super().get_result_callback(future)


def plan(node, joint_goals):
    """Planning sequence goes here."""
    yield from bps.mv(node, joint_goals)


rclpy.init()

node = PickPlaceRepeatDevice(node_name="pdf_beamtime_control_device", action_client_name="pdf_beamtime_action_server")

joint_goals = {
    "pickup_approach": [1.466, -2.042, -2.1293, -2.164, -0.105, 0.0],
    "pickup": [1.099, -2.234, -1.728, -2.339, -0.489, -0.035],
    "place_approach": [2.618, -2.356, -1.763, -2.216, -2.215, 0.0],
    "place": [2.618, -2.356, -1.663, -2.416, -2.215, 0.0],
}

RE = RunEngine({})
RE(plan(node, joint_goals))
