"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""

import bluesky.plan_stubs as bps
import rclpy
from bluesky import RunEngine
from ophyd.sim import det1, motor1
from rclpy.task import Future

from bluesky_ros.ophyd_ros import ActionMovable
from hello_moveit_interfaces.action import PickPlaceRepeat


class PickPlaceRepeatDevice(ActionMovable):
    """Construct a class to build a new device object."""

    action_type = PickPlaceRepeat
    parent = None

    def construct_goal_mesage(self, *args, **kwargs):
        """Populate the goal msg."""
        goal_msg = PickPlaceRepeat.Goal()
        goal_msg.repeats = args[0]
        return goal_msg

    def feedback_callback(self, feedback_msg):
        """Handle regular feedback and print the completion percentage."""
        feedback: PickPlaceRepeat.Feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Number completed: {feedback.num_completed}. "
            f"Percentage completed: {feedback.percent_completed}. "
            f"Percentage current: {feedback.percentage_current}"
        )

    def get_result_callback(self, future: Future):
        """Handle results at the end of a state transition."""
        return super().get_result_callback(future)


def plan(node, det, motor):
    """Planning sequence goes here."""
    yield from bps.mv(motor, 0)
    yield from bps.mv(node, 1)
    yield from bps.read(det)


rclpy.init()

node = PickPlaceRepeatDevice(node_name="pick_place_repeat_device", action_client_name="pick_place_repeat")

RE = RunEngine({})
RE(plan(node, det1, motor1))
