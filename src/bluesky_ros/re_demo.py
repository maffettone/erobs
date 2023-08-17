import bluesky.plan_stubs as bps
import rclpy
from bluesky import RunEngine
from ophyd.sim import det1, motor1
from rclpy.task import Future

from bluesky_ros.ophyd_ros import ActionMovable
from hello_moveit_interfaces.action import PickPlaceRepeat


class PickPlaceRepeatDevice(ActionMovable):
    action_type = PickPlaceRepeat
    parent = None

    def construct_goal_mesage(self, *args, **kwargs):
        goal_msg = PickPlaceRepeat.Goal()
        goal_msg.repeats = args[0]
        return goal_msg

    def feedback_callback(self, feedback_msg):
        feedback: PickPlaceRepeat.Feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Number completed: {feedback.num_completed}. "
            f"Percentage completed: {feedback.percent_completed}. "
            f"Percentage current: {feedback.percentage_current}"
        )

    def get_result_callback(self, future: Future):
        return super().get_result_callback(future)


def plan(node, det, motor):
    yield from bps.mv(motor, 0)
    yield from bps.mv(node, 1)
    yield from bps.read(det)


rclpy.init()

node = PickPlaceRepeatDevice(node_name="pick_place_repeat_device", action_client_name="pick_place_repeat")

RE = RunEngine({})
RE(plan(node, det1, motor1))
