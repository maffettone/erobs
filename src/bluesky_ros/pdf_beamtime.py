import bluesky.plan_stubs as bps
import rclpy
from bluesky import RunEngine
from custom_msgs.action import PickPlace

from bluesky_ros.ophyd_ros import ActionMovable

"""
Action
string sample_name
string target_name
string task_name # 4 options are : PICK_UP, PLACE, RETURN_PICK_UP, RETURN_PLACE
---
bool success
---
int32 completed_precentages
# bool partial_success
int32 total_number_of_stages
"""


class PickPlaceDevice(ActionMovable):
    action_type = PickPlace
    parent = None

    def construct_goal_mesage(self, sample_name, target_name, task_name):
        goal_msg = PickPlace.Goal()
        goal_msg.sample_name = sample_name
        goal_msg.target_name = target_name
        goal_msg.task_name = task_name
        return goal_msg

    def feedback_callback(self, feedback_msg):
        feedback: PickPlace.Feedback = feedback_msg.feedback
        self.get_logger().info(
            f"Total stages: {feedback.total_number_of_stages}. "
            f"Percentage completed: {feedback.completed_precentages}. "
        )

    def get_result_callback(self, future):
        return super().get_result_callback(future)


def iterative_plan(node, n_iter, motor=OT_stage_3_X, safe_position=0.0):  # noqa: F821
    load_position = list(motor.read().values())[0]["value"]
    for i in range(n_iter):
        print(f"Starting iteration {i}")
        yield from bps.abs_set(node, ["holder_shaft_storage", "holder_shaft_inbeam", "PICK_UP"], group="A")
        yield from bps.wait("A")
        yield from bps.abs_set(node, ["holder_shaft_storage", "holder_shaft_inbeam", "PLACE"], group="A")
        yield from bps.wait("A")
        print(f"Moving motor to safe position {safe_position}")
        yield from bps.mv(motor, safe_position)
        yield from bps.wait()
        print(f"Moving motor to load position {load_position}")
        yield from bps.mv(motor, load_position)
        yield from bps.wait()
        print("Returning sample to storage")
        yield from bps.abs_set(node, ["holder_shaft_inbeam", "holder_shaft_storage", "RETURN_PICK_UP"], group="A")
        yield from bps.wait("A")
        yield from bps.abs_set(node, ["holder_shaft_inbeam", "holder_shaft_storage", "RETURN_PLACE"], group="A")
        yield from bps.wait("A")


if __name__ == "main":
    rclpy.init()
    node = PickPlaceDevice(node_name="pick_place_device", action_client_name="erbos_pdf_pick_place_action")
    RE = RunEngine({})
    RE(iterative_plan(node))
    rclpy.shutdown()
