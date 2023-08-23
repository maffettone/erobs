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


"""Example
def plan(node, det, motor):
    yield from bps.mv(motor, 0)
    yield from bps.abs_set(node, "storage", "inbeam", "PICKUP")
    yield from bps.abs_set(node, "storage", "inbeam", "PLACE")
    yield from bps.mv(motor, 10) # Out of the way
    yield from bps.read(det)
    yield from bps.mv(motor, 0)
     yield from bps.abs_set(node, "inbeam", "storage", "RETURN_PICKUP")
    yield from bps.abs_set(node, "inbeam", "storage", "RETURN_PLACE")
    yield from bps.read(det)

rclpy.init()

node = PickPlaceDevice(node_name="pick_place_device", action_client_name="pick_place")

RE = RunEngine({})
RE(plan(node, det1, motor1))
"""
