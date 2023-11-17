import bluesky.plan_stubs as bps
import math
import rclpy
from bluesky import RunEngine
from ophyd.sim import det1, motor1
from rclpy.task import Future

from bluesky_ros.ophyd_ros import ActionMovable

# from hello_moveit_interfaces.action import PickPlaceRepeat
from pdf_beamtime_interfaces.action import PickPlaceControlMsg

import pdb
class PickPlaceRepeatDevice(ActionMovable):
    action_type = PickPlaceControlMsg
    parent = None

    def construct_goal_mesage(self, *args, **kwargs):
        goal_msg = PickPlaceControlMsg.Goal()
        goal_msg.pickup_approach = kwargs.get("pickup_approach")
        goal_msg.pickup = kwargs.get("pickup")
        goal_msg.place_approach = kwargs.get("place_approach")
        goal_msg.place = kwargs.get("place")
        return goal_msg

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        pdb.set_trace()
        self.get_logger().info("Completion percentage: {0} %".format(math.ceil(feedback.status * 100)))

    def get_result_callback(self, future: Future):
        return super().get_result_callback(future)


def plan(node, joint_goals):
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
