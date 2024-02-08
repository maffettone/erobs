"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from pdf_beamtime_interfaces.action import PickPlaceControlMsg
from pdf_beamtime_interfaces.srv import BlueskyOverrideMsg


class SimpleClient(Node):
    """Send a simple goal request to the action server."""

    def __init__(self):
        """Python init."""
        super().__init__("pdf_beamtime_client")
        self._action_client = ActionClient(self, PickPlaceControlMsg, "pdf_beamtime_action_server")
        self._bluesky_override_service_client = self.create_client(BlueskyOverrideMsg, "pdf_bluesky_override")
        self._goal_handle = None

    def send_goal(self):
        """Send a working goal."""
        goal_msg = PickPlaceControlMsg.Goal()
        goal_msg.pickup_approach = [3.0, -2.2230397, -2.4030528 , -1.627018 , -1.541525, 3.141435]
        goal_msg.pickup = [-4.251453 , -2.2230678 , -2.4030139 , -1.6269628 , -1.0731676, 3.1413669]
        goal_msg.place_approach = [-0.9322131, -1.4574486, -1.2510113 , -3.5442234 , -0.928826 , 3.14093]
        goal_msg.place = [-4.223015, -2.1772848 , -2.330709 , -1.745418, -0.9967396, 3.141905]

        self._action_client.wait_for_server()
        self.get_logger().warn("********** Sending Goal Now *********")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def send_incompatible_goal(self):
        """Send a goal that gets rejected by the move group."""
        goal_msg = PickPlaceControlMsg.Goal()
        goal_msg.pickup_approach = [1.466, -2.042, -2.1293, -2.164, -0.105, 0.0]
        goal_msg.pickup = [1.099, -2.234, -1.728, -2.339, -0.489, -0.035]
        goal_msg.place_approach = [2.618, -2.356, -1.763, -2.216, -2.215, 0.0]
        goal_msg.place = [2.618, -2.566, -1.7453, -2.025, -2.217, 0.0]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def send_self_cancelling_goal(self):
        """Send a goal that gets cancelled after 15 seconds."""
        goal_msg = PickPlaceControlMsg.Goal()
        goal_msg.pickup_approach = [1.466, -2.042, -2.1293, -2.164, -0.105, 0.0]
        goal_msg.pickup = [1.099, -2.234, -1.728, -2.339, -0.489, -0.035]
        goal_msg.place_approach = [2.618, -2.356, -1.763, -2.216, -2.215, 0.0]
        goal_msg.place = [2.618, -2.356, -1.663, -2.416, -2.215, 0.0]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Display the feedback."""
        feedback = feedback_msg.feedback
        self.get_logger().info("Completion percentage: {0} %".format(math.ceil(feedback.status * 100)))

    def goal_response_callback(self, future):
        """Send a cancellation after 15 seconds."""
        self._goal_handle = future.result()
        time.sleep(2.0)
        self.get_logger().warn("********** Goal Canceling Now *********")
        self._goal_handle.cancel_goal_async()

    def cancel_goal(self):
        self._goal_handle = self._send_goal_future.result()
        if self._goal_handle is not None:
            self._action_client._cancel_goal(self._goal_handle)
            print("Goal cancelled successfully.")
        else:
            print("No active goal to cancel.")

    def send_pause(self):
        stop_msg = BlueskyOverrideMsg.Request()
        stop_msg.stop = False
        stop_msg.abort = True
        stop_msg.pause = False
        stop_msg.halt = False
        stop_msg.resume = False
        self.get_logger().warn("********** Goal Pausing Now *********")
        self.response = self._bluesky_override_service_client.call(stop_msg)
        self.get_logger().info(self.response)
        self.get_logger().warn("********** Self Response posted*********")

        return True

    # def send_resume(self):
    #     stop_msg = BlueskyOverrideMsg.Request()
    #     stop_msg.stop = False
    #     stop_msg.abort = False
    #     stop_msg.pause = True
    #     stop_msg.halt = False
    #     stop_msg.resume = False
    #     self.get_logger().warn("********** Goal Resuming Now *********")
    #     self._bluesky_override_service_client.call(stop_msg)


def main(args=None):
    """Python main."""
    rclpy.init(args=args)

    client = SimpleClient()
    client.send_goal()
    time.sleep(4)
    client.cancel_goal()
    # client.send_pause()
    # time.sleep(4)
    # client.send_resume()

    # client.send_incompatible_goal()
    # client.send_self_cancelling_goal()

    # rclpy.spin(client)

    client.destroy_node()


if __name__ == "__main__":
    main()
