"""Copyright 2023 Brookhaven National Laboratory
BSD 3 Clause License. See LICENSE.txt for details."""
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from pdf_beamtime_interfaces.action import PickPlaceControlMsg


class SimpleClient(Node):
    # Send a simple goal request to the action server
    def __init__(self):
        super().__init__("pdf_beamtime_client")
        self._action_client = ActionClient(self, PickPlaceControlMsg, "pdf_beamtime_action_server")

    def send_goal(self):
        goal_msg = PickPlaceControlMsg.Goal()
        goal_msg.run = True

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    client = SimpleClient()
    client.send_goal()

    rclpy.spin(client)

    client.destroy_node()


if __name__ == "__main__":
    main()
