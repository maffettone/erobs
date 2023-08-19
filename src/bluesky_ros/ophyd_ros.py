from abc import abstractmethod
from typing import Any, Iterable, List

import rclpy
from bluesky.protocols import Movable
from ophyd.status import DeviceStatus
from rclpy.action import ActionClient, client
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future


class ActionStatus(DeviceStatus):
    """
    Track the status of a potentially-lengthy ROS Action using the Bluesky interface.
    """

    def __init__(self, device, **kwargs):
        super().__init__(device, **kwargs)

    def _handle_failure(self):
        self.log.debug("Trying to stop %s", repr(self.device))
        self.device.cancel_goal()


class ActionMovable(Node, Movable):
    def __init__(
        self,
        node_name: str,
        *,
        action_client_name: str,
        context: Context = None,
        cli_args: List[str] = None,
        namespace: str = None,
        use_global_arguments: bool = True,
        enable_rosout: bool = True,
        start_parameter_services: bool = True,
        parameter_overrides: List[Parameter] = None,
        allow_undeclared_parameters: bool = False,
        automatically_declare_parameters_from_overrides: bool = False
    ) -> None:
        super().__init__(
            node_name,
            context=context,
            cli_args=cli_args,
            namespace=namespace,
            use_global_arguments=use_global_arguments,
            enable_rosout=enable_rosout,
            start_parameter_services=start_parameter_services,
            parameter_overrides=parameter_overrides,
            allow_undeclared_parameters=allow_undeclared_parameters,
            automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides,
        )
        self._action_client = ActionClient(self, self.action_type, action_client_name)
        self._goal_handle: client.ClientGoalHandle = None

        # Associated futures objects for ROS and Bluesky
        self._send_goal_future: Future = None
        self._get_result_future: Future = None
        self._bluesky_status: ActionStatus = None
        self._finalize_future = Future()

    """
    ######## Begin ROS Method Block ########
    Some consistent required methods for a ROS api to work with the movable
    """

    @property
    @abstractmethod
    def action_type(self):
        pass

    @abstractmethod
    def construct_goal_mesage(self, *args, **kwargs):
        """Responsible for assembling the goal msg and sending it to the action server."""
        ...

    @abstractmethod
    def get_result_callback(self, future: Future):
        """Any additional callback for when the action is complete."""
        return NotImplemented

    def _stop_spin_callback(self, future: Future):
        """Final callback that manages the execution of all other done Callabacks.
        This ensures a sensible ordering.
        """
        if not future.done():
            self.get_logger().error("Somehow the stop spin callback was called before the future was done...")

        self.get_result_callback(future)

        self._update_bluesky_status(future)
        self.get_logger().info("Setting finalize future to done to stop spin...")
        self._finalize_future.set_result(True)

    @abstractmethod
    def feedback_callback(self, feedback_msg: Any):
        """Callback to work with feedback messages."""
        return NotImplemented

    def cancel_goal(self) -> None:
        if self._goal_handle is not None:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_done)

    def cancel_done(self, future: Future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info("Goal successfully canceled")
        else:
            self.get_logger().info("Goal failed to cancel")

        rclpy.shutdown()

    def _goal_response_callback(self, future: Future):
        self.get_logger().info("Entering goal response callback...")
        goal_handle: client.ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return
        else:
            self.get_logger().info("Goal accepted")

        self._goal_handle = goal_handle
        self._get_result_future: Future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._stop_spin_callback)

    def _send_goal(self, goal: Any) -> None:
        if isinstance(goal, dict):
            goal_msg = self.construct_goal_mesage(**goal)
        elif isinstance(goal, Iterable):
            goal_msg = self.construct_goal_mesage(*goal)
        else:
            goal_msg = self.construct_goal_mesage(goal)

        if not isinstance(goal_msg, self.action_type.Goal):
            raise TypeError("Goal must be of type {}. Received type {}".format(self.action_type.Goal, type(goal_msg)))

        self._action_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info("Sending goal request...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    """
    ####### End ROS Method Block ########
    """
    """
    ####### Begin Bluesky Method Block ########
    """

    def set(self, value) -> ActionStatus:
        self._send_goal(value)
        self._bluesky_status = ActionStatus(self)
        rclpy.spin_until_future_complete(self, self._finalize_future)
        return self._bluesky_status

    def _update_bluesky_status(self, future: Future) -> None:
        self.get_logger().info("Updating bluesky status...")
        if self._bluesky_status is None:
            return

        if future.exception():
            self._bluesky_status.set_exception(future.exception())
        elif future.done():
            self._bluesky_status.set_finished()

    """
    ####### End Bluesky Method Block ########
    """
