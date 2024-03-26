"""Copyright 2023 Brookhaven National Laboratory BSD 3 Clause License. See LICENSE.txt for details."""

import time

import rclpy
from rclpy.node import Node

from pdf_beamtime_interfaces.srv import BlueskyInterruptMsg


class BlueskyInterrupt(Node):
    """Send a service request to the server."""

    def __init__(self):
        """Create the client here."""
        super().__init__("bluesky_interrupt")
        self.client = self.create_client(BlueskyInterruptMsg, "bluesky_interrupt")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = BlueskyInterruptMsg.Request()

    def send_pause_request(self):
        """Populate and send the pause request."""
        self.req.interrupt_type = "PAUSE"
        self.get_logger().info("Pause request sent")
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_resume_request(self):
        """Populate and send the resume request."""
        self.req.interrupt_type = "RESUME"
        self.get_logger().info("Resume request sent")
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_stop_request(self):
        """Populate and send the stop request."""
        self.req.interrupt_type = "STOP"
        self.get_logger().info("Stop request sent")
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_abort_request(self):
        """Populate and send the abort request."""
        self.req.interrupt_type = "ABORT"
        self.get_logger().info("Abort request sent")
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_halt_request(self):
        """Populate and send the halt request."""
        self.req.interrupt_type = "HALT"
        self.get_logger().info("Halt request sent")
        self.future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    """Python main."""
    rclpy.init(args=args)

    minimal_client = BlueskyInterrupt()
    pause_future_results = minimal_client.send_pause_request()
    minimal_client.get_logger().info("Pause request results: " + str(pause_future_results))

    time.sleep(2.0)

    resume_future_results = minimal_client.send_stop_request()
    minimal_client.get_logger().info("Resume request results: " + str(resume_future_results))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
