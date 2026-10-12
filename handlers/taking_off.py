import time
from functools import partial
from typing import TYPE_CHECKING
from ardupilot_msgs.srv import Takeoff
from ..flight_state import FlightState
from .handler import FlightStateHandler

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode

class TakingOffHandler(FlightStateHandler):
    def handle(self, node: "FlightTestNode") -> None:
        if node.request_in_progress:
            return

        if not node.takeoff_command_sent: 
            self.call_takeoff(node) 
            return

        if node.odometry is None:
            return

        current_altitude = node.odometry.pose.pose.position.z
        altitude_tolerance = 0.5
        if (node.TAKEOFF_ALTITUDE - current_altitude) <= altitude_tolerance:
            node.get_logger().info(f"Altitude {current_altitude:.2f}m reached")
            node.get_logger().info(
            f"Hovering for {node.HOVER_DURATION:.1f} seconds"
        )
            node.hover_started_at = time.monotonic()
            node.state = FlightState.HOVERING
        

    def call_takeoff(self, node: "FlightTestNode") -> None:
        node.takeoff_command_sent = True
        node.request_in_progress = True
        request = Takeoff.Request()
        request.alt = float(node.TAKEOFF_ALTITUDE)
        future = node.takeoff_client.call_async(request)
        future.add_done_callback(partial(self.takeoff_callback, node))
        node.get_logger().info(
            f"Requesting takeoff to {node.TAKEOFF_ALTITUDE:.1f} m"
        )

    def takeoff_callback(self, node: "FlightTestNode", future) -> None:
        node.request_in_progress = False
        try:
            response = future.result()
        except Exception as error:
            node.fail(f"Takeoff service failed: {error}")
            return
        if not response.status:
            node.fail("ArduPilot rejected the takeoff request")
            return
        node.get_logger().info("Takeoff command accepted")
        # node.get_logger().info(
        #     f"Hovering for {node.HOVER_DURATION:.1f} seconds"
        # )
        # node.hover_started_at = time.monotonic()
        # node.state = FlightState.FLYING_TO_TARGET