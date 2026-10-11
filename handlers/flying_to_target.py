from typing import TYPE_CHECKING
from .handler import FlightStateHandler
from math import sqrt, pow
from geometry_msgs.msg import TwistStamped
from ..flight_state import FlightState
import time

if TYPE_CHECKING:
    from ..flight_test_node import FlightTestNode

class FlyingToTargetHandler(FlightStateHandler):
    distance_tolerance = 0.25
    def handle(self, node: "FlightTestNode") -> None:
        if node.status is None:
            return

        if node.odometry is None:
            return

        current_x = node.odometry.pose.pose.position.x 
        current_y = node.odometry.pose.pose.position.y

        if not node.target_initialised:
            node.x0 = current_x
            node.y0 = current_y
            
            delta_x = 4.0
            delta_y = 1.4
            node.target_x = node.x0 - delta_x
            node.target_y = node.y0 - delta_y
            node.target_initialised = True

        dx = node.target_x - current_x
        dy = node.target_y - current_y

        distance = sqrt(pow(dx, 2) + pow(dy, 2))
        
        vel_msg = TwistStamped()
        vel_msg.header.stamp = node.get_clock().now().to_msg()
        vel_msg.header.frame_id = "map"

        if distance <= self.distance_tolerance:
            vel_msg.twist.linear.x = 0.0
            vel_msg.twist.linear.y = 0.0
            vel_msg.twist.linear.z = 0.0
            node.velocity_publisher.publish(vel_msg)
            node.get_logger().info(f"REACHED TARGET: x = {current_x}, target_x = {node.target_x}, y = {current_y}, target_y = {node.target_y}")
            node.get_logger().info(f"Distance = {distance}")
            node.hover_started_at = time.monotonic()
            node.state = FlightState.LANDING
            return

        vel_msg.twist.linear.x = float(self.linear_vel(dx))
        vel_msg.twist.linear.y = float(self.linear_vel(dy))
        vel_msg.twist.linear.z = 0.0
        
        node.velocity_publisher.publish(vel_msg)
    
    def linear_vel(self, parametr, constant=0.3):
        return constant * parametr