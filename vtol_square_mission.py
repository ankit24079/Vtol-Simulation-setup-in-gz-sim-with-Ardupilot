#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from mavros_msgs.msg import State, HomePosition
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear
from geometry_msgs.msg import PoseStamped

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from mavros_msgs.msg import Waypoint


class VTOLSquareMissionNode(Node):
    def __init__(self):
        super().__init__("vtol_square_mission_node")

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.state = State()
        self.home = None
        self.current_alt = 0.0

        self.state_sub = self.create_subscription(State, "/mavros/state", self.state_cb, 10)
        self.home_sub = self.create_subscription(HomePosition, "/mavros/home_position/home", self.home_cb, qos)
        self.alt_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.alt_cb,
            qos
        )

        self.wp_clear = self.create_client(WaypointClear, "/mavros/mission/clear")
        self.wp_push = self.create_client(WaypointPush, "/mavros/mission/push")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff_client = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

        self.get_logger().info("VTOL square mission node initialized")

    def state_cb(self, msg):
        self.state = msg

    def home_cb(self, msg):
        self.home = msg
        lat = self.home.geo.latitude
        lon = self.home.geo.longitude
        self.get_logger().info(f"Home pos lat={lat:.7f}, lon={lon:.7f}")

    def alt_cb(self, msg):
        self.current_alt = msg.pose.position.z

    def wait_for_connection(self):
        while rclpy.ok() and not self.state.connected:
            self.get_logger().info("Waiting for MAVROS connection...")
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("MAVROS connected")

    def wait_for_home(self):
        while rclpy.ok() and self.home is None:
            self.get_logger().info("Waiting for home position...")
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("Home position received")

    def clear_mission(self):
        while not self.wp_clear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for waypoint_clear service...")
        req = WaypointClear.Request()
        future = self.wp_clear.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Existing mission cleared")

    def arm(self):
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")
        req = CommandBool.Request(value=True)
        future = self.arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Vehicle armed successfully")

    def set_mode(self, mode):
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for set_mode service...")
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Mode set to {mode}")

    def takeoff(self, altitude=30.0):
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for takeoff service...")
        req = CommandTOL.Request()
        req.altitude = altitude
        req.min_pitch = 0.0
        req.yaw = 0.0
        req.latitude = 0.0
        req.longitude = 0.0
        future = self.takeoff_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Takeoff command sent to {altitude}m")

    def upload_square_mission(self, altitude=30.0, size=0.001):
        lat0 = self.home.geo.latitude
        lon0 = self.home.geo.longitude

        # larger square ~100m side length for fixed wing
        points = [
            (lat0, lon0),
            (lat0, lon0 + size),
            (lat0 + size, lon0 + size),
            (lat0 + size, lon0),
            (lat0, lon0)
        ]

        square = []
        for i, (p_lat, p_lon) in enumerate(points):
            square.append(Waypoint(
                frame=3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
                command=16,  # MAV_CMD_NAV_WAYPOINT
                is_current=(i == 0),
                autocontinue=True,
                x_lat=p_lat,
                y_long=p_lon,
                z_alt=altitude
            ))

        while not self.wp_push.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for waypoint_push service...")
        req = WaypointPush.Request(start_index=0, waypoints=square)
        future = self.wp_push.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() and future.result().success:
            self.get_logger().info("Square mission uploaded")
            return True
        else:
            self.get_logger().error("Mission upload failed")
            return False

    def run(self):
        self.wait_for_connection()
        self.wait_for_home()
        self.clear_mission()

        self.set_mode("GUIDED")
        self.arm()
        self.takeoff(30.0)

        self.get_logger().info("Waiting to reach altitude 30m...")
        timeout = time.time() + 60
        while rclpy.ok() and self.current_alt < 29.0:
            rclpy.spin_once(self, timeout_sec=1.0)
            self.get_logger().info(f"Current altitude: {self.current_alt:.1f} m")
            if time.time() > timeout:
                self.get_logger().error("Timeout waiting to reach altitude")
                return

        self.get_logger().info("Reached 30m, switching to CRUISE")
        self.set_mode("CRUISE")

        if not self.upload_square_mission(30.0, size=0.001):
            return

        self.set_mode("AUTO")
        self.get_logger().info("Executing square mission in AUTO mode")

        while rclpy.ok() and self.state.armed:
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Mission ended. Switching to GUIDED for landing.")
        self.set_mode("GUIDED")

        self.get_logger().info("Performing vertical landing (QLAND)")
        self.set_mode("QLAND")

        while rclpy.ok() and self.state.armed:
            self.get_logger().info("Landing... waiting to disarm")
            rclpy.spin_once(self, timeout_sec=1.0)

        self.get_logger().info("Vehicle landed and disarmed")


def main(args=None):
    rclpy.init(args=args)
    node = VTOLSquareMissionNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
