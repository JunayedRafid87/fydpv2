#!/usr/bin/env python3
"""
Scan Mux Node — FYDP Cv2
=========================
Routes the raw '/scan' topic to '/scan_slam' only when the stepper motor is
flat (during MOVING and STABILIZING states). This blinds Google Cartographer
during the 3D pitching sweep to prevent map corruption.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ScanMuxNode(Node):
    def __init__(self):
        super().__init__('scan_mux_node')

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )
        self.state_sub = self.create_subscription(
            String,
            '/scan_state',
            self._state_callback,
            10
        )

        # Publisher for gated scan
        self.scan_slam_pub = self.create_publisher(LaserScan, '/scan_slam', 10)

        # State tracking (default to MOVING/flat so it maps initially)
        self.current_state = "MOVING"

        self.get_logger().info("Scan Mux Node initialized. Default state: MOVING (forwarding scans).")

    def _state_callback(self, msg):
        state = msg.data.upper()
        if state != self.current_state:
            self.current_state = state
            self.get_logger().info(f"State changed to: {self.current_state}")
            if self.current_state in ["SCANNING", "SCAN_COMPLETE"]:
                self.get_logger().info("LiDAR is pitching. Gating (blinding) Cartographer scan inputs.")
            else:
                self.get_logger().info("LiDAR is flat. Forwarding scans to Cartographer SLAM.")

    def _scan_callback(self, msg):
        # Forward scans to Cartographer in all states except when actively pitching (SCANNING)
        if self.current_state != "SCANNING":
            self.scan_slam_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
