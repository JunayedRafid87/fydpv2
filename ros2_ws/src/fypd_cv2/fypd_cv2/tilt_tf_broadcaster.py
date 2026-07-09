#!/usr/bin/env python3
"""
Tilt TF Broadcaster — FYDP Cv2
==============================
Reads tilt angle and orientation from ESP32 serial port.
Broadcasts static translation (no drift) with IMU orientation rotation,
swapping Y and Z axes in software to correct RViz coordinate swap.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster
import serial
import math
import threading


class TiltTFBroadcaster(Node):
    def __init__(self):
        super().__init__('tilt_tf_broadcaster')

        # Declare parameters with defaults
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'base_link')

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher for motion gating
        self.moving_pub = self.create_publisher(Bool, '/moving', 10)

        # Open serial connection to ESP32
        try:
            self.ser = serial.Serial()
            self.ser.port = port
            self.ser.baudrate = baud
            self.ser.timeout = 0.1
            self.ser.dtr = False   # Prevent ESP32-S3 reset on connect
            self.ser.rts = False
            self.ser.open()
            import time
            time.sleep(2)  # Wait for ESP32 to stabilize after port open
            self.ser.reset_input_buffer()  # Flush any boot messages
            self.get_logger().info(f'Connected to ESP32 on {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            self.get_logger().error('Check: ls /dev/ttyACM* /dev/ttyUSB*')
            raise

        # Read serial in a background thread
        self.running = True
        self.serial_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self.serial_thread.start()

    def _serial_reader(self):
        """Continuously read serial data and broadcast TFs."""
        while self.running and rclpy.ok():
            try:
                if not self.ser.is_open:
                    continue
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith('IMU:'):
                    parts = line.split(':')[1].split(',')
                    qw = float(parts[0])
                    qx = float(parts[1])
                    qy = float(parts[2])
                    qz = float(parts[3])
                    
                    if not (math.isnan(qw) or math.isnan(qx) or math.isnan(qy) or math.isnan(qz)):
                        self._broadcast_imu_orientation(qw, qx, qy, qz)
                elif line.startswith('STEP:'):
                    angle_deg = float(line.split(':')[1])
                    self._broadcast_stepper_tilt(angle_deg)
                elif line.startswith('MOVING:'):
                    is_moving_val = int(line.split(':')[1])
                    msg = Bool()
                    msg.data = (is_moving_val == 1)
                    self.moving_pub.publish(msg)
            except Exception:
                pass

    def _broadcast_imu_orientation(self, qw, qx, qy, qz):
        """Broadcast the IMU orientation as map -> base_link (stationary base)."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # Flat translation (no Z climbing, no sliding drift)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.w = qw
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz

        self.tf_broadcaster.sendTransform(t)

    def _broadcast_stepper_tilt(self, angle_deg):
        """Broadcast transform from base_link to tilt_link based on stepper motor tilt."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.child_frame
        t.child_frame_id = 'tilt_link'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05  # 5cm above base origin

        # Stepper tilt rotation around Y-axis
        angle_rad = math.radians(angle_deg)
        t.transform.rotation.w = math.cos(angle_rad / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(angle_rad / 2.0)
        t.transform.rotation.z = 0.0

        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self.running = False
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TiltTFBroadcaster()
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
