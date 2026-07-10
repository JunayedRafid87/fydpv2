#!/usr/bin/env python3
"""
Scan to PointCloud2 Converter & Map Accumulator — FYDP Cv2
=========================================================
Converts 2D LaserScan messages into 3D PointCloud2 using TF transforms
(slam_toolbox provides map→odom, IMU provides odom→base_link, etc.)
and accumulates them into a persistent 3D voxel map.

Sweep accumulation is gated by /scan_state:
  SCANNING     → accumulate into both sweep_points and map_points
  SCAN_COMPLETE → publish sweep_points as /sweep_3d, clear sweep buffer
  MOVING / STABILIZING → accumulate into map_points only
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from laser_geometry import LaserProjection
import tf2_sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2


class ScanToPointCloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')

        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter('target_frame').value

        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('invert_z', False)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.invert_z = self.get_parameter('invert_z').value

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Laser projection utility
        self.laser_projector = LaserProjection()

        # Global accumulated map — key: (vx, vy, vz), value: [x, y, z, intensity]
        self.map_points = {}

        # Sweep buffer — same voxel structure, accumulated only during SCANNING
        self.sweep_points = {}

        self.scan_count = 0

        # Current scan state (from stepper controller)
        self.scan_state = ''

        # Subscribe to 2D scan
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publish per-scan 3D cloud (for RViz visualization)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/pointcloud_3d', 10)

        # Publish the accumulated persistent global map
        self.map_pub = self.create_publisher(
            PointCloud2, '/map_3d', 10)

        # Publish completed sweep point cloud
        self.sweep_pub = self.create_publisher(
            PointCloud2, '/sweep_3d', 10)

        # Subscribe to motion gating topic
        self.moving_sub = self.create_subscription(
            Bool, '/moving', self.moving_callback, 10)
        self.is_moving = False

        # Subscribe to scan state from stepper controller
        self.state_sub = self.create_subscription(
            String, '/scan_state', self.state_callback, 10)

        # Service to clear/delete the accumulated map
        self.clear_service = self.create_service(
            Trigger, '/clear_map', self.clear_map_callback)

        # Define PointCloud2 fields
        self.map_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.get_logger().info('ScanToPointCloud node started')
        self.get_logger().info(f'Projecting scans into frame: {self.target_frame}')
        self.get_logger().info(f'Voxel filter resolution set to: {self.voxel_size}m')

    def clear_map_callback(self, request, response):
        self.map_points = {}
        self.sweep_points = {}
        response.success = True
        response.message = "Map and sweep buffer cleared."
        return response

    def moving_callback(self, msg):
        self.is_moving = msg.data

    def state_callback(self, msg):
        new_state = msg.data.strip()
        prev_state = self.scan_state

        # Detect transition into SCAN_COMPLETE
        if new_state == 'SCAN_COMPLETE' and prev_state != 'SCAN_COMPLETE':
            self.publish_sweep()

        self.scan_state = new_state

    def scan_callback(self, scan_msg):
        try:
            # 1. Project the LaserScan into a PointCloud2 in its own frame
            cloud_in_laser_frame = self.laser_projector.projectLaser(scan_msg)

            # 2. Lookup the transform from laser frame to target frame (map)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    scan_msg.header.frame_id,
                    scan_msg.header.stamp,
                    rclpy.duration.Duration(seconds=0.05)
                )
            except Exception:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    scan_msg.header.frame_id,
                    rclpy.time.Time(),
                    rclpy.duration.Duration(seconds=0.05)
                )

            # 3. Transform the PointCloud2 into the target frame
            cloud_in_target_frame = tf2_sensor_msgs.do_transform_cloud(
                cloud_in_laser_frame, transform)

            # Read raw points from the transformed cloud
            raw_points = [
                tuple(p) for p in pc2.read_points(
                    cloud_in_target_frame,
                    field_names=['x', 'y', 'z', 'intensity'],
                    skip_nans=True
                )
            ]

            # Publish per-scan point cloud for RViz
            per_scan_cloud = pc2.create_cloud(
                cloud_in_target_frame.header, self.map_fields, raw_points)
            self.cloud_pub.publish(per_scan_cloud)

            # 4. Accumulate into global map (always — slam_toolbox handles positioning)
            for p in raw_points:
                x, y, z, intensity = p
                if self.invert_z:
                    z = -z

                vx = int(x / self.voxel_size)
                vy = int(y / self.voxel_size)
                vz = int(z / self.voxel_size)
                key = (vx, vy, vz)

                if key not in self.map_points:
                    self.map_points[key] = [float(x), float(y), float(z), float(intensity)]

            # 5. Accumulate into sweep buffer only during SCANNING (and not moving)
            if self.scan_state == 'SCANNING' and not self.is_moving:
                for p in raw_points:
                    x, y, z, intensity = p
                    if self.invert_z:
                        z = -z

                    vx = int(x / self.voxel_size)
                    vy = int(y / self.voxel_size)
                    vz = int(z / self.voxel_size)
                    key = (vx, vy, vz)

                    if key not in self.sweep_points:
                        self.sweep_points[key] = [float(x), float(y), float(z), float(intensity)]

            # 6. Periodically publish the accumulated map (every 10 scans)
            self.scan_count += 1
            if self.scan_count % 10 == 0:
                self.publish_map(cloud_in_target_frame.header)

        except Exception as e:
            self.get_logger().warn(
                f'Could not transform/accumulate scan: {e}', throttle_duration_sec=2.0)

    def publish_map(self, header):
        if not self.map_points:
            return
        try:
            points_list = list(self.map_points.values())
            map_msg = pc2.create_cloud(header, self.map_fields, points_list)
            self.map_pub.publish(map_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish accumulated map: {e}")

    def publish_sweep(self):
        """Publish the accumulated sweep and clear the buffer."""
        if not self.sweep_points:
            self.get_logger().warn("SCAN_COMPLETE received but sweep buffer is empty.")
            return

        num_points = len(self.sweep_points)
        self.get_logger().info(
            f"SCAN_COMPLETE: publishing sweep with {num_points} voxel points.")

        try:
            from builtin_interfaces.msg import Time as TimeMsg
            header = PointCloud2().header
            header.frame_id = self.target_frame
            now = self.get_clock().now().to_msg()
            header.stamp = now

            points_list = list(self.sweep_points.values())
            sweep_msg = pc2.create_cloud(header, self.map_fields, points_list)
            self.sweep_pub.publish(sweep_msg)

            # Clear the sweep buffer for the next scan
            self.sweep_points = {}
        except Exception as e:
            self.get_logger().error(f"Failed to publish sweep: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointCloud()
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
