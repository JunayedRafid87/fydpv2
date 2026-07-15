#!/usr/bin/env python3
"""
Scan to PointCloud2 Converter & Map Accumulator — FYDP Cv2
=========================================================
Converts 2D LaserScan messages into 3D PointCloud2 using dynamic TF transforms,
and accumulates them into a persistent 3D voxel map only during scanning sweeps.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from laser_geometry import LaserProjection
import tf2_sensor_msgs
import sensor_msgs_py.point_cloud2 as pc2
import math


class ScanToPointCloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')

        self.declare_parameter('target_frame', 'map')
        self.target_frame = self.get_parameter('target_frame').value

        # Parameters for point cloud accumulation
        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('invert_z', False)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.invert_z = self.get_parameter('invert_z').value

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Laser projection utility
        self.laser_projector = LaserProjection()

        # Storage for voxel-filtered accumulated map points
        # key: (vx, vy, vz), value: [x, y, z, intensity]
        self.map_points = {}
        self.scan_count = 0

        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.state_sub = self.create_subscription(
            String, '/scan_state', self.state_callback, 10)

        # Publishers
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/pointcloud_3d', 10)
        self.map_pub = self.create_publisher(
            PointCloud2, '/map_3d', 10)

        # State tracking
        self.current_state = "MOVING"

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
        self.get_logger().info(f'Projecting scans into target frame: {self.target_frame}')
        self.get_logger().info(f'Voxel filter resolution set to: {self.voxel_size}m')

    def clear_map_callback(self, request, response):
        self.map_points = {}
        response.success = True
        response.message = "Accumulated 3D map cleared."
        return response

    def state_callback(self, msg):
        self.current_state = msg.data.upper()

    def scan_callback(self, scan_msg):
        try:
            # 1. Project the LaserScan into a PointCloud2 in its own frame (e.g. 'laser')
            cloud_in_laser_frame = self.laser_projector.projectLaser(scan_msg)

            # 2. Lookup the transform from the laser frame to the target frame (typically 'map')
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

            # 3. Transform the PointCloud2 into the target frame (resolving IMU + stepper tilt + SLAM pose)
            cloud_in_target_frame = tf2_sensor_msgs.do_transform_cloud(
                cloud_in_laser_frame, transform)

            # Read raw points from the transformed cloud and cast them to Python tuples
            raw_points = [
                tuple(p) for p in pc2.read_points(
                    cloud_in_target_frame,
                    field_names=['x', 'y', 'z', 'intensity'],
                    skip_nans=True
                )
            ]

            # Create and publish the single-scan point cloud for real-time visual painting
            aligned_points = []
            for p in raw_points:
                x, y, z, intensity = p
                if self.invert_z:
                    z = -z
                aligned_points.append((x, y, z, intensity))

            aligned_cloud = pc2.create_cloud(cloud_in_target_frame.header, self.map_fields, aligned_points)
            aligned_cloud.header.frame_id = self.target_frame
            self.cloud_pub.publish(aligned_cloud)

            # 4. Accumulate points into the 3D map ONLY during stationary active sweeps (SCANNING state)
            if self.current_state == "SCANNING":
                for p in aligned_points:
                    x, y, z, intensity = p
                    vx = int(x / self.voxel_size)
                    vy = int(y / self.voxel_size)
                    vz = int(z / self.voxel_size)
                    key = (vx, vy, vz)
                    
                    # Store unique voxel point
                    self.map_points[key] = [float(x), float(y), float(z), float(intensity)]

            # 5. Periodically publish the accumulated map (every 10 scans)
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
            map_msg.header.frame_id = self.target_frame
            self.map_pub.publish(map_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish accumulated map: {e}")


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
