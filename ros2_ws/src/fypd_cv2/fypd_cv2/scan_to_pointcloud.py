#!/usr/bin/env python3
"""
Scan to PointCloud2 Converter & Map Accumulator — FYDP Cv2
=========================================================
Converts 2D LaserScan messages into 3D PointCloud2 using TF transforms,
aligns them using auto-correlation scan matching on stop, and accumulates
them into a persistent 3D voxel map.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool
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

        # Parameters for point cloud accumulation and saving
        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('enable_motion_gating', False)
        self.declare_parameter('invert_z', False)

        # 90-Degree Overwrite Prevention parameters
        self.declare_parameter('prevent_overwrite', True)
        self.declare_parameter('stable_hit_threshold', 5)
        self.declare_parameter('neighbor_search_radius', 3)

        self.voxel_size = self.get_parameter('voxel_size').value
        self.enable_motion_gating = self.get_parameter('enable_motion_gating').value
        self.invert_z = self.get_parameter('invert_z').value
        self.prevent_overwrite = self.get_parameter('prevent_overwrite').value
        self.stable_hit_threshold = self.get_parameter('stable_hit_threshold').value
        self.neighbor_search_radius = self.get_parameter('neighbor_search_radius').value

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Yaw and translation alignment calibration variables
        self.auto_yaw_offset = 0.0
        self.auto_x_offset = 0.0
        self.auto_y_offset = 0.0
        self.needs_alignment = False

        # Laser projection utility
        self.laser_projector = LaserProjection()

        # Storage for voxel-filtered accumulated map points
        # key: (vx, vy, vz), value: [x, y, z, intensity, hit_count]
        self.map_points = {}
        self.scan_count = 0

        # Subscribe to 2D scan, publish 3D cloud (individual scans)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/pointcloud_3d', 10)

        # Publish the accumulated persistent map
        self.map_pub = self.create_publisher(
            PointCloud2, '/map_3d', 10)

        # Subscribe to motion gating topic
        self.moving_sub = self.create_subscription(
            Bool, '/moving', self.moving_callback, 10)
        self.is_moving = False

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
        self.auto_yaw_offset = 0.0
        self.auto_x_offset = 0.0
        self.auto_y_offset = 0.0
        response.success = True
        response.message = "Map and alignment offsets cleared."
        return response

    def moving_callback(self, msg):
        # Trigger alignment search when transitioned from moving to stationary
        if self.is_moving and not msg.data:
            self.needs_alignment = True
            self.get_logger().info("Rover became stationary. Triggering auto-alignment check on next scan...")
        self.is_moving = msg.data

    def scan_callback(self, scan_msg):
        try:
            # 1. Project the LaserScan into a PointCloud2 in its own frame (e.g. 'laser')
            cloud_in_laser_frame = self.laser_projector.projectLaser(scan_msg)

            # 2. Lookup the transform from the laser frame to the target frame
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

            # Read raw points from the transformed cloud and cast them to Python tuples
            raw_points = [
                tuple(p) for p in pc2.read_points(
                    cloud_in_target_frame,
                    field_names=['x', 'y', 'z', 'intensity'],
                    skip_nans=True
                )
            ]

            # Perform automatic scan-matching correlation alignment (once when stopping)
            if not self.is_moving and self.needs_alignment and self.map_points:
                self.needs_alignment = False
                
                best_dx = self.auto_x_offset
                best_dy = self.auto_y_offset
                best_dtheta = self.auto_yaw_offset
                max_matches = -1
                sub_points = raw_points[::6]  # Subsample for speed

                # Coarse Search: Search X/Y within last_offset +- 0.2m in 4cm steps, Yaw last_yaw +- 10 deg in 2.0 deg steps
                coarse_dxs = [self.auto_x_offset + x * 0.04 for x in range(-5, 6)]
                coarse_dys = [self.auto_y_offset + y * 0.04 for y in range(-5, 6)]
                coarse_yaws = [self.auto_yaw_offset + math.radians(deg) for deg in range(-10, 11, 2)]

                coarse_best_dx = self.auto_x_offset
                coarse_best_dy = self.auto_y_offset
                coarse_best_yaw = self.auto_yaw_offset

                for dyaw in coarse_yaws:
                    cos_a = math.cos(dyaw)
                    sin_a = math.sin(dyaw)
                    # Pre-calculate rotated coordinates to boost loop execution speed
                    rotated_pts = []
                    for p in sub_points:
                        x, y, z, _ = p
                        rx_rot = x * cos_a - y * sin_a
                        ry_rot = x * sin_a + y * cos_a
                        rotated_pts.append((rx_rot, ry_rot, z))

                    for dx in coarse_dxs:
                        for dy in coarse_dys:
                            matches = 0
                            for rx_rot, ry_rot, z in rotated_pts:
                                rx = rx_rot + dx
                                ry = ry_rot + dy
                                vx = int(rx / self.voxel_size)
                                vy = int(ry / self.voxel_size)
                                vz = int(z / self.voxel_size)
                                if (vx, vy, vz) in self.map_points:
                                    matches += 1
                            if matches > max_matches:
                                max_matches = matches
                                coarse_best_dx = dx
                                coarse_best_dy = dy
                                coarse_best_yaw = dyaw

                # Fine Search: Search X/Y within coarse_best +- 0.03m in 1cm steps, Yaw coarse_best_yaw +- 1.5 deg in 0.5 deg steps
                fine_dxs = [coarse_best_dx + x * 0.01 for x in range(-3, 4)]
                fine_dys = [coarse_best_dy + y * 0.01 for y in range(-3, 4)]
                fine_yaws = [coarse_best_yaw + math.radians(deg_half * 0.5) for deg_half in range(-3, 4)]

                max_matches = -1
                for dyaw in fine_yaws:
                    cos_a = math.cos(dyaw)
                    sin_a = math.sin(dyaw)
                    rotated_pts = []
                    for p in sub_points:
                        x, y, z, _ = p
                        rx_rot = x * cos_a - y * sin_a
                        ry_rot = x * sin_a + y * cos_a
                        rotated_pts.append((rx_rot, ry_rot, z))

                    for dx in fine_dxs:
                        for dy in fine_dys:
                            matches = 0
                            for rx_rot, ry_rot, z in rotated_pts:
                                rx = rx_rot + dx
                                ry = ry_rot + dy
                                vx = int(rx / self.voxel_size)
                                vy = int(ry / self.voxel_size)
                                vz = int(z / self.voxel_size)
                                if (vx, vy, vz) in self.map_points:
                                    matches += 1
                            if matches > max_matches:
                                max_matches = matches
                                best_dx = dx
                                best_dy = dy
                                best_dtheta = dyaw

                # Apply alignment if minimum overlap matches found
                if max_matches >= 15:
                    self.auto_x_offset = best_dx
                    self.auto_y_offset = best_dy
                    self.auto_yaw_offset = best_dtheta
                    self.get_logger().info(
                        f"Auto-aligned scan: dx={best_dx:.3f}m, dy={best_dy:.3f}m, yaw={math.degrees(best_dtheta):.2f}° "
                        f"(matched {max_matches} points with existing map)"
                    )
                else:
                    self.get_logger().info(
                        f"Auto-alignment skipped: only {max_matches} matches (needs >=15)"
                    )

            # Apply cumulative alignment offset (X, Y translation and Yaw rotation)
            aligned_points = []
            cos_a = math.cos(self.auto_yaw_offset)
            sin_a = math.sin(self.auto_yaw_offset)
            for p in raw_points:
                x, y, z, intensity = p
                rx = x * cos_a - y * sin_a + self.auto_x_offset
                ry = x * sin_a + y * cos_a + self.auto_y_offset
                aligned_points.append((rx, ry, z, intensity))

            # Create and publish the aligned point cloud
            aligned_cloud = pc2.create_cloud(cloud_in_target_frame.header, self.map_fields, aligned_points)
            self.cloud_pub.publish(aligned_cloud)

            # 4. Accumulate and voxel-filter points
            should_map = True
            if self.enable_motion_gating and self.is_moving:
                should_map = False
                self.get_logger().info("Mapping paused (Rover is moving)", throttle_duration_sec=5.0)

            if should_map:
                for p in aligned_points:
                    x, y, z, intensity = p
                    if self.invert_z:
                        z = -z
                    
                    vx = int(x / self.voxel_size)
                    vy = int(y / self.voxel_size)
                    vz = int(z / self.voxel_size)
                    key = (vx, vy, vz)
                    
                    if key in self.map_points:
                        # Voxel occupied: increment hit count
                        self.map_points[key][4] += 1
                    else:
                        if self.prevent_overwrite:
                            # Avoid adding ghost double-walls from small IMU drifts:
                            # Search neighborhood of given radius
                            has_stable_neighbor = False
                            r = self.neighbor_search_radius
                            for dx in range(-r, r + 1):
                                for dy in range(-r, r + 1):
                                    for dz in range(-r, r + 1):
                                        if dx == 0 and dy == 0 and dz == 0:
                                            continue
                                        neighbor_key = (vx + dx, vy + dy, vz + dz)
                                        if neighbor_key in self.map_points:
                                            # Check if neighbor has reached stable hit count
                                            if self.map_points[neighbor_key][4] >= self.stable_hit_threshold:
                                                has_stable_neighbor = True
                                                break
                                    if has_stable_neighbor:
                                        break
                                if has_stable_neighbor:
                                    break
                            
                            if has_stable_neighbor:
                                continue  # Reject point, too close to a stable existing surface
                        
                        # Add new voxel point with hit count = 1
                        self.map_points[key] = [float(x), float(y), float(z), float(intensity), 1]

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
            # Strip the hit_count element before constructing PointCloud2
            points_list = [p[:4] for p in self.map_points.values()]
            map_msg = pc2.create_cloud(header, self.map_fields, points_list)
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
