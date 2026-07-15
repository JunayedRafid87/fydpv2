import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('fypd_cv2')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument(
            'invert_z', default_value='False',
            description='Invert Z axis to fix upside-down height map'),

        # ── 1. LaserScan ➔ PointCloud2 Converter & Map Accumulator ──
        Node(
            package='fypd_cv2',
            executable='scan_to_pointcloud',
            name='scan_to_pointcloud',
            parameters=[{
                'target_frame': 'map',
                'invert_z': LaunchConfiguration('invert_z'),
                'voxel_size': 0.02,
            }],
            output='screen',
        ),

        # ── 2. Scan Multiplexer (gating scans during tilts) ──
        Node(
            package='fypd_cv2',
            executable='scan_mux_node',
            name='scan_mux_node',
            output='screen',
        ),

        # ── 3. Google Cartographer 2D SLAM ──
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'cartographer_2d.lua'
            ],
            remappings=[
                ('/scan', '/scan_slam'),
            ],
        ),

        # ── 4. Cartographer Occupancy Grid Node (2D Floorplan publisher) ──
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'resolution': 0.05,
                'publish_period_sec': 1.0,
            }],
        ),
    ])
